import argparse
import json
import math
import os
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN_CPP = ROOT / "src" / "main.cpp"
MAP_PNG = ROOT / "src" / "map.png"
OUTPUT_HTML = ROOT / "bin" / "auton_viewer.html"


def split_args(text):
    args = []
    current = []
    depth = 0
    in_string = False
    escape = False

    for ch in text:
        if in_string:
            current.append(ch)
            if escape:
                escape = False
            elif ch == "\\":
                escape = True
            elif ch == '"':
                in_string = False
            continue

        if ch == '"':
            in_string = True
            current.append(ch)
        elif ch in "({[":
            depth += 1
            current.append(ch)
        elif ch in ")}]":
            depth -= 1
            current.append(ch)
        elif ch == "," and depth == 0:
            args.append("".join(current).strip())
            current = []
        else:
            current.append(ch)

    if current:
        args.append("".join(current).strip())
    return args


def number(expr, constants=None):
    expr = expr.strip()
    if constants and expr in constants:
        return constants[expr]
    match = re.search(r"[-+]?\d*\.?\d+", expr)
    if not match:
        return None
    return float(match.group(0))


def parse_constants(source):
    """Scans code for float, double, int, and constexpr constant declarations."""
    constants = {}
    pattern = re.compile(
        r"\b(?:float|double|auto|int|constexpr|const)\s+(\w+)\s*=\s*([-+]?\d*\.?\d+)\s*f?;",
        re.IGNORECASE
    )
    for name, val in pattern.findall(source):
        constants[name] = float(val)
    return constants


def parse_poses(source, constants):
    """Parses standard C++ struct initializations of type Pose, e.g., Pose a = {x, y, theta};"""
    poses = {}
    pattern = re.compile(
        r"\bPose\s+(\w+)\s*(?:=\s*)?\{([^}]+)\};"
    )
    for name, raw_args in pattern.findall(source):
        parts = split_args(raw_args)
        if len(parts) >= 2:
            x_val = number(parts[0], constants)
            y_val = number(parts[1], constants)
            theta_val = number(parts[2], constants) if len(parts) >= 3 else None
            poses[name] = {
                "x": x_val if x_val is not None else 0.0,
                "y": y_val if y_val is not None else 0.0,
                "theta": theta_val if theta_val is not None else 0.0
            }
    return poses


def strip_comments(text):
    """Safely strips C++ line and block comments, preserving string literals and raw strings."""
    pattern = re.compile(
        r'(R\"[^(]*\((?:.*?)\)[^\"]*\")|'       # Group 1: C++ Raw string R"(...)"
        r'(\"(?:\\.|[^\"\\])*\")|'               # Group 2: Regular string "..."
        r'(\'(?:\\.|[^\'\\])*\')|'               # Group 3: Character literal '...'
        r'(/\*.*?\*/)|'                          # Group 4: Block comment /*...*/
        r'(//[^\r\n]*)',                         # Group 5: Line comment //...
        re.DOTALL
    )
    def replacer(match):
        if match.group(4) is not None: 
            return ""
        elif match.group(5) is not None: 
            return ""
        else:
            return match.group(0) 
    return pattern.sub(replacer, text)


def extract_simulation(source):
    match = re.search(r"\bvoid\s+simulation\s*\(\s*\)\s*\{", source)
    if not match:
        raise RuntimeError("Could not find simulation() in src/main.cpp")

    start = match.end()
    depth = 1
    i = start
    while i < len(source) and depth > 0:
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
        i += 1

    return source[start : i - 1]


def parse_path_strings(source_or_body):
    """Parses raw paths globally from source to find globally declared path definitions."""
    paths = {}
    pattern = re.compile(
        r"std::string\s+(\w+)\s*=\s*R\"\((.*?)\)\";", re.DOTALL
    )

    for name, raw in pattern.findall(source_or_body):
        points = []
        for line in raw.splitlines():
            parts = [p.strip() for p in line.split(",") if p.strip()]
            if len(parts) < 2:
                continue
            try:
                x_val = float(parts[0])
                y_val = float(parts[1])
                if len(parts) >= 3:
                    theta_val = float(parts[2])
                else:
                    theta_val = points[-1]["theta"] if points else 0.0
                
                points.append(
                    {
                        "x": x_val,
                        "y": y_val,
                        "theta": theta_val,
                    }
                )
            except ValueError:
                pass
        paths[name] = points
    return paths


def find_calls(body):
    calls = []
    i = 0
    prefix = "chassis."
    while True:
        start = body.find(prefix, i)
        if start == -1:
            break
        name_start = start + len(prefix)
        name_match = re.match(r"([A-Za-z_]\w*)\s*\(", body[name_start:])
        if not name_match:
            i = name_start
            continue

        name = name_match.group(1)
        args_start = name_start + name_match.end() - 1
        depth = 0
        end = args_start
        while end < len(body):
            ch = body[end]
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    break
            end += 1

        args = body[args_start + 1 : end]
        calls.append((name, split_args(args)))
        i = end + 1
    return calls


def directed_delta(target, current, direction):
    shortest = ((target - current + 180.0) % 360.0) - 180.0
    if direction == "Auto" or abs(shortest) < 1e-6:
        return shortest
    delta = (target - current) % 360.0
    return delta if direction == "CW" else delta - 360.0


def heading_vectors(theta_deg):
    rad = math.radians(theta_deg)
    forward = (math.sin(rad), math.cos(rad))
    strafe = (math.cos(rad), -math.sin(rad))
    return forward, strafe


def sample_line(start, end, step=0.5):
    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    dtheta = directed_delta(end["theta"], start["theta"], "Auto")
    dist = max(math.hypot(dx, dy), abs(dtheta) / 6.0)
    samples = max(2, int(dist / step) + 1)
    out = []
    for idx in range(1, samples + 1):
        t = idx / samples
        out.append(
            {
                "x": start["x"] + dx * t,
                "y": start["y"] + dy * t,
                "theta": start["theta"] + dtheta * t,
            }
        )
    return out


def parse_direction(args):
    joined = " ".join(args)
    if "CurveDirection::CW" in joined:
        return "CW"
    if "CurveDirection::CCW" in joined:
        return "CCW"
    return "Auto"


def find_lookahead_point(path, curr_pos, lookahead_distance, start_idx):
    """Finds the closest point on the path and looks ahead by lookahead_distance."""
    best_dist = float('inf')
    closest_idx = start_idx
    for idx in range(start_idx, len(path)):
        dist = math.hypot(path[idx]['x'] - curr_pos['x'], path[idx]['y'] - curr_pos['y'])
        if dist < best_dist:
            best_dist = dist
            closest_idx = idx

    accum_dist = 0.0
    lookahead_pt = path[closest_idx]
    for idx in range(closest_idx, len(path) - 1):
        p1 = path[idx]
        p2 = path[idx + 1]
        seg_len = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
        if accum_dist + seg_len >= lookahead_distance:
            ratio = (lookahead_distance - accum_dist) / (seg_len if seg_len > 1e-6 else 1e-6)
            angle_diff = directed_delta(p2['theta'], p1['theta'], "Auto")
            lookahead_pt = {
                'x': p1['x'] + (p2['x'] - p1['x']) * ratio,
                'y': p1['y'] + (p2['y'] - p1['y']) * ratio,
                'theta': p1['theta'] + angle_diff * ratio
            }
            break
        accum_dist += seg_len
    else:
        lookahead_pt = path[-1]

    return lookahead_pt, closest_idx


def simulate(source, body):
    constants = parse_constants(source)
    poses_table = parse_poses(source, constants)
    paths = parse_path_strings(source)
    constants.update(parse_constants(body))
    poses_table.update(parse_poses(body, constants))
    
    calls = find_calls(body)
    poses = [{"x": 0.0, "y": 0.0, "theta": 0.0}]
    events = []
    used_paths = set()

    def get_arg_val_local(args, index, default):
        """Resolves local parameters against the constants symbol table."""
        if index < len(args):
            val = number(args[index], constants)
            if val is not None:
                return val
        return default

    def append_motion(target, label):
        nonlocal poses
        poses.extend(sample_line(poses[-1], target))
        events.append({"label": label, "pose": target})

    for name, args in calls:
        cur = poses[-1]
        if name == "setPose":
            if len(args) == 1:
                pose_name = args[0].strip()
                if pose_name in poses_table:
                    target = dict(poses_table[pose_name])
                else:
                    target = {"x": 0.0, "y": 0.0, "theta": 0.0}
            else:
                target = {
                    "x": get_arg_val_local(args, 0, 0.0),
                    "y": get_arg_val_local(args, 1, 0.0),
                    "theta": get_arg_val_local(args, 2, 0.0),
                }
            poses.append(target)
            events.append({"label": f"setPose({target['x']}, {target['y']}, {target['theta']})", "pose": target})
        elif name == "moveToPoint" and len(args) >= 2:
            tx = get_arg_val_local(args, 0, cur["x"])
            ty = get_arg_val_local(args, 1, cur["y"])
            
            angle_correction = True
            if len(args) >= 4:
                angle_correction = "false" not in args[3].lower()
            
            if angle_correction:
                target_theta = math.degrees(math.atan2(tx - cur["x"], ty - cur["y"]))
            else:
                target_theta = cur["theta"]
                
            target = {
                "x": tx,
                "y": ty,
                "theta": target_theta,
            }
            append_motion(target, f"moveToPoint({target['x']}, {target['y']})")
        elif name == "moveToPose" and len(args) >= 3:
            target = {
                "x": get_arg_val_local(args, 0, cur["x"]),
                "y": get_arg_val_local(args, 1, cur["y"]),
                "theta": get_arg_val_local(args, 2, cur["theta"]),
            }
            append_motion(target, f"moveToPose({target['x']}, {target['y']}, {target['theta']})")
        elif name in ("moveDistance", "strafeDistance") and args:
            distance = get_arg_val_local(args, 0, 0.0)
            forward, strafe = heading_vectors(cur["theta"])
            vx, vy = forward if name == "moveDistance" else strafe
            target = {
                "x": cur["x"] + distance * vx,
                "y": cur["y"] + distance * vy,
                "theta": cur["theta"],
            }
            append_motion(target, f"{name}({distance})")
        elif name == "moveRelative" and len(args) >= 2:
            forward_dist = get_arg_val_local(args, 0, 0.0)
            sideways_dist = get_arg_val_local(args, 1, 0.0)
            forward, strafe = heading_vectors(cur["theta"])
            target = {
                "x": cur["x"] + forward_dist * forward[0] + sideways_dist * strafe[0],
                "y": cur["y"] + forward_dist * forward[1] + sideways_dist * strafe[1],
                "theta": cur["theta"],
            }
            append_motion(target, f"moveRelative({forward_dist}, {sideways_dist})")
        elif name == "turnToHeading" and args:
            target = dict(cur)
            target["theta"] = get_arg_val_local(args, 0, cur["theta"])
            append_motion(target, f"turnToHeading({target['theta']})")
        elif name == "turnToPoint" and len(args) >= 2:
            tx = get_arg_val_local(args, 0, cur["x"])
            ty = get_arg_val_local(args, 1, cur["y"])
            target = dict(cur)
            target["theta"] = math.degrees(math.atan2(tx - cur["x"], ty - cur["y"]))
            append_motion(target, f"turnToPoint({tx}, {ty})")
        elif name == "curveCircle" and len(args) >= 2:
            target_theta = get_arg_val_local(args, 0, cur["theta"])
            radius = abs(get_arg_val_local(args, 1, 0.0))
            direction = parse_direction(args)
            delta = directed_delta(target_theta, cur["theta"], direction)
            side = 1.0 if delta >= 0 else -1.0
            start_rad = math.radians(cur["theta"])
            cx = cur["x"] + side * radius * math.cos(start_rad)
            cy = cur["y"] - side * radius * math.sin(start_rad)
            steps = max(8, int(abs(delta) / 3.0))
            for idx in range(1, steps + 1):
                theta = cur["theta"] + delta * idx / steps
                rad = math.radians(theta)
                poses.append(
                    {
                        "x": cx - side * radius * math.cos(rad),
                        "y": cy + side * radius * math.sin(rad),
                        "theta": theta,
                    }
                )
            events.append({"label": f"curveCircle({target_theta}, {radius}, {direction})", "pose": poses[-1]})
        elif name == "followPathPID":
            joined = " ".join(args)
            path_match = re.search(r"parsePathData\s*\(\s*(\w+)", joined)
            if path_match and path_match.group(1) in paths:
                path_name = path_match.group(1)
                used_paths.add(path_name)
                path = paths[path_name]
                
                if path:
                    lookahead_dist = get_arg_val_local(args, 1, 10.0)
                    
                    heading_mode = "FollowPath"
                    if "HeadingMode::HoldAngle" in joined:
                        heading_mode = "HoldAngle"
                    elif "HeadingMode::CustomAngles" in joined:
                        heading_mode = "CustomAngles"
                        
                    hold_angle_deg = get_arg_val_local(args, 4, 0.0)
                    
                    reversed_bool = False
                    if len(args) >= 6:
                        reversed_bool = "true" in args[5].lower()
                    
                    spatial_points = []
                    if math.hypot(path[0]['x'] - cur['x'], path[0]['y'] - cur['y']) > 0.5:
                        spatial_points.extend(sample_line(cur, path[0], step=0.5))
                    for idx in range(len(path) - 1):
                        spatial_points.extend(sample_line(path[idx], path[idx + 1], step=0.5))
                    
                    simulated_poses = []
                    current_theta = cur["theta"]
                    start_idx = 0
                    
                    for pt in spatial_points:
                        lookahead, start_idx = find_lookahead_point(path, pt, lookahead_dist, start_idx)
                        
                        if heading_mode == "HoldAngle":
                            target_heading = hold_angle_deg
                        elif heading_mode == "CustomAngles":
                            target_heading = lookahead['theta']
                            if reversed_bool:
                                target_heading += 180.0
                        else:  # FollowPath
                            dist_to_end = math.hypot(path[-1]['x'] - pt['x'], path[-1]['y'] - pt['y'])
                            if dist_to_end > 4.0:
                                globalDX = lookahead['x'] - pt['x']
                                globalDY = lookahead['y'] - pt['y']
                                target_heading = math.degrees(math.atan2(globalDX, globalDY))
                                if reversed_bool:
                                    target_heading += 180.0
                            else:
                                if len(path) >= 2:
                                    finalDX = path[-1]['x'] - path[-2]['x']
                                    finalDY = path[-1]['y'] - path[-2]['y']
                                else:
                                    finalDX = 0.0
                                    finalDY = 1.0
                                target_heading = math.degrees(math.atan2(finalDX, finalDY))
                                if reversed_bool:
                                    target_heading += 180.0
                        
                        angle_err = directed_delta(target_heading, current_theta, "Auto")
                        current_theta += angle_err * 0.25
                        current_theta = (current_theta + 180) % 360 - 180
                        
                        simulated_poses.append({
                            'x': pt['x'],
                            'y': pt['y'],
                            'theta': current_theta
                        })
                    
                    if simulated_poses:
                        poses.extend(simulated_poses)
                        events.append({"label": f"followPathPID({path_name}, {heading_mode})", "pose": simulated_poses[-1]})

    if len(poses) <= 2 and paths:
        name, path = next(iter(paths.items()))
        poses.extend(path)
        events.append({"label": f"unused path preview: {name}", "pose": path[-1]})

    return poses, events


def build_html(poses, events, field_size, map_path):
    data = {
        "poses": poses,
        "events": events,
        "fieldSize": field_size,
        "mapSrc": os.path.relpath(Path(map_path).resolve(),
                                  OUTPUT_HTML.parent.resolve()),
    }
    payload = json.dumps(data)
    return f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Autonomous Viewer</title>
  <style>
    html, body {{ margin: 0; height: 100%; background: #111; color: #f7f7f7; font-family: system-ui, sans-serif; }}
    #wrap {{ display: grid; grid-template-rows: 1fr auto; height: 100%; }}
    canvas {{ width: 100%; height: 100%; display: block; background: #222; }}
    #bar {{ display: flex; gap: 12px; align-items: center; padding: 10px 14px; background: #1b1b1b; }}
    button {{ padding: 6px 12px; }}
    input[type=range] {{ flex: 1; }}
    code {{ color: #9ee7ff; }}
  </style>
</head>
<body>
  <div id="wrap">
    <canvas id="field"></canvas>
    <div id="bar">
      <button id="play">Pause</button>
      <input id="scrub" type="range" min="0" max="1000" value="0">
      <span id="readout"></span>
    </div>
  </div>
  <script>
    const sim = {payload};
    const canvas = document.getElementById('field');
    const ctx = canvas.getContext('2d');
    const img = new Image();
    img.src = sim.mapSrc;
    const play = document.getElementById('play');
    const scrub = document.getElementById('scrub');
    const readout = document.getElementById('readout');
    
    // Animation controls
    let running = true;
    let frame = 0;
    let exactFrame = 0;
    let speed = 0.75; // Change this value to adjust playback speed (e.g., 0.5 for half speed, 2.0 for double)

    function resize() {{
      canvas.width = canvas.clientWidth * devicePixelRatio;
      canvas.height = canvas.clientHeight * devicePixelRatio;
    }}

    function toPx(p) {{
      const scale = Math.min(canvas.width, canvas.height) / sim.fieldSize;
      return {{
        x: canvas.width / 2 + p.x * scale,
        y: canvas.height / 2 - p.y * scale,
        scale
      }};
    }}

    function robot(p) {{
      const q = toPx(p);
      
      // Robot dimensions
      const w = 7 * q.scale;
      const l = 7 * q.scale;
      const a = p.theta * Math.PI / 180;
      
      ctx.save();
      ctx.translate(q.x, q.y);
      ctx.rotate(a);
      
      // Draw robot body
      ctx.fillStyle = 'rgba(255, 235, 80, 0.9)';
      ctx.strokeStyle = '#111';
      ctx.lineWidth = 2 * devicePixelRatio;
      ctx.fillRect(-w / 2, -l / 2, w, l);
      ctx.strokeRect(-w / 2, -l / 2, w, l);
      
      // Draw proportional heading triangle
      ctx.fillStyle = '#e43';
      ctx.beginPath();
      // Scale the points of the triangle relative to width (w) and length (l)
      ctx.moveTo(0, -l / 2 - (l * 0.6));            // Tip extends forward proportionally
      ctx.lineTo(w * 0.4, -l / 2 + (l * 0.2));      // Right point of triangle base
      ctx.lineTo(-w * 0.4, -l / 2 + (l * 0.2));     // Left point of triangle base
      ctx.closePath();
      ctx.fill();
      
      ctx.restore();
    }}

    function draw() {{
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      const mapScale = Math.min(canvas.width / img.width, canvas.height / img.height);
      const mw = img.width * mapScale;
      const mh = img.height * mapScale;
      ctx.drawImage(img, (canvas.width - mw) / 2, (canvas.height - mh) / 2, mw, mh);

      ctx.lineWidth = 3 * devicePixelRatio;
      ctx.strokeStyle = 'rgba(0, 255, 190, 0.9)';
      ctx.beginPath();
      sim.poses.forEach((p, i) => {{
        const q = toPx(p);
        if (i === 0) ctx.moveTo(q.x, q.y);
        else ctx.lineTo(q.x, q.y);
      }});
      ctx.stroke();

      const p = sim.poses[Math.min(frame, sim.poses.length - 1)];
      robot(p);
      readout.innerHTML = `<code>x=${{p.x.toFixed(1)}} y=${{p.y.toFixed(1)}} theta=${{p.theta.toFixed(1)}} frame=${{frame}}/${{sim.poses.length - 1}}</code>`;
      scrub.value = sim.poses.length <= 1 ? 0 : Math.round(frame * 1000 / (sim.poses.length - 1));
    }}

    function tick() {{
      if (running) {{
        exactFrame = (exactFrame + speed) % Math.max(sim.poses.length, 1);
        frame = Math.floor(exactFrame);
      }}
      draw();
      requestAnimationFrame(tick);
    }}

    play.onclick = () => {{
      running = !running;
      play.textContent = running ? 'Pause' : 'Play';
    }};
    
    scrub.oninput = () => {{
      running = false;
      play.textContent = 'Play';
      frame = Math.round(Number(scrub.value) * (sim.poses.length - 1) / 1000);
      exactFrame = frame; // Sync up exact floating frame tracker
      draw();
    }};
    
    addEventListener('resize', () => {{ resize(); draw(); }});
    img.onload = () => {{ resize(); tick(); }};
  </script>
</body>
</html>
"""


def main():
    global OUTPUT_HTML

    parser = argparse.ArgumentParser(description="Generate an autonomous path viewer.")
    parser.add_argument("--field-size", type=float, default=144.0,
                        help="field width/height in inches represented by map.png")
    parser.add_argument("--main", default=MAIN_CPP)
    parser.add_argument("--map", default=MAP_PNG)
    parser.add_argument("--out", default=OUTPUT_HTML)
    args = parser.parse_args()

    source = Path(args.main).read_text()
    clean_source = strip_comments(source)
    
    body = extract_simulation(clean_source)
    poses, events = simulate(clean_source, body)

    OUTPUT_HTML = Path(args.out)
    OUTPUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_HTML.write_text(build_html(poses, events, args.field_size, args.map))
    print(f"Wrote {OUTPUT_HTML}")
    print(f"Loaded {len(poses)} simulated poses from simulation()")


if __name__ == "__main__":
    main()