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


def number(expr):
    match = re.search(r"[-+]?\d*\.?\d+", expr)
    if not match:
        return None
    return float(match.group(0))


def get_arg_val(args, index, default):
    """Safely extracts numeric arguments, preserving 0.0 without treating it as falsy."""
    if index < len(args):
        val = number(args[index])
        if val is not None:
            return val
    return default


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


def parse_path_strings(body):
    paths = {}
    pattern = re.compile(
        r"std::string\s+(\w+)\s*=\s*R\"\((.*?)\)\";", re.DOTALL
    )

    for name, raw in pattern.findall(body):
        points = []
        for line in raw.splitlines():
            parts = [p.strip() for p in line.split(",") if p.strip()]
            if len(parts) < 2:
                continue
            try:
                x_val = float(parts[0])
                y_val = float(parts[1])
                # If theta is missing (such as the end points), propagate the last heading
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


def heading_vectors(theta_deg):
    rad = math.radians(theta_deg)
    forward = (math.sin(rad), math.cos(rad))
    strafe = (math.cos(rad), -math.sin(rad))
    return forward, strafe


def sample_line(start, end, step=0.5):
    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    dtheta = end["theta"] - start["theta"]
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


def directed_delta(target, current, direction):
    shortest = ((target - current + 180.0) % 360.0) - 180.0
    if direction == "Auto" or abs(shortest) < 1e-6:
        return shortest
    delta = (target - current) % 360.0
    return delta if direction == "CW" else delta - 360.0


def simulate(body):
    paths = parse_path_strings(body)
    calls = find_calls(body)
    poses = [{"x": 0.0, "y": 0.0, "theta": 0.0}]
    events = []
    used_paths = set()

    def append_motion(target, label):
        nonlocal poses
        poses.extend(sample_line(poses[-1], target))
        events.append({"label": label, "pose": target})

    for name, args in calls:
        cur = poses[-1]
        if name == "setPose" and len(args) >= 2:
            target = {
                "x": get_arg_val(args, 0, 0.0),
                "y": get_arg_val(args, 1, 0.0),
                "theta": get_arg_val(args, 2, 0.0),
            }
            poses.append(target)
            events.append({"label": f"setPose({target['x']}, {target['y']}, {target['theta']})", "pose": target})
        elif name == "moveToPoint" and len(args) >= 2:
            target = {
                "x": get_arg_val(args, 0, cur["x"]),
                "y": get_arg_val(args, 1, cur["y"]),
                "theta": cur["theta"],
            }
            append_motion(target, f"moveToPoint({target['x']}, {target['y']})")
        elif name == "moveToPose" and len(args) >= 3:
            target = {
                "x": get_arg_val(args, 0, cur["x"]),
                "y": get_arg_val(args, 1, cur["y"]),
                "theta": get_arg_val(args, 2, cur["theta"]),
            }
            append_motion(target, f"moveToPose({target['x']}, {target['y']}, {target['theta']})")
        elif name in ("moveDistance", "strafeDistance") and args:
            distance = get_arg_val(args, 0, 0.0)
            forward, strafe = heading_vectors(cur["theta"])
            vx, vy = forward if name == "moveDistance" else strafe
            target = {
                "x": cur["x"] + distance * vx,
                "y": cur["y"] + distance * vy,
                "theta": cur["theta"],
            }
            append_motion(target, f"{name}({distance})")
        elif name == "moveRelative" and len(args) >= 2:
            forward_dist = get_arg_val(args, 0, 0.0)
            sideways_dist = get_arg_val(args, 1, 0.0)
            forward, strafe = heading_vectors(cur["theta"])
            target = {
                "x": cur["x"] + forward_dist * forward[0] + sideways_dist * strafe[0],
                "y": cur["y"] + forward_dist * forward[1] + sideways_dist * strafe[1],
                "theta": cur["theta"],
            }
            append_motion(target, f"moveRelative({forward_dist}, {sideways_dist})")
        elif name == "turnToHeading" and args:
            target = dict(cur)
            target["theta"] = get_arg_val(args, 0, cur["theta"])
            append_motion(target, f"turnToHeading({target['theta']})")
        elif name == "turnToPoint" and len(args) >= 2:
            tx = get_arg_val(args, 0, cur["x"])
            ty = get_arg_val(args, 1, cur["y"])
            target = dict(cur)
            # Standard compass rotation heading calculated using atan2(dx, dy)
            target["theta"] = math.degrees(math.atan2(tx - cur["x"], ty - cur["y"]))
            append_motion(target, f"turnToPoint({tx}, {ty})")
        elif name == "curveCircle" and len(args) >= 2:
            target_theta = get_arg_val(args, 0, cur["theta"])
            radius = abs(get_arg_val(args, 1, 0.0))
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
                    poses.extend(path)
                    events.append({"label": f"followPathPID({path_name})", "pose": path[-1]})

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
    body = extract_simulation(source)
    poses, events = simulate(body)

    OUTPUT_HTML = Path(args.out)
    OUTPUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_HTML.write_text(build_html(poses, events, args.field_size, args.map))
    print(f"Wrote {OUTPUT_HTML}")
    print(f"Loaded {len(poses)} simulated poses from simulation()")


if __name__ == "__main__":
    main()