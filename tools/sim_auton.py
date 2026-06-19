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
    constants = {}
    pattern = re.compile(
        r"\b(?:float|double|auto|int|constexpr|const)\s+(\w+)\s*=\s*([-+]?\d*\.?\d+)\s*f?;",
        re.IGNORECASE
    )
    for name, val in pattern.findall(source):
        constants[name] = float(val)
    return constants


def parse_poses(source, constants):
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


def get_list_arg(parts, index, default, constants):
    if index < len(parts):
        val = number(parts[index], constants)
        if val is not None:
            return val
    return default


def parse_move_params(source, constants):
    params_table = {}
    pattern = re.compile(
        r"\bMoveParams\s+(\w+)\s*(?:=\s*)?\{([^;]+)\}\s*;"
    )
    for name, raw_args in pattern.findall(source):
        raw_args = raw_args.strip()
        if "." in raw_args:
            parsed = parse_inline_params("{" + raw_args + "}", constants)
        else:
            parts = split_args(raw_args)
            parsed = {
                "maxTranslationSpeed": get_list_arg(parts, 0, 127.0, constants),
                "maxRotationSpeed":    get_list_arg(parts, 1, 127.0, constants),
                "minSpeed":            get_list_arg(parts, 2, 0.0,   constants),
                "exitRange":           get_list_arg(parts, 3, 0.5,   constants),
                "earlyExitRange":      get_list_arg(parts, 4, 0.0,   constants),
                "timeout":             get_list_arg(parts, 5, 5000,  constants),
                "async": "false" not in parts[6].lower() if len(parts) >= 7 else True
            }
        params_table[name] = parsed
    return params_table


def parse_inline_params(param_str, constants):
    content = param_str.strip("{} ")
    parts = split_args(content)

    params = {
        "maxTranslationSpeed": 127.0,
        "maxRotationSpeed":    127.0,
        "minSpeed":            0.0,
        "exitRange":           0.5,
        "earlyExitRange":      0.0,
        "timeout":             5000,
        "async":               True
    }

    for p in parts:
        kv_match = re.match(r"\.?(\w+)\s*[=:]\s*(.+)", p.strip())
        if kv_match:
            key     = kv_match.group(1).strip()
            val_str = kv_match.group(2).strip()
            if key == "async":
                params["async"] = "false" not in val_str.lower()
            else:
                val = number(val_str, constants)
                if val is not None:
                    params[key] = val
    return params


def parse_scheduler(source, name):
    pattern = re.compile(rf"chassis\.set{name}Gains\s*\(\s*\{{(.*?)\}}\s*\)", re.DOTALL)
    match = pattern.search(source)
    schedules = []
    if match:
        step_pattern = re.compile(r"\{\s*([-+]?\d*\.?\d+)[fF]?\s*,\s*\{\s*([^}]+)\s*\}\s*\}")
        for raw_thresh, raw_gains in step_pattern.findall(match.group(1)):
            try:
                thresh = float(raw_thresh)
                gain_parts = [p.strip() for p in raw_gains.split(",")]
                gains = [float(number(p) or 0.0) for p in gain_parts]
                while len(gains) < 5:
                    gains.append(0.0)
                schedules.append({
                    "threshold": thresh,
                    "kP":        gains[0],
                    "kI":        gains[1],
                    "kD":        gains[2],
                    "kF":        gains[3],
                    "slew":      gains[4]
                })
            except Exception:
                pass
    schedules.sort(key=lambda s: s["threshold"])
    return schedules


def strip_comments(text):
    pattern = re.compile(
        r'(R\"[^(]*\((?:.*?)\)[^\"]*\")|'
        r'(\"(?:\\.|[^\"\\])*\")|'
        r'(\'/.*?/\')|'
        r'(/\*.*?\*/)|'
        r'(//[^\r\n]*)',
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


def strip_autonomous(source):
    match = re.search(r"\bvoid\s+autonomous\s*\(\s*\)\s*\{", source)
    if not match:
        return source
    start = match.end()
    depth = 1
    i = start
    while i < len(source) and depth > 0:
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
            if depth == 0:
                break
        i += 1
    return source[:match.start()] + source[i:]


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
    paths = {}
    pattern = re.compile(
        r"\b(?:std::)?string\s+(\w+)\s*=\s*R\"\((.*?)\)\";", re.DOTALL
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
                points.append({"x": x_val, "y": y_val, "theta": theta_val})
            except ValueError:
                pass
        paths[name] = points

    vector_pattern = re.compile(
        r"\b(?:std::)?vector\s*<\s*PathPoint\s*>\s+(\w+)\s*(?:=\s*)?\{([^;]+)\}\s*;", re.DOTALL
    )
    for name, raw in vector_pattern.findall(source_or_body):
        points = []
        point_pattern = re.compile(r"\{\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*(?:,\s*([-+]?\d*\.?\d+)\s*)?\}")
        for x_val, y_val, theta_val in point_pattern.findall(raw):
            points.append({
                "x":     float(x_val),
                "y":     float(y_val),
                "theta": float(theta_val) if theta_val else (points[-1]["theta"] if points else 0.0)
            })
        if points:
            paths[name] = points

    return paths


def find_all_calls(body):
    calls = []
    i = 0
    prefixes = ["chassis.", "chassis->", "pros::Task::delay", "pros::delay", "delay"]

    while i < len(body):
        next_start = -1
        found_prefix = ""
        for p in prefixes:
            idx = body.find(p, i)
            if idx != -1:
                if p == "delay":
                    if idx > 0 and (body[idx-1].isalnum() or body[idx-1] == '_'):
                        continue
                    if idx + len(p) < len(body) and (body[idx+len(p)].isalnum() or body[idx+len(p)] == '_'):
                        continue
                if next_start == -1 or idx < next_start:
                    next_start = idx
                    found_prefix = p

        if next_start == -1:
            break

        name_start = next_start + len(found_prefix)

        if found_prefix in ("chassis.", "chassis->"):
            name_match = re.match(r"([A-Za-z_]\w*)\s*\(", body[name_start:])
            if not name_match:
                i = name_start
                continue
            name      = name_match.group(1)
            args_start = name_start + name_match.end() - 1
        else:
            name = "delay"
            args_start = name_start
            whitespace_match = re.match(r"\s*\(", body[args_start:])
            if whitespace_match:
                args_start += whitespace_match.end() - 1
            else:
                i = name_start
                continue

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

        args_str = body[args_start + 1 : end]
        calls.append((name, split_args(args_str)))
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
    strafe  = (math.cos(rad), -math.sin(rad))
    return forward, strafe


def sample_line(start, end, step=0.5):
    dx     = end["x"] - start["x"]
    dy     = end["y"] - start["y"]
    dtheta = directed_delta(end["theta"], start["theta"], "Auto")
    dist   = max(math.hypot(dx, dy), abs(dtheta) / 6.0)
    samples = max(2, int(dist / step) + 1)
    out = []
    for idx in range(1, samples + 1):
        t = idx / samples
        out.append({
            "x":     start["x"] + dx * t,
            "y":     start["y"] + dy * t,
            "theta": start["theta"] + dtheta * t,
        })
    return out


def parse_direction(args):
    joined = " ".join(args)
    if "CurveDirection::CW" in joined:
        return "CW"
    if "CurveDirection::CCW" in joined:
        return "CCW"
    return "Auto"


def find_lookahead_point(path, curr_pos, lookahead_distance, start_idx):
    best_dist   = float('inf')
    closest_idx = start_idx
    for idx in range(start_idx, len(path)):
        dist = math.hypot(path[idx]['x'] - curr_pos['x'], path[idx]['y'] - curr_pos['y'])
        if dist < best_dist:
            best_dist   = dist
            closest_idx = idx

    accum_dist  = 0.0
    lookahead_pt = path[closest_idx]
    for idx in range(closest_idx, len(path) - 1):
        p1      = path[idx]
        p2      = path[idx + 1]
        seg_len = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
        if accum_dist + seg_len >= lookahead_distance:
            ratio      = (lookahead_distance - accum_dist) / (seg_len if seg_len > 1e-6 else 1e-6)
            angle_diff = directed_delta(p2['theta'], p1['theta'], "Auto")
            lookahead_pt = {
                'x':     p1['x'] + (p2['x'] - p1['x']) * ratio,
                'y':     p1['y'] + (p2['y'] - p1['y']) * ratio,
                'theta': p1['theta'] + angle_diff * ratio
            }
            break
        accum_dist += seg_len
    else:
        lookahead_pt = path[-1]

    return lookahead_pt, closest_idx


def get_cumulative_distances(trajectory):
    distances = [0.0]
    accum = 0.0
    for i in range(1, len(trajectory)):
        p1 = trajectory[i - 1]
        p2 = trajectory[i]
        accum += math.hypot(p2["x"] - p1["x"], p2["y"] - p1["y"])
        distances.append(accum)
    return distances


def resolve_path_name(source, arg_str):
    arg_str = arg_str.strip()

    inline_match = re.search(r"parsePathData\s*\(\s*(\w+)", arg_str)
    if inline_match:
        return inline_match.group(1)

    var_name = arg_str
    if re.match(r"^[A-Za-z_]\w*$", var_name):
        pattern = re.compile(
            r"\b" + re.escape(var_name) + r"\s*(?:=\s*|\(\s*|\{\s*)parsePathData\s*\(\s*(\w+)",
            re.IGNORECASE
        )
        var_match = pattern.search(source)
        if var_match:
            return var_match.group(1)
        return var_name

    return None


def resolve_path_scale(source, arg_str, constants):
    inline_match = re.search(r"parsePathData\s*\(\s*\w+\s*,\s*([^)]+)\)", arg_str)
    if inline_match:
        scale_arg = inline_match.group(1).strip().lower()
        return 39.3700787 if "true" in scale_arg or (number(scale_arg, constants) and number(scale_arg, constants) != 0) else 1.0

    var_name = arg_str.strip()
    if re.match(r"^[A-Za-z_]\w*$", var_name):
        pattern = re.compile(
            r"\b" + re.escape(var_name) + r"\s*(?:=\s*|\(\s*|\{\s*)parsePathData\s*\(\s*\w+\s*,\s*([^)]+)\)",
            re.IGNORECASE
        )
        var_match = pattern.search(source)
        if var_match:
            scale_arg = var_match.group(1).strip().lower()
            return 39.3700787 if "true" in scale_arg or (number(scale_arg, constants) and number(scale_arg, constants) != 0) else 1.0

    return 1.0


class PythonGainScheduler:
    def __init__(self, schedules, default_kp, default_kd, default_slew):
        self.schedules   = schedules
        self.default_kp  = default_kp
        self.default_kd  = default_kd
        self.default_slew = default_slew

    def get_gains(self, error):
        if not self.schedules:
            return {"kP": self.default_kp, "kI": 0.0, "kD": self.default_kd, "kF": 0.0, "slew": self.default_slew}

        abs_err = abs(error)
        if abs_err <= self.schedules[0]["threshold"]:
            return self.schedules[0]
        if abs_err >= self.schedules[-1]["threshold"]:
            return self.schedules[-1]

        for i in range(len(self.schedules) - 1):
            lo = self.schedules[i]
            hi = self.schedules[i + 1]
            if lo["threshold"] <= abs_err <= hi["threshold"]:
                t = (abs_err - lo["threshold"]) / (hi["threshold"] - lo["threshold"])
                return {
                    "kP":   lo["kP"]   + t * (hi["kP"]   - lo["kP"]),
                    "kI":   lo["kI"]   + t * (hi["kI"]   - lo["kI"]),
                    "kD":   lo["kD"]   + t * (hi["kD"]   - lo["kD"]),
                    "kF":   0.0,
                    "slew": lo["slew"] + t * (hi["slew"] - lo["slew"])
                }
        return self.schedules[-1]


class PythonPID:
    def __init__(self, scheduler):
        self.scheduler    = scheduler
        self.prev_error   = 0.0
        self.integral     = 0.0
        self.prev_output  = 0.0

    def update(self, error, dt):
        gains      = self.scheduler.get_gains(error)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.integral += error * dt
        self.integral  = max(-100.0, min(100.0, self.integral))

        output = (error * gains["kP"]) + (self.integral * gains["kI"]) + (derivative * gains["kD"])

        if gains["slew"] > 0:
            change = output - self.prev_output
            if abs(change) > gains["slew"]:
                output = self.prev_output + math.copysign(gains["slew"], change)

        self.prev_error  = error
        self.prev_output = output
        return output


def run_motion_sim(start_state, tx, ty, target_theta, params, angle_correction, x_sched, y_sched, theta_sched, dt=0.01):
    x_pid = PythonPID(x_sched)
    y_pid = PythonPID(y_sched)
    t_pid = PythonPID(theta_sched)

    state   = dict(start_state)
    history = []

    settle_start = None
    settle_time  = 0.100

    max_time         = params.get("timeout", 5000) / 1000.0
    elapsed          = 0.0
    voltage_to_ips   = 60.0 / 127.0
    voltage_to_dps   = 360.0 / 127.0
    drag_coeff       = 0.15

    while elapsed < max_time:
        ex       = tx - state["x"]
        ey       = ty - state["y"]
        dist_err = math.hypot(ex, ey)

        early_exit = params.get("earlyExitRange", 0.0)
        if early_exit > 0.0 and dist_err <= early_exit:
            break

        pos_settled = dist_err < params.get("exitRange", 0.5)

        if angle_correction:
            if dist_err < 2.0 and target_theta is None:
                angle_error = 0.0
            else:
                calc_theta  = math.degrees(math.atan2(ex, ey))
                angle_error = directed_delta(calc_theta, state["theta"], "Auto")
        else:
            angle_error = directed_delta(
                target_theta if target_theta is not None else state["theta"],
                state["theta"], "Auto"
            )

        angle_settled = abs(angle_error) < 2.0

        if target_theta is None and angle_correction:
            settled = pos_settled
        else:
            settled = pos_settled and angle_settled

        if settled:
            if settle_start is None:
                settle_start = elapsed
            if elapsed - settle_start >= settle_time:
                break
        else:
            settle_start = None

        outX_g = x_pid.update(ex, dt)
        outY_g = y_pid.update(ey, dt)
        outT   = t_pid.update(angle_error, dt) if (angle_correction or target_theta is not None) else 0.0

        mag       = math.hypot(outX_g, outY_g)
        max_trans = params.get("maxTranslationSpeed", 127.0)
        min_speed = params.get("minSpeed", 0.0)

        if mag > 1e-3 and mag < min_speed and dist_err > params.get("exitRange", 0.5):
            scale   = min_speed / mag
            outX_g *= scale
            outY_g *= scale
        if mag > max_trans:
            scale   = max_trans / mag
            outX_g *= scale
            outY_g *= scale

        if dist_err < 2.0 and target_theta is None and angle_correction:
            outT = 0.0

        max_rot = params.get("maxRotationSpeed", 127.0)
        outT    = max(-max_rot, min(max_rot, outT))

        target_vx    = outX_g * voltage_to_ips
        target_vy    = outY_g * voltage_to_ips
        target_omega = outT   * voltage_to_dps

        state["vx"] += (target_vx    - state["vx"]) * drag_coeff
        state["vy"] += (target_vy    - state["vy"]) * drag_coeff
        state["w"]  += (target_omega - state["w"])  * drag_coeff

        state["x"]     += state["vx"] * dt
        state["y"]     += state["vy"] * dt
        state["theta"] += state["w"]  * dt
        state["theta"]  = (state["theta"] + 180) % 360 - 180

        history.append({
            "x": state["x"], "y": state["y"], "theta": state["theta"],
            "vx": state["vx"], "vy": state["vy"], "w": state["w"]
        })

        elapsed += dt

    return history, state


def run_path_sim(start_state, path, lookahead_dist, params, heading_mode, hold_angle_deg, reversed_bool, x_sched, y_sched, theta_sched, dt=0.01):
    x_pid = PythonPID(x_sched)
    y_pid = PythonPID(y_sched)
    t_pid = PythonPID(theta_sched)

    state   = dict(start_state)
    history = []

    settle_start   = None
    settle_time    = 0.120
    max_time       = params.get("timeout", 5000) / 1000.0
    elapsed        = 0.0
    voltage_to_ips = 60.0 / 127.0
    voltage_to_dps = 360.0 / 127.0
    drag_coeff     = 0.15

    N = len(path)
    if N < 2:
        return history, state

    arc_len = [0.0] * N
    for i in range(1, N):
        dx = path[i]["x"] - path[i-1]["x"]
        dy = path[i]["y"] - path[i-1]["y"]
        arc_len[i] = arc_len[i-1] + math.hypot(dx, dy)

    total_path_len = arc_len[N-1]

    def sample_path(s):
        s = max(0.0, min(s, total_path_len))
        lo = 0
        hi = N - 2
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if arc_len[mid] <= s:
                lo = mid
            else:
                hi = mid - 1
        seg_len = arc_len[lo+1] - arc_len[lo]
        t = (s - arc_len[lo]) / seg_len if seg_len > 1e-6 else 0.0
        p_lo = path[lo]
        p_hi = path[lo+1]
        return {
            "x": p_lo["x"] + t * (p_hi["x"] - p_lo["x"]),
            "y": p_lo["y"] + t * (p_hi["y"] - p_lo["y"]),
            "theta": p_lo["theta"] + t * directed_delta(p_hi["theta"], p_lo["theta"], "Auto")
        }

    def path_tangent_deg(s):
        s = max(0.0, min(s, total_path_len))
        lo = 0
        hi = N - 2
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if arc_len[mid] <= s:
                lo = mid
            else:
                hi = mid - 1
        dx = path[lo+1]["x"] - path[lo]["x"]
        dy = path[lo+1]["y"] - path[lo]["y"]
        return math.degrees(math.atan2(dx, dy))

    path_progress = 0.0
    prev_pose = {"x": state["x"], "y": state["y"]}
    locked_heading = hold_angle_deg if heading_mode == "HoldAngle" else state["theta"]

    while elapsed < max_time:
        dx_odom = state["x"] - prev_pose["x"]
        dy_odom = state["y"] - prev_pose["y"]
        dist = math.hypot(dx_odom, dy_odom)

        if dist > 1e-4:
            tangent_deg = path_tangent_deg(path_progress)
            tangent_rad = math.radians(tangent_deg)
            tx = math.sin(tangent_rad)
            ty = math.cos(tangent_rad)
            along = dx_odom * tx + dy_odom * ty
            along = max(along, -0.05 * dist)
            path_progress = max(0.0, min(path_progress + along, total_path_len))

        prev_pose = {"x": state["x"], "y": state["y"]}

        on_path = sample_path(path_progress)
        ex_error = state["x"] - on_path["x"]
        ey_error = state["y"] - on_path["y"]

        tangent_deg = path_tangent_deg(path_progress)
        tangent_rad = math.radians(tangent_deg)
        tx = math.sin(tangent_rad)
        ty = math.cos(tangent_rad)
        s_err = ex_error * tx + ey_error * ty

        correction = max(-lookahead_dist * 0.25, min(lookahead_dist * 0.25, s_err * 0.05))
        path_progress = max(0.0, min(path_progress + correction, total_path_len))

        lookahead_s = min(path_progress + lookahead_dist, total_path_len)
        lookahead = sample_path(lookahead_s)

        heading_rad = math.radians(state["theta"])
        forward_x = math.sin(heading_rad)
        forward_y = math.cos(heading_rad)
        strafe_x = math.cos(heading_rad)
        strafe_y = -math.sin(heading_rad)

        global_dx = lookahead["x"] - state["x"]
        global_dy = lookahead["y"] - state["y"]

        local_forward = global_dx * forward_x + global_dy * forward_y
        local_strafe = global_dx * strafe_x + global_dy * strafe_y

        if reversed_bool:
            local_forward = -local_forward
            local_strafe = -local_strafe

        dist_to_end = total_path_len - path_progress

        if heading_mode == "FollowPath":
            tangent_s = min(path_progress + 2.0, total_path_len)
            target_heading = path_tangent_deg(tangent_s)
            if reversed_bool:
                target_heading += 180.0
        elif heading_mode == "HoldAngle":
            target_heading = locked_heading
        elif heading_mode == "CustomAngles":
            target_heading = lookahead["theta"]
            if reversed_bool:
                target_heading += 180.0
        else:
            target_heading = state["theta"]

        angle_error = directed_delta(target_heading, state["theta"], "Auto")

        early_exit = params.get("earlyExitRange", 0.0)
        if early_exit > 0.0 and dist_to_end <= early_exit:
            break

        settled_pos = dist_to_end < params.get("exitRange", 0.5)
        settled_angle = abs(angle_error) < 2.0

        if settled_pos and settled_angle:
            if settle_start is None:
                settle_start = elapsed
            if elapsed - settle_start >= settle_time:
                break
        else:
            if dist_to_end > params.get("exitRange", 0.5) * 1.5 or abs(angle_error) > 4.0:
                settle_start = None

        forward = y_pid.update(local_forward, dt)
        strafe = x_pid.update(local_strafe, dt)
        turn = t_pid.update(angle_error, dt)

        translational_mag = math.hypot(forward, strafe)
        max_trans = params.get("maxTranslationSpeed", 127.0)
        min_speed = params.get("minSpeed", 0.0)

        if translational_mag > max_trans:
            scale = max_trans / translational_mag
            forward *= scale
            strafe *= scale

        if translational_mag > 1e-3 and translational_mag < min_speed and dist_to_end > params.get("exitRange", 0.5):
            scale = min_speed / translational_mag
            forward *= scale
            strafe *= scale

        max_rot = params.get("maxRotationSpeed", 127.0)
        turn = max(-max_rot, min(max_rot, turn))

        total = abs(forward) + abs(strafe) + abs(turn)
        if total > max_trans:
            scale = max_trans / total
            forward *= scale
            strafe *= scale
            turn *= scale

        target_vx = (strafe * strafe_x + forward * forward_x) * voltage_to_ips
        target_vy = (strafe * strafe_y + forward * forward_y) * voltage_to_ips
        target_omega = turn * voltage_to_dps

        state["vx"] += (target_vx - state["vx"]) * drag_coeff
        state["vy"] += (target_vy - state["vy"]) * drag_coeff
        state["w"] += (target_omega - state["w"]) * drag_coeff

        state["x"] += state["vx"] * dt
        state["y"] += state["vy"] * dt
        state["theta"] += state["w"] * dt
        state["theta"] = (state["theta"] + 180) % 360 - 180

        history.append({
            "x": state["x"], "y": state["y"], "theta": state["theta"],
            "vx": state["vx"], "vy": state["vy"], "w": state["w"]
        })

        elapsed += dt

    return history, state


def run_curve_circle_sim(start_state, target_theta, radius, direction, params, x_sched, y_sched, theta_sched, dt=0.01):
    x_pid = PythonPID(x_sched)
    y_pid = PythonPID(y_sched)
    t_pid = PythonPID(theta_sched)

    state   = dict(start_state)
    history = []

    settle_start   = None
    settle_time    = 0.120
    max_time       = params.get("timeout", 5000) / 1000.0
    elapsed        = 0.0
    voltage_to_ips = 60.0 / 127.0
    voltage_to_dps = 360.0 / 127.0
    drag_coeff     = 0.15

    sp_x, sp_y, sp_theta = state["x"], state["y"], state["theta"]

    init_err        = directed_delta(target_theta, sp_theta, direction)
    dir_sign        = 1.0 if init_err >= 0 else -1.0
    arc_radius      = abs(radius)
    max_curve_trans = min(params.get("maxTranslationSpeed", 127.0), 60.0)
    max_curve_rot   = min(params.get("maxRotationSpeed",    127.0), 70.0)

    start_rad = math.radians(sp_theta)
    center_x  = sp_x + dir_sign * arc_radius * math.cos(start_rad)
    center_y  = sp_y - dir_sign * arc_radius * math.sin(start_rad)

    target_rad = math.radians(target_theta)
    final_x    = center_x - dir_sign * arc_radius * math.cos(target_rad)
    final_y    = center_y + dir_sign * arc_radius * math.sin(target_rad)

    while elapsed < max_time:
        angle_error    = directed_delta(target_theta, state["theta"], direction)
        final_dist_err = math.hypot(final_x - state["x"], final_y - state["y"])

        early_exit = params.get("earlyExitRange", 0.0)
        if early_exit > 0.0 and final_dist_err <= early_exit:
            break

        pos_settled   = final_dist_err < params.get("exitRange", 0.5)
        angle_settled = abs(angle_error) < 2.0

        if pos_settled and angle_settled:
            if settle_start is None:
                settle_start = elapsed
            if elapsed - settle_start >= settle_time:
                break
        else:
            settle_start = None

        to_center_x    = center_x - state["x"]
        to_center_y    = center_y - state["y"]
        dist_to_center = math.hypot(to_center_x, to_center_y)
        radius_error   = dist_to_center - arc_radius

        rad          = math.radians(state["theta"])
        cosH         = math.cos(rad)
        sinH         = math.sin(rad)
        center_local_x = to_center_x * cosH - to_center_y * sinH
        center_side    = 1.0 if center_local_x >= 0.0 else -1.0

        arc_remaining = abs(angle_error) * (math.pi / 180.0) * arc_radius

        outX_local = x_pid.update(radius_error * center_side, dt)
        outY_local = y_pid.update(arc_remaining,               dt)
        outT       = t_pid.update(angle_error,                 dt)

        mag = math.hypot(outX_local, outY_local)
        if not pos_settled and mag > 1e-3 and mag < params.get("minSpeed", 0.0):
            scale       = params.get("minSpeed", 0.0) / mag
            outX_local *= scale
            outY_local *= scale
        if mag > max_curve_trans:
            scale       = max_curve_trans / mag
            outX_local *= scale
            outY_local *= scale
        outT = max(-max_curve_rot, min(max_curve_rot, outT))

        target_vx    = (outX_local * cosH + outY_local * sinH) * voltage_to_ips
        target_vy    = (-outX_local * sinH + outY_local * cosH) * voltage_to_ips
        target_omega = outT * voltage_to_dps

        state["vx"] += (target_vx    - state["vx"]) * drag_coeff
        state["vy"] += (target_vy    - state["vy"]) * drag_coeff
        state["w"]  += (target_omega - state["w"])  * drag_coeff

        state["x"]     += state["vx"] * dt
        state["y"]     += state["vy"] * dt
        state["theta"] += state["w"]  * dt
        state["theta"]  = (state["theta"] + 180) % 360 - 180

        history.append({
            "x": state["x"], "y": state["y"], "theta": state["theta"],
            "vx": state["vx"], "vy": state["vy"], "w": state["w"]
        })

        elapsed += dt

    return history, state


def simulate(source, body):
    stripped_source = strip_autonomous(source)

    constants    = parse_constants(stripped_source)
    poses_table  = parse_poses(stripped_source, constants)
    params_table = parse_move_params(stripped_source, constants)
    paths        = parse_path_strings(stripped_source)

    constants.update(parse_constants(body))
    poses_table.update(parse_poses(body, constants))
    params_table.update(parse_move_params(body, constants))

    x_schedules     = parse_scheduler(stripped_source, "X")
    y_schedules     = parse_scheduler(stripped_source, "Y")
    theta_schedules = parse_scheduler(stripped_source, "Theta")

    x_sched     = PythonGainScheduler(x_schedules,     default_kp=8.0, default_kd=15.0, default_slew=10.0)
    y_sched     = PythonGainScheduler(y_schedules,     default_kp=8.0, default_kd=15.0, default_slew=10.0)
    theta_sched = PythonGainScheduler(theta_schedules, default_kp=3.5, default_kd=8.0,  default_slew=0.0)

    calls = find_all_calls(body)

    state  = {"x": 0.0, "y": 0.0, "theta": 0.0, "vx": 0.0, "vy": 0.0, "w": 0.0}
    segments = []
    current_segment = [{"x": 0.0, "y": 0.0, "theta": 0.0}]
    events   = []
    used_paths = set()

    default_params = {
        "maxTranslationSpeed": 127.0,
        "maxRotationSpeed":    127.0,
        "minSpeed":            0.0,
        "exitRange":           0.5,
        "earlyExitRange":      0.0,
        "timeout":             5000,
        "async":               True
    }

    def get_arg_val_local(args, index, default):
        if index < len(args):
            val = number(args[index], constants)
            if val is not None:
                return val
        return default

    def get_call_params(args, index):
        if index < len(args):
            param_name = args[index].strip()
            if param_name.startswith("{") and param_name.endswith("}"):
                return parse_inline_params(param_name, constants)
            if param_name in params_table:
                return params_table[param_name]
            if re.match(r"^[A-Za-z_]\w*$", param_name):
                pattern = re.compile(
                    r"\bMoveParams\s+" + re.escape(param_name) + r"\s*(?:=\s*)?\{([^;]+)\}\s*;",
                    re.DOTALL
                )
                m = pattern.search(body)
                if m:
                    raw = m.group(1).strip()
                    if "." in raw:
                        return parse_inline_params("{" + raw + "}", constants)
                    parts = split_args(raw)
                    return {
                        "maxTranslationSpeed": get_list_arg(parts, 0, 127.0, constants),
                        "maxRotationSpeed":    get_list_arg(parts, 1, 127.0, constants),
                        "minSpeed":            get_list_arg(parts, 2, 0.0,   constants),
                        "exitRange":           get_list_arg(parts, 3, 0.5,   constants),
                        "earlyExitRange":      get_list_arg(parts, 4, 0.0,   constants),
                        "timeout":             get_list_arg(parts, 5, 5000,  constants),
                        "async": "false" not in parts[6].lower() if len(parts) >= 7 else True
                    }
        return default_params

    active_trajectory = None

    def flush_active_motion():
        nonlocal active_trajectory, state, current_segment
        if active_trajectory:
            current_segment.extend(active_trajectory)
            last_state      = active_trajectory[-1]
            state["x"]      = last_state["x"]
            state["y"]      = last_state["y"]
            state["theta"]  = last_state["theta"]
            state["vx"]     = last_state.get("vx", 0.0)
            state["vy"]     = last_state.get("vy", 0.0)
            state["w"]      = last_state.get("w",  0.0)
            active_trajectory = None

    def commit_segment_and_reset(new_x, new_y, new_theta):
        nonlocal current_segment, state
        flush_active_motion()
        if current_segment:
            segments.append(list(current_segment))
        current_segment = [{"x": new_x, "y": new_y, "theta": new_theta}]
        state["x"]     = new_x
        state["y"]     = new_y
        state["theta"] = new_theta
        state["vx"]    = 0.0
        state["vy"]    = 0.0
        state["w"]     = 0.0

    for name, args in calls:
        if name == "setPose":
            if len(args) == 1:
                pose_name = args[0].strip()
                if pose_name in poses_table:
                    target = dict(poses_table[pose_name])
                else:
                    target = {"x": 0.0, "y": 0.0, "theta": 0.0}
            else:
                target = {
                    "x":     get_arg_val_local(args, 0, 0.0),
                    "y":     get_arg_val_local(args, 1, 0.0),
                    "theta": get_arg_val_local(args, 2, 0.0),
                }
            commit_segment_and_reset(target["x"], target["y"], target["theta"])
            events.append({"label": f"setPose({target['x']}, {target['y']}, {target['theta']})", "pose": target})

        elif name == "moveToPoint" and len(args) >= 2:
            flush_active_motion()
            tx    = get_arg_val_local(args, 0, state["x"])
            ty    = get_arg_val_local(args, 1, state["y"])
            p_val = get_call_params(args, 2)
            angle_correction = True
            if len(args) >= 4:
                angle_correction = "false" not in args[3].lower()
            history, state = run_motion_sim(state, tx, ty, None, p_val, angle_correction, x_sched, y_sched, theta_sched)
            active_trajectory = history
            if not p_val.get("async", True):
                flush_active_motion()
            events.append({"label": f"moveToPoint({tx}, {ty})", "pose": dict(state)})

        elif name == "moveToPose" and len(args) >= 3:
            flush_active_motion()
            tx           = get_arg_val_local(args, 0, state["x"])
            ty           = get_arg_val_local(args, 1, state["y"])
            target_theta = get_arg_val_local(args, 2, state["theta"])
            p_val        = get_call_params(args, 3)
            history, state = run_motion_sim(state, tx, ty, target_theta, p_val, False, x_sched, y_sched, theta_sched)
            active_trajectory = history
            if not p_val.get("async", True):
                flush_active_motion()
            events.append({"label": f"moveToPose({tx}, {ty}, {target_theta})", "pose": dict(state)})

        elif name in ("moveDistance", "strafeDistance") and args:
            flush_active_motion()
            distance = get_arg_val_local(args, 0, 0.0)
            p_val    = get_call_params(args, 1)
            forward, strafe = heading_vectors(state["theta"])
            vx, vy = forward if name == "moveDistance" else strafe
            tx = state["x"] + distance * vx
            ty = state["y"] + distance * vy
            history, state = run_motion_sim(state, tx, ty, state["theta"], p_val, False, x_sched, y_sched, theta_sched)
            active_trajectory = history
            if not p_val.get("async", True):
                 flush_active_motion()
            events.append({"label": f"{name}({distance})", "pose": dict(state)})

        elif name == "moveRelative" and len(args) >= 2:
            flush_active_motion()
            forward_dist  = get_arg_val_local(args, 0, 0.0)
            sideways_dist = get_arg_val_local(args, 1, 0.0)
            p_val         = get_call_params(args, 2)
            forward, strafe = heading_vectors(state["theta"])
            tx = state["x"] + forward_dist * forward[0] + sideways_dist * strafe[0]
            ty = state["y"] + forward_dist * forward[1] + sideways_dist * strafe[1]
            history, state = run_motion_sim(state, tx, ty, state["theta"], p_val, False, x_sched, y_sched, theta_sched)
            active_trajectory = history
            if not p_val.get("async", True):
                flush_active_motion()
            events.append({"label": f"moveRelative({forward_dist}, {sideways_dist})", "pose": dict(state)})

        elif name == "turnToHeading" and args:
            flush_active_motion()
            target_theta = get_arg_val_local(args, 0, state["theta"])
            p_val        = get_call_params(args, 1)
            history, state = run_motion_sim(state, state["x"], state["y"], target_theta, p_val, False, x_sched, y_sched, theta_sched)
            active_trajectory = history
            if not p_val.get("async", True):
                flush_active_motion()
            events.append({"label": f"turnToHeading({target_theta})", "pose": dict(state)})

        elif name == "turnToPoint" and len(args) >= 2:
            flush_active_motion()
            tx           = get_arg_val_local(args, 0, state["x"])
            ty           = get_arg_val_local(args, 1, state["y"])
            p_val        = get_call_params(args, 2)
            target_theta = math.degrees(math.atan2(tx - state["x"], ty - state["y"]))
            history, state = run_motion_sim(state, state["x"], state["y"], target_theta, p_val, False, x_sched, y_sched, theta_sched)
            active_trajectory = history
            if not p_val.get("async", True):
                flush_active_motion()
            events.append({"label": f"turnToPoint({tx}, {ty})", "pose": dict(state)})

        elif name == "curveCircle" and len(args) >= 2:
            flush_active_motion()
            target_theta = get_arg_val_local(args, 0, state["theta"])
            radius       = abs(get_arg_val_local(args, 1, 0.0))
            p_val        = get_call_params(args, 2)
            direction    = parse_direction(args)
            history, state = run_curve_circle_sim(state, target_theta, radius, direction, p_val, x_sched, y_sched, theta_sched)
            active_trajectory = history
            if not p_val.get("async", True):
                flush_active_motion()
            events.append({"label": f"curveCircle({target_theta}, {radius}, {direction})", "pose": dict(state)})

        elif name in ("followPath", "followPathPID"):
            flush_active_motion()
            joined        = " ".join(args)
            path_var_name = resolve_path_name(stripped_source, args[0])

            if path_var_name and path_var_name in paths:
                used_paths.add(path_var_name)
                path = paths[path_var_name]

                if path:
                    lookahead_dist = get_arg_val_local(args, 1, 10.0)
                    p_val          = get_call_params(args, 2)

                    heading_mode = "FollowPath"
                    if "HeadingMode::HoldAngle" in joined:
                        heading_mode = "HoldAngle"
                    elif "HeadingMode::CustomAngles" in joined:
                        heading_mode = "CustomAngles"

                    hold_angle_deg = get_arg_val_local(args, 4, 0.0)
                    reversed_bool  = False
                    if len(args) >= 6:
                        reversed_bool = "true" in args[5].lower()

                    scale_factor = resolve_path_scale(stripped_source, args[0], constants)
                    scaled_path  = [{"x": pt["x"] * scale_factor, "y": pt["y"] * scale_factor, "theta": pt["theta"]} for pt in path]

                    history, state = run_path_sim(state, scaled_path, lookahead_dist, p_val, heading_mode, hold_angle_deg, reversed_bool, x_sched, y_sched, theta_sched)
                    active_trajectory = history
                    if not p_val.get("async", True):
                        flush_active_motion()
                    events.append({"label": f"{name}({path_var_name}, {heading_mode})", "pose": dict(state)})

        elif name == "waitUntil" and len(args) >= 1:
            dist = get_arg_val_local(args, 0, 0.0)
            if active_trajectory:
                cum_dists = get_cumulative_distances(active_trajectory)
                split_idx = len(active_trajectory) - 1
                for idx, d in enumerate(cum_dists):
                    if d >= dist:
                        split_idx = idx
                        break
                current_segment.extend(active_trajectory[:split_idx + 1])
                events.append({"label": f"waitUntil({dist})", "pose": active_trajectory[split_idx]})
                split_state     = active_trajectory[split_idx]
                state["x"]     = split_state["x"]
                state["y"]     = split_state["y"]
                state["theta"] = split_state["theta"]
                state["vx"]    = split_state.get("vx", 0.0)
                state["vy"]    = split_state.get("vy", 0.0)
                state["w"]     = split_state.get("w",  0.0)
                active_trajectory = active_trajectory[split_idx:]

        elif name == "waitUntilDone":
            flush_active_motion()

        elif name in ("cancelMotion", "cancelAllMotions"):
            if active_trajectory:
                active_trajectory = None

        elif name == "delay" and args:
            ms            = get_arg_val_local(args, 0, 0.0)
            steps_to_advance = int(ms / 10)
            if ms > 0:
                if active_trajectory:
                    if len(active_trajectory) > steps_to_advance:
                        current_segment.extend(active_trajectory[:steps_to_advance + 1])
                        events.append({"label": f"delay({int(ms)}ms)", "pose": active_trajectory[steps_to_advance]})
                        split_state     = active_trajectory[steps_to_advance]
                        state["x"]     = split_state["x"]
                        state["y"]     = split_state["y"]
                        state["theta"] = split_state["theta"]
                        state["vx"]    = split_state.get("vx", 0.0)
                        state["vy"]    = split_state.get("vy", 0.0)
                        state["w"]     = split_state.get("w",  0.0)
                        active_trajectory = active_trajectory[steps_to_advance:]
                    else:
                        current_segment.extend(active_trajectory)
                        last_state      = active_trajectory[-1]
                        state["x"]     = last_state["x"]
                        state["y"]     = last_state["y"]
                        state["theta"] = last_state["theta"]
                        state["vx"]    = last_state.get("vx", 0.0)
                        state["vy"]    = last_state.get("vy", 0.0)
                        state["w"]     = last_state.get("w",  0.0)
                        remaining_steps = steps_to_advance - len(active_trajectory)
                        for _ in range(remaining_steps):
                            current_segment.append({"x": state["x"], "y": state["y"], "theta": state["theta"], "vx": 0.0, "vy": 0.0, "w": 0.0})
                        active_trajectory = None
                else:
                    for _ in range(steps_to_advance):
                        current_segment.append({"x": state["x"], "y": state["y"], "theta": state["theta"], "vx": 0.0, "vy": 0.0, "w": 0.0})
                    events.append({"label": f"delay({int(ms)}ms) [idle]", "pose": {"x": state["x"], "y": state["y"], "theta": state["theta"]}})

    flush_active_motion()
    if current_segment:
        segments.append(list(current_segment))

    if not segments and paths:
        seg_name, path = next(iter(paths.items()))
        segments.append(path)
        events.append({"label": f"unused path preview: {seg_name}", "pose": path[-1]})

    return segments, events


def build_html(segments, events, field_size, map_path):
    data = {
        "segments":  segments,
        "events":    events,
        "fieldSize": field_size,
        "mapSrc":    os.path.relpath(Path(map_path).resolve(), OUTPUT_HTML.parent.resolve()),
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
      <button id="speed">Speed: 1x</button>
      <input id="scrub" type="range" min="0" max="1000" value="0">
      <span id="readout"></span>
    </div>
  </div>
  <script>
    const sim = {payload};
    const allPoses = sim.segments.flat();
    const segmentBreaks = [];
    let acc = 0;
    for (let i = 0; i < sim.segments.length - 1; i++) {{
      acc += sim.segments[i].length;
      segmentBreaks.push(acc);
    }}

    const canvas  = document.getElementById('field');
    const ctx     = canvas.getContext('2d');
    const img     = new Image();
    img.src       = sim.mapSrc;
    const play    = document.getElementById('play');
    const speedBtn = document.getElementById('speed');
    const scrub   = document.getElementById('scrub');
    const readout = document.getElementById('readout');

    let running    = true;
    let frame      = 0;
    let exactFrame = 0;

    const speeds     = [0.75, 1.5, 3.0, 7.5, 15.0];
    const speedLabels = ['1x', '2x', '4x', '10x', '20x'];
    let speedIdx     = 0;
    let speed        = speeds[speedIdx];
    speedBtn.onclick = () => {{
      speedIdx = (speedIdx + 1) % speeds.length;
      speed    = speeds[speedIdx];
      speedBtn.textContent = 'Speed: ' + speedLabels[speedIdx];
    }};
    function resize() {{
      canvas.width  = canvas.clientWidth  * devicePixelRatio;
      canvas.height = canvas.clientHeight * devicePixelRatio;
    }}

    function toPx(p) {{
      const scale = Math.min(canvas.width, canvas.height) / sim.fieldSize;
      return {{ x: canvas.width / 2 + p.x * scale, y: canvas.height / 2 - p.y * scale, scale }};
    }}

    function robot(p) {{
      const q = toPx(p);
      const w = 7 * q.scale;
      const l = 7 * q.scale;
      const a = p.theta * Math.PI / 180;
      ctx.save();
      ctx.translate(q.x, q.y);
      ctx.rotate(a);
      ctx.fillStyle   = 'rgba(255, 235, 80, 0.9)';
      ctx.strokeStyle = '#111';
      ctx.lineWidth   = 2 * devicePixelRatio;
      ctx.fillRect(-w / 2, -l / 2, w, l);
      ctx.strokeRect(-w / 2, -l / 2, w, l);
      ctx.fillStyle = '#e43';
      ctx.beginPath();
      ctx.moveTo(0,          -l / 2 - l * 0.6);
      ctx.lineTo( w * 0.4,  -l / 2 + l * 0.2);
      ctx.lineTo(-w * 0.4,  -l / 2 + l * 0.2);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }}

    function draw() {{
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      const mapScale = Math.min(canvas.width / img.width, canvas.height / img.height);
      const mw = img.width  * mapScale;
      const mh = img.height * mapScale;
      ctx.drawImage(img, (canvas.width - mw) / 2, (canvas.height - mh) / 2, mw, mh);
      const colors = ['rgba(0,255,190,0.9)', 'rgba(255,160,0,0.9)', 'rgba(100,180,255,0.9)', 'rgba(255,80,180,0.9)', 'rgba(180,255,80,0.9)'];
      ctx.lineWidth = 3 * devicePixelRatio;

      let segIdx = 0;
      let poseIdx = 0;
      for (let s = 0; s < sim.segments.length; s++) {{
        const seg = sim.segments[s];
        ctx.strokeStyle = colors[s % colors.length];
        ctx.beginPath();
        let started = false;
        for (let i = 0; i < seg.length; i++) {{
          const globalIdx = poseIdx + i;
          if (globalIdx > frame) break;
          const q = toPx(seg[i]);
          if (!started) {{ ctx.moveTo(q.x, q.y); started = true;
          }}
          else           {{ ctx.lineTo(q.x, q.y);
          }}
        }}
        ctx.stroke();
        poseIdx += seg.length;
      }}

      const p = allPoses[Math.min(frame, allPoses.length - 1)];
      if (p) {{
        robot(p);
        readout.innerHTML = '<code>x=' + p.x.toFixed(1) + ' y=' + p.y.toFixed(1) + ' theta=' + p.theta.toFixed(1) + ' frame=' + frame + '/' + (allPoses.length - 1) + '</code>';
        scrub.value = allPoses.length <= 1 ? 0 : Math.round(frame * 1000 / (allPoses.length - 1));
      }}
    }}

    function tick() {{
      if (running) {{
        exactFrame = (exactFrame + speed) % Math.max(allPoses.length, 1);
        frame      = Math.floor(exactFrame);
      }}
      draw();
      requestAnimationFrame(tick);
    }}

    play.onclick = () => {{
      running          = !running;
      play.textContent = running ? 'Pause' : 'Play';
    }};

    scrub.oninput = () => {{
      running          = false;
      play.textContent = 'Play';
      frame            = Math.round(Number(scrub.value) * (allPoses.length - 1) / 1000);
      exactFrame       = frame;
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
    parser.add_argument("--field-size", type=float, default=144.0)
    parser.add_argument("--main", default=MAIN_CPP)
    parser.add_argument("--map",  default=MAP_PNG)
    parser.add_argument("--out",  default=OUTPUT_HTML)
    args = parser.parse_args()

    source       = Path(args.main).read_text()
    clean_source = strip_comments(source)
    clean_source_no_auton = strip_autonomous(clean_source)
    body         = extract_simulation(clean_source_no_auton)
    segments, events = simulate(clean_source_no_auton, body)

    OUTPUT_HTML = Path(args.out)
    OUTPUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_HTML.write_text(build_html(segments, events, args.field_size, args.map))
    print(f"Wrote {OUTPUT_HTML}")
    total_poses = sum(len(s) for s in segments)
    print(f"Loaded {total_poses} simulated poses across {len(segments)} segment(s)")


if __name__ == "__main__":
    main()