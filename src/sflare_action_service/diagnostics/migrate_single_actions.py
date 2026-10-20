"""One-shot source-to-JSON migration. Prints an apply_patch; never writes files.

Run with the original source path. Only constant expressions are evaluated;
the production ROS node and SDK are never imported or started.
"""
import ast
import json
import math
import sys
from pathlib import Path


source = Path(sys.argv[1]).read_text(encoding="utf-8-sig")
tree = ast.parse(source)
env = {"math": math}
needed = {"ZERO", "LIKE_POSE", "RAISE_HEAD_POSE", "SLIGHT_RAISE_POSE", "DIAGONAL_POINT_POSE",
          "CHIN_REST_POSE", "HANDOVER_POSE", "DOUBT_POSE", "DOUBT_ALT_POSE", "WAVE", "HANDSHAKE",
          "P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8", "P_FINAL", "STAGE_1", "STAGE_2", "STAGE_4"}
constants = []
for item in tree.body:
    if isinstance(item, ast.Assign) and isinstance(item.targets[0], ast.Name):
        name = item.targets[0].id
        if name in needed or name.startswith("CHASSIS_"):
            env[name] = eval(compile(ast.Expression(item.value), "<constant>", "eval"), {"__builtins__": {}}, env)
            if name.startswith("CHASSIS_"):
                constants.append(ast.get_source_segment(source, item))


def move(i, seconds=1.8, settle=True):
    return {"op": "move", "frame": i, "duration": seconds, "settle": settle}


def wait(seconds):
    return {"op": "wait", "seconds": seconds}


def hand(action):
    return {"op": "hand", "action": action}


def yaw(degrees, **kwargs):
    return dict(op="yaw_target", degrees=degrees, **kwargs)


def loop(cycles, count):
    return {"op": "loop_until", "max_iterations": count, "cycles": cycles}


documents = {}


def action(name, poses, steps):
    coordinates = [list(p[:7]) for p in poses]
    assert coordinates[-1] == [0]*7
    documents[name] = {
        "schema_version": "1.0.0", "name": name, "scope": "right", "units": "deg",
        "control": {"period": 0.020, "speed_scale": 1.0}, "exit": {"auto_return_zero": False},
        "keyframes": [{"time": float(i), "right": p, "type": "stop"} for i, p in enumerate(coordinates)],
        "events": [], "program": steps,
    }


z = env["ZERO"]
for name, pose, gesture, seconds, preparation in (
    ("like", "LIKE_POSE", "like", 2.5, False),
    ("raise_head", "RAISE_HEAD_POSE", "open", 3.0, False),
    ("slight_raise", "SLIGHT_RAISE_POSE", None, 2.0, False),
    ("diagonal_point", "DIAGONAL_POINT_POSE", "point", 3.0, True),
    ("chin_rest", "CHIN_REST_POSE", "chin_rest", 3.0, True),
    ("handover", "HANDOVER_POSE", "open", 3.0, False),
):
    steps = [hand("open"), wait(0.5)] if preparation else []
    steps += [move(0)]
    if gesture:
        steps.append(hand(gesture))
    steps.append(wait(seconds))
    if name != "handover":
        steps.append(hand("open"))
    steps.append(move(1, 4.8))
    action(name, [env[pose], z], steps)

for name in ("handshake", "shakehand"):
    poses = [*env["HANDSHAKE"], z]
    action(name, poses, [move(i, 0.4) for i in range(7)] + [wait(3), hand("open"), move(7, 4.8)])

poses = [env["DOUBT_POSE"], *([env["DOUBT_ALT_POSE"], env["DOUBT_POSE"]]*3), z]
action("doubt", poses, [move(0), hand("point")] + [move(i, 1.2) for i in range(1, 7)] + [hand("open"), move(7, 4.8)])

mode = {"op": "mode", "value": "remote_control"}
yaw_start = {"op": "yaw_start"}
yaw_stop = {"op": "yaw_stop"}
linear_start = {"op": "linear_start", "speed": 0.2, "distance": 0.3}
linear_stop = {"op": "linear_stop"}
steps = [mode, wait(0.3), yaw_start]
for i in range(6):
    steps += [yaw(30 if i % 2 == 0 else -30), move(i)]
action("sway", [*env["STAGE_1"], z], steps + [yaw(0), hand("open"), move(6, 4.8), yaw_stop])

poses = []
for p in env["STAGE_2"]:
    p = list(p)
    p[0] += 10
    poses.append(p)
steps = [mode, wait(0.3), {"op": "yaw_start", "kp": 2.6}, hand("half_heart")]
for i in range(6):
    steps += [yaw(-10 if i % 2 == 0 else 10), move(i), wait(0.4)]
action("heart", [*poses, z], steps + [yaw(0), wait(3), hand("open"), move(6, 4.8), yaw_stop])

action("forward", [env["P5"], env["P6"], z],
       [mode, wait(0.3), hand("open"), linear_start] +
       ([move(0), wait(0.3), move(1), wait(0.3)] * 3) + [hand("open"), move(2, 4.8), linear_stop])
steps = [mode, wait(0.3), hand("open"), linear_start]
for i in range(6):
    steps += [move(i, 4.8 if i == 5 else 1.8), wait(0.2)]
action("forward_wave", env["WAVE"], steps + [linear_stop])

poses = [env["P7"], *env["WAVE"][:-1], z]
cycles = [[move(i), wait(0.3)] for i in [1, 2, 3, 4, 5, 0]]
action("turn_grip", poses, [mode, wait(0.3), {"op": "yaw_start", "timeout": 30}, hand("safe_fist"),
       wait(0.5), move(0), yaw(360), loop(cycles, 100), hand("open"), move(6, 4.8), yaw_stop])

poses = [*env["STAGE_1"], *env["STAGE_2"], env["P5"], env["P6"], *env["STAGE_4"]]
steps = [mode, wait(0.5), yaw_start]
for i in range(6):
    steps += [yaw(20 if i % 2 == 0 else -20), move(i, 1.8, False), wait(0.3)]
steps += [hand("half_heart")]
for i in range(6, 12):
    steps += [yaw(-20 if i % 2 == 0 else 20), move(i, 1.8, False), wait(0.3)]
steps += [hand("open"), yaw(360, reference="current", timeout=18, max_speed=0.6, travel_degrees=540),
          loop([[move(12, 1.8, False), wait(0.3)], [move(13, 1.8, False), wait(0.3)]], 34),
          {"op": "yaw_final", "degrees": 360, "guard_degrees": 90, "timeout": 8, "max_speed": 0.4}]
for i in range(14, 18):
    steps.append(move(i, 4.8 if i == 17 else 1.8, i == 17))
    if i == 16:
        steps += [{"op": "settle", "seconds": 10}, hand("half_heart"), loop([[wait(0.1)]], 50), wait(2), hand("open")]
    else:
        steps.append(wait(0.08))
action("dance", poses, steps + [yaw_stop])

node_class = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == "DualArmActionNode")
methods = [m for m in node_class.body if isinstance(m, ast.FunctionDef) and
           (m.name in {"_on_odom", "has_pose", "current_yaw", "set_target_yaw", "reached", "chassis_failure_reason",
                       "raise_if_yaw_failed", "set_mode", "_publish_chassis_speed", "start_yaw_control", "_yaw_control_task",
                       "stop_yaw_control", "start_linear_motion", "stop_linear_motion", "stop_all_linear_motion"})]
init_method = next(m for m in node_class.body if isinstance(m, ast.FunctionDef) and m.name == "__init__")
init_source = ast.get_source_segment(source, init_method)
start = init_source.index('        self.cmd_vel_pub =')
end = init_source.index('        self.actions:')
chassis = '''"""Original chassis-only ROS wiring and odometry control, extracted without SDK code."""
import math
import threading
import time
from typing import Dict, Optional, Tuple
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from aid_robot_msgs.srv import StatusChange

class ChassisMotionFailure(RuntimeError):
    pass

''' + "\n".join(constants) + '''

class ChassisController:
    def __init__(self, node):
        self.node = node
        self._chassis_lock = threading.Lock()
''' + init_source[start:end] + '''
    def create_publisher(self, *args):
        return self.node.create_publisher(*args)

    def create_client(self, *args):
        return self.node.create_client(*args)

    def create_subscription(self, *args):
        return self.node.create_subscription(*args)

    def get_logger(self):
        return self.node.get_logger()

''' + "\n\n".join("    " + ast.get_source_segment(source, m) for m in methods) + "\n"

root = Path(__file__).resolve().parents[1]
print("*** Begin Patch")
for name, document in documents.items():
    path = root / "actions" / (name + ".json")
    if path.exists():
        print("*** Update File: " + str(path))
        print("@@")
        for line in path.read_text(encoding="utf-8-sig").splitlines():
            print("-" + line)
    else:
        print("*** Add File: " + str(path))
    for line in json.dumps(document, ensure_ascii=False, indent=2).splitlines():
        print("+" + line)
print("*** Add File: " + str(root / "chassis_control.py"))
for line in chassis.splitlines():
    print("+" + line)
print("*** End Patch")
