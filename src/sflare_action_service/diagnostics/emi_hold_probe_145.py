"""One-shot, stationary EMI transport probe; never plays an action trajectory."""

import math
import sys
import threading
import time

sys.path.insert(0, "/home/aidlux/RobotGK")
from PallasSDK import Controller


def ok(value, label):
    if value != 0:
        raise RuntimeError(f"{label}: {value}")


def joints(robot):
    raw = robot.get_joint_pos()
    values = raw.toList() if hasattr(raw, "toList") else list(raw)
    result = [float(v) for v in values[:7]]
    if len(result) != 7 or not all(math.isfinite(v) for v in result):
        raise RuntimeError(f"Invalid encoder joints: {result}")
    return result


def power(robot):
    raw = robot.get_state()
    values = raw.toList() if hasattr(raw, "toList") else list(raw)
    return int(values[0])


def main():
    controller = Controller()
    robot = None
    connected = False
    emi_started = False
    power_attempted = False
    feedback = []
    excessive_motion = threading.Event()
    try:
        ok(controller.connect("192.168.3.100"), "connect")
        connected = True
        print("firmware", controller.get_controller_version().split(";")[0], flush=True)
        ok(controller.set_data_unit(1), "set_data_unit")
        robot = controller.add_robot(1)
        ok(robot.attach(), "attach")
        if power(robot) != 0:
            raise RuntimeError("Arm is not initially powered off")
        before = joints(robot)
        print("initial_joints", before, flush=True)
        def on_feedback(data):
            actual_j1 = float(data.robot_joint_pos[0])
            feedback.append((time.monotonic(), int(data.error_code), actual_j1))
            if abs(actual_j1 - before[0]) > 2.0:
                excessive_motion.set()

        robot.set_emi_feedback_callback(on_feedback)
        power_attempted = True
        ok(controller.set_power_enable(True, 1), "power on")
        deadline = time.monotonic() + 5.0
        while power(robot) != 1:
            if time.monotonic() >= deadline:
                raise RuntimeError("Power-on confirmation timeout")
            time.sleep(0.05)
        ok(robot.abort(), "abort")
        hold = joints(robot)
        if max(abs(a - b) for a, b in zip(before, hold)) > 0.5:
            raise RuntimeError("Position shifted on power-on; no EMI commands sent")
        print("hold_joints", hold, flush=True)
        micro = len(sys.argv) > 1 and sys.argv[1] == "--micro"
        if micro:
            upper, lower = [float(v) for v in robot.get_limit(1)]
            if not lower + 2.0 < hold[0] < upper - 2.0:
                raise RuntimeError("J1 is too near a soft limit")
        emi_started = True
        ok(robot.emi_start(0x8800, 1, 0.04, 0, 0), "emi_start")
        start = time.monotonic()
        frames = 100 if micro else 25
        for index in range(frames):
            if excessive_motion.is_set():
                raise RuntimeError("Unexpected J1 motion above 2 degrees")
            command = hold + [0.0] * 8
            if micro:
                phase = ((index + 1) / 50.0) if index < 50 else ((100 - index - 1) / 50.0)
                phase = max(0.0, min(1.0, phase))
                command[0] += phase * phase * (3.0 - 2.0 * phase)
            ok(robot.servo_j(command, 0.0, 0.04, t=0.04,
                             lookahead_time=0.03, gain=900.0),
               f"servo_j[{index}]")
            deadline = start + (index + 1) * 0.04
            while time.monotonic() < deadline:
                time.sleep(min(0.002, deadline - time.monotonic()))
        after = joints(robot)
        print("after_joints", after, flush=True)
        print("max_displacement_deg", max(abs(a - b) for a, b in zip(hold, after)), flush=True)
        print("feedback_count", len(feedback), "feedback_errors", sorted(set(code for _, code, _ in feedback)), flush=True)
        if feedback:
            print("feedback_j1_range", min(j1 for _, _, j1 in feedback),
                  max(j1 for _, _, j1 in feedback), flush=True)
        print("controller_errors", controller.check_error(), flush=True)
    finally:
        if connected and robot is not None:
            if emi_started:
                try:
                    ok(robot.emi_end(), "emi_end")
                except Exception as exc:
                    print("cleanup_emi_error", repr(exc), flush=True)
            if power_attempted:
                for attempt in range(3):
                    try:
                        ok(controller.set_power_enable(False, 1), "power off")
                        deadline = time.monotonic() + 3.0
                        while power(robot) != 0 and time.monotonic() < deadline:
                            time.sleep(0.05)
                        if power(robot) == 0:
                            print("power_off_confirmed", flush=True)
                            break
                    except Exception as exc:
                        print("cleanup_power_error", attempt, repr(exc), flush=True)
                else:
                    print("POWER_OFF_NOT_CONFIRMED; connection retained", flush=True)
                    return
            try:
                robot.detach()
            finally:
                controller.disconnect()


if __name__ == "__main__":
    main()
