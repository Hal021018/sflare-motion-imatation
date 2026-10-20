"""JSON sequencing for migrated single-arm actions; no SDK/ROS imports.

All possible motion edges are prepared before power-on. Runtime selects cached
frames and retains the original odometry-dependent chassis loops and guards.
"""
import math
import time
from dataclasses import replace

from action_pipeline import ActionEvent, ActionPlan, transition_path, MAX_JOINT_SPEED_DEG_S


HAND_ACTIONS = {"open", "safe_fist", "half_heart", "like", "point", "chin_rest"}
OPS = {
    "move", "wait", "hand", "mode", "yaw_start", "yaw_target", "yaw_final",
    "yaw_assert", "yaw_stop", "linear_start", "linear_stop", "loop_until", "settle",
}
CHASSIS_OPS = OPS - {"move", "wait", "hand", "settle", "loop_until"}


class ProgramSequenceFailure(RuntimeError):
    """Choreography failure, not an SDK, encoder, emergency-stop, or servo fault."""


def _positive(value, label, allow_zero=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(label + " must be numeric")
    if not math.isfinite(value) or value < 0 or (not allow_zero and value == 0):
        raise ValueError(label + " must be finite and positive")


def validate_program(steps, frame_count, nested=False):
    if not isinstance(steps, (list, tuple)) or not steps:
        raise ValueError("program must be a nonempty sequence")
    for step in steps:
        if not isinstance(step, dict) or step.get("op") not in OPS:
            raise ValueError("unsupported program instruction")
        op = step["op"]
        if "phase" in step and (op != "wait" or step["phase"] != "still"):
            raise ValueError("only presentation wait may declare phase=still")
        if op == "move":
            index = step.get("frame")
            if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < frame_count:
                raise ValueError("move frame is outside keyframes")
            _positive(step.get("duration"), "move duration")
            if not isinstance(step.get("settle", True), bool):
                raise ValueError("move settle must be boolean")
            _positive(step.get("settle_timeout", 10.0), "settle timeout")
        elif op in {"wait", "settle"}:
            _positive(step.get("seconds"), op + " seconds", op == "wait")
        elif op == "hand":
            if step.get("action") not in HAND_ACTIONS:
                raise ValueError("unsupported hand gesture")
        elif op == "mode":
            if step.get("value") != "remote_control":
                raise ValueError("only the original remote_control mode is allowed")
        elif op == "yaw_start":
            _positive(step.get("timeout", 45.0), "yaw timeout")
            _positive(step.get("kp", 1.3), "yaw kp")
        elif op in {"yaw_target", "yaw_final"}:
            if op == "yaw_final" and "guard_degrees" not in step:
                raise ValueError("yaw_final requires an explicit correction guard")
            if step.get("reference", "initial") not in {"initial", "current"}:
                raise ValueError("invalid yaw reference")
            offset = step.get("degrees", 0.0)
            if isinstance(offset, bool) or not isinstance(offset, (int, float)) or not math.isfinite(offset):
                raise ValueError("yaw offset must be finite")
            for key in ("timeout", "max_speed", "travel_degrees", "guard_degrees"):
                if key in step:
                    _positive(step[key], "yaw " + key)
        elif op == "linear_start":
            _positive(step.get("speed"), "linear speed")
            _positive(step.get("distance"), "linear distance")
        elif op == "loop_until":
            if nested:
                raise ValueError("nested chassis loops are not supported")
            count = step.get("max_iterations")
            if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 100:
                raise ValueError("loop bound must be 1..100")
            cycles = step.get("cycles")
            if not isinstance(cycles, list) or not cycles:
                raise ValueError("loop cycles must be nonempty")
            for cycle in cycles:
                validate_program(cycle, frame_count, True)
                if any(s["op"] not in {"move", "wait", "hand", "settle", "yaw_assert"} for s in cycle):
                    raise ValueError("loop may not start/stop chassis workers or switch modes")
    # Explicit, statically known zero endpoint is required for cleanup/settling.
    if not nested and steps[-1].get("op") not in {"move", "wait", "yaw_stop", "linear_stop"}:
        raise ValueError("program must end with a move/hold/stop")
    if not nested:
        last_move = next((s for s in reversed(steps) if s["op"] == "move"), None)
        if last_move is None or last_move["frame"] != frame_count - 1:
            raise ValueError("program must explicitly finish at its final zero keyframe")
        flat = tuple(walk_steps(steps))
        has_yaw = any(s["op"].startswith("yaw_") for s in flat)
        has_linear = any(s["op"].startswith("linear_") for s in flat)
        if has_yaw and has_linear:
            raise ValueError("simultaneous yaw and linear workers are not supported")
        if any(s["op"] == "loop_until" for s in flat) and not has_yaw:
            raise ValueError("odometry loop requires yaw control")
        yaw_active = linear_active = mode_set = False
        for step in steps:
            op = step["op"]
            if op == "mode":
                mode_set = True
            elif op == "yaw_start":
                if not mode_set or yaw_active:
                    raise ValueError("yaw worker requires mode and exclusive ownership")
                yaw_active = True
            elif op in {"yaw_target", "yaw_final", "yaw_assert", "loop_until"}:
                if not yaw_active:
                    raise ValueError("yaw instruction requires an active worker")
            elif op == "yaw_stop":
                if not yaw_active:
                    raise ValueError("yaw_stop without yaw_start")
                yaw_active = False
            elif op == "linear_start":
                if not mode_set or linear_active:
                    raise ValueError("linear worker requires mode and exclusive ownership")
                linear_active = True
            elif op == "linear_stop":
                if not linear_active:
                    raise ValueError("linear_stop without linear_start")
                linear_active = False
        if yaw_active or linear_active:
            raise ValueError("program must explicitly stop its chassis worker")
        last_index = max(i for i, s in enumerate(steps) if s["op"] == "move")
        if any(s["op"] not in {"wait", "yaw_stop", "linear_stop"} for s in steps[last_index+1:]):
            raise ValueError("program may only hold or stop after final zero")


def walk_steps(steps):
    for step in steps:
        yield step
        if step["op"] == "loop_until":
            for cycle in step["cycles"]:
                yield from walk_steps(cycle)


class ProgramRunner:
    def __init__(self, document, initial, chassis, cancel):
        self.notify = lambda phase: None
        self.doc = document
        self.period = document.period
        self.cancel = cancel
        self.chassis = chassis
        self.initial = list(initial)
        self.current = list(initial)
        self.frame = -1
        self.linear_worker = None
        self.base_started = False
        self.yaw_started = False
        self.initial_yaw = None
        self.steps = tuple(walk_steps(document.program))
        self.has_chassis = any(s["op"] in CHASSIS_OPS for s in self.steps)
        self.needs_yaw = any(s["op"].startswith("yaw_") for s in self.steps)
        if self.has_chassis and chassis is None:
            raise RuntimeError("this action requires the original chassis interface")
        if self.needs_yaw:
            if not chassis.has_pose():
                raise RuntimeError("action requires fresh /odom before it can run")
            self.initial_yaw = chassis.current_yaw()
        self.paths = {}
        self.poses = [list(frame.right) for frame in document.keyframes]
        if any(self.poses[-1]):
            raise ValueError("program final keyframe must be the original zero pose")
        moves = {(s["frame"], s["duration"]) for s in self.steps if s["op"] == "move"}
        for previous in range(-1, len(self.poses)):
            start = self.initial if previous == -1 else self.poses[previous]
            for target, duration in moves:
                # Identical stop-to-stop quintic kernel as the wave pipeline.
                path, _ = transition_path(start, start, self.poses[target], self.poses[target],
                                          self.period, MAX_JOINT_SPEED_DEG_S,
                                          5.0 if previous == -1 else duration / document.speed_scale)
                self.paths[(previous, target, duration)] = path

    def validation_plan(self, plan=None):
        frames = [point for path in self.paths.values() for point in path]
        if plan is None:
            return ActionPlan(
                name=self.doc.name, scope="right", period=self.period,
                speed_scale=self.doc.speed_scale, trans_l=[], trans_r=[self.initial],
                action_l=[], action_r=frames, return_l=[], return_r=[self.poses[-1]], events=(),
            )
        return replace(plan, trans_r=[self.initial], action_r=frames,
                       return_r=[self.poses[-1]], events=())

    def _check(self):
        if self.cancel.is_set():
            raise RuntimeError("收到停止/急停请求，中断发包")
        if self.yaw_started:
            self.chassis.raise_if_yaw_failed(self.doc.name)

    def _hold(self, seconds, tick):
        for _ in range(math.ceil(seconds / self.period)):
            self._check()
            tick([], self.current, "主体")

    def _settle(self, seconds, tick, tracker):
        deadline = time.monotonic() + seconds
        stable = None
        while True:
            self._check()
            tick([], self.current, "主体")
            now = time.monotonic()
            if tracker.at_target({"右臂": self.current}):
                stable = now if stable is None else stable
                if now - stable >= 0.2:
                    return
            else:
                stable = None
            if now >= deadline:
                raise RuntimeError("动作阶段到位超时: " + tracker.describe_target_error({"右臂": self.current}))

    def prepare_chassis(self):
        # Mode service readiness may block: do this before starting EMI.
        if self.has_chassis:
            self._check()
            if self.needs_yaw and not self.chassis.has_pose():
                raise RuntimeError("action requires fresh /odom before it can run")
            self.base_started = True
            self.chassis.set_mode("remote_control")

    def _execute(self, steps, tick, hand, tracker):
        for step in steps:
            self._check()
            op = step["op"]
            if op == "move":
                target = step["frame"]
                self.notify('reset' if target == len(self.poses)-1 else 'moving')
                first = self.frame == -1
                path = self.paths[(self.frame, target, step["duration"])]
                for point in path[1:]:
                    self._check()
                    tick([], point, "起势" if first else "收尾" if target == len(self.poses)-1 else "主体")
                self.frame = target
                self.current = self.poses[target]
                if step.get("settle", True):
                    self._settle(step.get("settle_timeout", 10.0), tick, tracker)
            elif op == "wait":
                if step.get('phase') == 'still':
                    self.notify('still')
                self._hold(step["seconds"], tick)
            elif op == "settle":
                self._settle(step["seconds"], tick, tracker)
            elif op == "hand":
                hand(ActionEvent(0.0, "right_hand", step["action"]))
            elif op == "mode":
                pass  # Prepared before EMI starts.
            elif op == "yaw_start":
                self.chassis.start_yaw_control(self.cancel, timeout_sec=step.get("timeout", 45.0),
                                               kp=step.get("kp", 1.3))
                self.yaw_started = True
            elif op in {"yaw_target", "yaw_final"}:
                reference = self.initial_yaw if step.get("reference", "initial") == "initial" else self.chassis.current_yaw()
                target = reference + math.radians(step.get("degrees", 0.0))
                kwargs = {}
                for key, argument in (("timeout", "timeout_sec"), ("max_speed", "max_angular_speed")):
                    if key in step:
                        kwargs[argument] = step[key]
                if "travel_degrees" in step:
                    kwargs["max_commanded_travel_rad"] = math.radians(step["travel_degrees"])
                if op == "yaw_final":
                    correction = target - self.chassis.current_yaw()
                    if abs(correction) > math.radians(step["guard_degrees"]):
                        raise ProgramSequenceFailure("final heading correction exceeds the original guard")
                    kwargs["max_commanded_travel_rad"] = abs(correction)*1.5 + math.radians(15)
                self.chassis.set_target_yaw(target, **kwargs)
            elif op == "yaw_assert":
                self.chassis.raise_if_yaw_failed(self.doc.name)
            elif op == "yaw_stop":
                if not self.chassis.stop_yaw_control():
                    raise RuntimeError("底盘旋转线程退出未确认")
                self.yaw_started = False
            elif op == "linear_start":
                self.linear_worker = self.chassis.start_linear_motion(self.cancel, step["speed"], step["distance"])
            elif op == "linear_stop":
                if self.linear_worker is not None:
                    if not self.chassis.stop_linear_motion(self.linear_worker):
                        raise RuntimeError("底盘直行线程退出未确认")
                    self.linear_worker = None
                self.chassis.set_mode("idle")
            elif op == "loop_until":
                for iteration in range(step["max_iterations"]):
                    self._check()
                    if self.chassis.reached():
                        break
                    self._execute(step["cycles"][iteration % len(step["cycles"])], tick, hand, tracker)
                if not self.chassis.reached():
                    raise ProgramSequenceFailure("chassis target timeout: " + self.doc.name)

    def run(self, tick, hand, tracker, notify=None):
        self.notify = notify or (lambda phase: None)
        try:
            self._execute(self.doc.program, tick, hand, tracker)
        except ProgramSequenceFailure:
            # Original dance-only failure return: stop the base first. Never
            # clear a shared cancel/E-stop event or recover after servo faults.
            if self.doc.name == "dance" and self.stop_chassis() and not self.cancel.is_set():
                if not tracker.check_faults(("右臂",)):
                    hand(ActionEvent(0.0, "right_hand", "open"))
                    target = len(self.poses) - 1
                    for point in self.paths[(self.frame, target, 4.8)][1:]:
                        self._check()
                        tick([], point, "收尾")
                    self.frame = target
                    self.current = self.poses[target]
                    self._settle(10.0, tick, tracker)
            raise
        if self.current != self.poses[-1]:
            raise RuntimeError("program did not finish at zero")

    def stop_chassis(self):
        if not self.base_started:
            return True
        ok = True
        if self.yaw_started:
            ok = self.chassis.stop_yaw_control() and ok
            self.yaw_started = False
        if self.linear_worker is not None:
            ok = self.chassis.stop_linear_motion(self.linear_worker) and ok
            self.linear_worker = None
        self.chassis.set_mode("idle")
        self.base_started = False
        return ok
