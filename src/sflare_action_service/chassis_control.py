"""Original chassis-only ROS wiring and odometry control, extracted without SDK code."""
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

CHASSIS_POSE_STALE_TIMEOUT_SEC = 1.0
CHASSIS_PROGRESS_WINDOW_SEC = 2.0
CHASSIS_MIN_PROGRESS_RAD = math.radians(1.0)
CHASSIS_REACHED_STABLE_SAMPLES = 5
CHASSIS_REACQUIRE_TIMEOUT_SEC = 3.0
CHASSIS_REACQUIRE_COMMAND_SCALE = 1.5
CHASSIS_REACQUIRE_COMMAND_ALLOWANCE_RAD = math.radians(15.0)
CHASSIS_REACQUIRE_COMMAND_MAX_RAD = math.radians(60.0)

class ChassisController:
    def __init__(self, node):
        self.node = node
        self._chassis_lock = threading.Lock()
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_remote_ctrl", 10)
        self.mode_client = self.create_client(StatusChange, "/mode_set")
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._on_odom, qos_profile_sensor_data
        )
        self._previous_yaw: Optional[float] = None
        self._continuous_yaw = 0.0
        self._last_pose_update_monotonic: Optional[float] = None
        self._target_yaw: Optional[float] = None
        self._chassis_reached = True
        self._chassis_reached_samples = 0
        self._chassis_active = False
        self._chassis_cancel_event: Optional[threading.Event] = None
        # Separate lifecycle events prevent a normal action cleanup from
        # relying on the shared action-cancel event.  In particular, a linear
        # worker must stop before its final zero command is published.
        self._chassis_stop_event = threading.Event()
        self._chassis_deadline: Optional[float] = None
        self._chassis_target_timeout_sec = 45.0
        self._chassis_kp = 1.3
        self._chassis_max_angular_speed = 1.0
        self._chassis_max_commanded_travel_rad: Optional[float] = None
        self._chassis_commanded_travel_rad = 0.0
        self._chassis_last_control_monotonic = time.monotonic()
        self._chassis_progress_window_started = time.monotonic()
        self._chassis_progress_window_yaw = 0.0
        self._chassis_failure_reason: Optional[str] = None
        self._chassis_thread: Optional[threading.Thread] = None
        self._linear_workers_lock = threading.Lock()
        self._linear_workers: Dict[int, Tuple[threading.Thread, threading.Event]] = {}

    def create_publisher(self, *args):
        return self.node.create_publisher(*args)

    def create_client(self, *args):
        return self.node.create_client(*args)

    def create_subscription(self, *args):
        return self.node.create_subscription(*args)

    def get_logger(self):
        return self.node.get_logger()

    def _on_odom(self, message: Odometry) -> None:
        q = message.pose.pose.orientation
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            return
        norm = math.hypot(*values)
        if norm < 1e-12:
            return
        x, y, z, w = (value / norm for value in values)
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        with self._chassis_lock:
            self._last_pose_update_monotonic = time.monotonic()
            if self._previous_yaw is None:
                self._previous_yaw = yaw
                self._continuous_yaw = yaw
                return
            delta = (yaw - self._previous_yaw + math.pi) % (2.0 * math.pi) - math.pi
            self._continuous_yaw += delta
            self._previous_yaw = yaw

    def has_pose(self) -> bool:
        with self._chassis_lock:
            return (
                self._previous_yaw is not None
                and self._last_pose_update_monotonic is not None
                and time.monotonic() - self._last_pose_update_monotonic
                <= CHASSIS_POSE_STALE_TIMEOUT_SEC
            )

    def current_yaw(self) -> float:
        with self._chassis_lock:
            return self._continuous_yaw

    def set_target_yaw(
        self,
        target: float,
        *,
        timeout_sec: Optional[float] = None,
        max_angular_speed: Optional[float] = None,
        max_commanded_travel_rad: Optional[float] = None,
    ) -> None:
        now = time.monotonic()
        with self._chassis_lock:
            if self._chassis_failure_reason is not None:
                raise ChassisMotionFailure(self._chassis_failure_reason)
            current = self._continuous_yaw
            self._target_yaw = float(target)
            self._chassis_reached = False
            self._chassis_reached_samples = 0
            if timeout_sec is not None:
                self._chassis_target_timeout_sec = float(timeout_sec)
            if max_angular_speed is not None:
                self._chassis_max_angular_speed = abs(float(max_angular_speed))
            self._chassis_max_commanded_travel_rad = (
                None
                if max_commanded_travel_rad is None
                else abs(float(max_commanded_travel_rad))
            )
            self._chassis_commanded_travel_rad = 0.0
            self._chassis_last_control_monotonic = now
            self._chassis_progress_window_started = now
            self._chassis_progress_window_yaw = current
            # Each heading target gets its own timeout.  A timeout is now a
            # terminal chassis failure, not merely a silent worker exit.
            if self._chassis_active:
                self._chassis_deadline = now + self._chassis_target_timeout_sec
            timeout_value = self._chassis_target_timeout_sec
            speed_value = self._chassis_max_angular_speed
            command_limit = self._chassis_max_commanded_travel_rad
        self.get_logger().info(
            "chassis yaw target: "
            f"current={current:.3f}rad target={float(target):.3f}rad "
            f"error={float(target) - current:.3f}rad "
            f"timeout={timeout_value:.1f}s max_speed={speed_value:.2f}rad/s "
            f"command_limit={command_limit}"
        )

    def reached(self) -> bool:
        with self._chassis_lock:
            return self._chassis_reached

    def chassis_failure_reason(self) -> Optional[str]:
        with self._chassis_lock:
            return self._chassis_failure_reason

    def raise_if_yaw_failed(self, phase: str) -> None:
        reason = self.chassis_failure_reason()
        if reason is not None:
            raise ChassisMotionFailure(f"{phase}: {reason}")

    def set_mode(self, mode: str) -> None:
        if self.mode_client.wait_for_service(timeout_sec=1.0):
            request = StatusChange.Request()
            request.action = mode
            self.mode_client.call_async(request)

    def _publish_chassis_speed(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        message = Twist()
        message.linear.x = float(linear_x)
        message.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(message)

    def start_yaw_control(
        self,
        cancel_event: threading.Event,
        timeout_sec: float = 45.0,
        kp: float = 1.3,
    ) -> None:
        with self._chassis_lock:
            self._chassis_active = True
            self._chassis_cancel_event = cancel_event
            self._chassis_target_timeout_sec = float(timeout_sec)
            self._chassis_deadline = time.monotonic() + self._chassis_target_timeout_sec
            self._chassis_kp = float(kp)
            self._chassis_max_angular_speed = 1.0
            self._chassis_max_commanded_travel_rad = None
            self._chassis_commanded_travel_rad = 0.0
            self._chassis_reached_samples = 0
            self._chassis_last_control_monotonic = time.monotonic()
            self._chassis_progress_window_started = time.monotonic()
            self._chassis_progress_window_yaw = self._continuous_yaw
            self._chassis_failure_reason = None
            self._chassis_stop_event = threading.Event()
            self._chassis_thread = threading.Thread(
                target=self._yaw_control_task,
                name="unified-yaw-control",
                daemon=True,
            )
            worker = self._chassis_thread
        worker.start()

    def _yaw_control_task(self) -> None:
        try:
            while True:
                now = time.monotonic()
                failure_reason = None
                command = 0.0
                reached_target = None
                reacquire_target = None
                with self._chassis_lock:
                    active = self._chassis_active
                    cancel_event = self._chassis_cancel_event
                    stop_event = getattr(self, "_chassis_stop_event", None)
                    deadline = self._chassis_deadline
                    target = self._target_yaw
                    current = self._continuous_yaw
                    kp = self._chassis_kp
                    if (
                        not active
                        or cancel_event is None
                        or cancel_event.is_set()
                        or (stop_event is not None and stop_event.is_set())
                    ):
                        break
                    if target is not None:
                        error = target - current
                        if abs(error) <= 0.05:
                            self._chassis_reached_samples += 1
                            if (
                                self._chassis_reached_samples
                                >= CHASSIS_REACHED_STABLE_SAMPLES
                            ):
                                if not self._chassis_reached:
                                    reached_target = (current, target)
                                self._chassis_reached = True
                                # This target has completed.  Keep the heading
                                # controller alive during presentation holds,
                                # but retire the old motion deadline so it
                                # cannot report a false timeout after arrival.
                                self._chassis_deadline = None
                                deadline = None
                                # The integrated-command hard limit belongs to
                                # the acquisition that just completed.  Retire
                                # it as well as the deadline; otherwise tiny
                                # heading corrections during ending-pose hold
                                # accumulate until they falsely trip the old
                                # budget after the target is already reached.
                                self._chassis_max_commanded_travel_rad = None
                                self._chassis_commanded_travel_rad = 0.0
                                self._chassis_progress_window_started = now
                                self._chassis_progress_window_yaw = current
                            elif deadline is not None and now >= deadline:
                                failure_reason = (
                                    "chassis yaw target timed out before stable arrival: "
                                    f"current={current:.3f}rad target={target}"
                                )
                            self._chassis_last_control_monotonic = now
                        else:
                            was_reached = self._chassis_reached
                            self._chassis_reached = False
                            self._chassis_reached_samples = 0
                            if was_reached:
                                # A completed target has drifted out of the
                                # tolerance.  Start a new bounded acquisition
                                # window instead of reviving its expired
                                # original deadline.
                                deadline = now + CHASSIS_REACQUIRE_TIMEOUT_SEC
                                self._chassis_deadline = deadline
                                reacquire_budget = min(
                                    CHASSIS_REACQUIRE_COMMAND_MAX_RAD,
                                    abs(error) * CHASSIS_REACQUIRE_COMMAND_SCALE
                                    + CHASSIS_REACQUIRE_COMMAND_ALLOWANCE_RAD,
                                )
                                self._chassis_max_commanded_travel_rad = reacquire_budget
                                self._chassis_commanded_travel_rad = 0.0
                                self._chassis_progress_window_started = now
                                self._chassis_progress_window_yaw = current
                                reacquire_target = (error, reacquire_budget)
                            pose_updated = self._last_pose_update_monotonic
                            if deadline is not None and now >= deadline:
                                failure_reason = (
                                    "chassis yaw target timed out: "
                                    f"current={current:.3f}rad target={target}"
                                )
                            elif (
                                pose_updated is None
                                or now - pose_updated > CHASSIS_POSE_STALE_TIMEOUT_SEC
                            ):
                                failure_reason = (
                                    "/odom feedback stale while turning: "
                                    f"age={None if pose_updated is None else now - pose_updated}"
                                )
                            elif (
                                now - self._chassis_progress_window_started
                                >= CHASSIS_PROGRESS_WINDOW_SEC
                            ):
                                window_progress = abs(
                                    current - self._chassis_progress_window_yaw
                                )
                                if window_progress < CHASSIS_MIN_PROGRESS_RAD:
                                    failure_reason = (
                                        "/odom yaw made no useful progress: "
                                        f"{math.degrees(window_progress):.2f}deg in "
                                        f"{CHASSIS_PROGRESS_WINDOW_SEC:.1f}s"
                                    )
                                else:
                                    self._chassis_progress_window_started = now
                                    self._chassis_progress_window_yaw = current

                            if failure_reason is None:
                                speed_limit = self._chassis_max_angular_speed
                                command = max(
                                    min(kp * error, speed_limit),
                                    -speed_limit,
                                )
                                elapsed = max(
                                    0.0,
                                    now - self._chassis_last_control_monotonic,
                                )
                                proposed_travel = (
                                    self._chassis_commanded_travel_rad
                                    + abs(command) * elapsed
                                )
                                command_limit = (
                                    self._chassis_max_commanded_travel_rad
                                )
                                if (
                                    command_limit is not None
                                    and proposed_travel >= command_limit
                                ):
                                    failure_reason = (
                                        "chassis commanded-angle hard limit reached: "
                                        f"{math.degrees(proposed_travel):.1f}deg >= "
                                        f"{math.degrees(command_limit):.1f}deg"
                                    )
                                else:
                                    self._chassis_commanded_travel_rad = proposed_travel
                                    self._chassis_last_control_monotonic = now
                    if failure_reason is not None:
                        self._chassis_failure_reason = failure_reason
                        self._chassis_active = False
                        self._chassis_reached = False
                        cancel_event.set()

                if failure_reason is not None:
                    self.get_logger().error(f"chassis yaw control failed: {failure_reason}")
                    break
                if reached_target is not None:
                    reached_current, reached_requested = reached_target
                    self.get_logger().info(
                        "chassis yaw target reached and stable: "
                        f"current={reached_current:.3f}rad "
                        f"target={reached_requested:.3f}rad"
                    )
                if reacquire_target is not None:
                    reacquire_error, reacquire_budget = reacquire_target
                    self.get_logger().warn(
                        "chassis heading drifted after stable arrival; "
                        "starting bounded reacquisition: "
                        f"error={math.degrees(reacquire_error):.2f}deg "
                        f"command_limit={math.degrees(reacquire_budget):.1f}deg"
                    )
                self._publish_chassis_speed(0.0, command)
                time.sleep(0.03)
        finally:
            self._publish_chassis_speed(0.0, 0.0)

    def stop_yaw_control(self) -> bool:
        with self._chassis_lock:
            self._chassis_active = False
            stop_event = getattr(self, "_chassis_stop_event", None)
            if stop_event is not None:
                stop_event.set()
            worker = self._chassis_thread
            self._chassis_thread = None
            self._chassis_deadline = None
            self._target_yaw = None
            self._chassis_reached_samples = 0
            self._chassis_max_commanded_travel_rad = None
            self._chassis_commanded_travel_rad = 0.0
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
            if worker.is_alive():
                self.get_logger().warn("chassis yaw worker did not exit within 2s")
        alive = worker is not None and worker.is_alive()
        # Publish zero only after the worker has observed its private stop
        # event.  This prevents a final non-zero command from racing the stop.
        self._publish_chassis_speed(0.0, 0.0)
        self.set_mode("idle")
        return not alive

    def start_linear_motion(
        self,
        cancel_event: threading.Event,
        speed: float,
        distance: float,
    ) -> threading.Thread:
        duration = abs(float(distance) / float(speed))
        stop_event = threading.Event()

        def run_linear() -> None:
            try:
                deadline = time.monotonic() + duration
                while (
                    not cancel_event.is_set()
                    and not stop_event.is_set()
                    and time.monotonic() < deadline
                ):
                    self._publish_chassis_speed(speed, 0.0)
                    stop_event.wait(0.05)
            finally:
                self._publish_chassis_speed(0.0, 0.0)
                with self._linear_workers_lock:
                    self._linear_workers.pop(id(threading.current_thread()), None)

        worker = threading.Thread(target=run_linear, name="unified-linear-control", daemon=True)
        # Keep the public return type compatible with existing action code,
        # while attaching a private cancellation handle for stop/shutdown.
        worker._linear_stop_event = stop_event  # type: ignore[attr-defined]
        with self._linear_workers_lock:
            self._linear_workers[id(worker)] = (worker, stop_event)
        worker.start()
        return worker

    def stop_linear_motion(self, worker: threading.Thread) -> bool:
        stop_event = getattr(worker, "_linear_stop_event", None)
        if isinstance(stop_event, threading.Event):
            stop_event.set()
        if worker.is_alive():
            worker.join(timeout=2.0)
        alive = worker.is_alive()
        if alive:
            self.get_logger().warn("chassis linear worker did not exit within 2s")
        # The worker's finally block and this final write are both after the
        # cancellation/join, so no later non-zero command from this worker can
        # overwrite the stop request.
        self._publish_chassis_speed(0.0, 0.0)
        with self._linear_workers_lock:
            self._linear_workers.pop(id(worker), None)
        return not alive

    def stop_all_linear_motion(self) -> bool:
        """Cancel and join every linear worker owned by this chassis object."""
        with self._linear_workers_lock:
            workers = [worker for worker, _ in self._linear_workers.values()]
        all_stopped = True
        for worker in workers:
            if not self.stop_linear_motion(worker):
                all_stopped = False
        if workers:
            self._publish_chassis_speed(0.0, 0.0)
        return all_stopped
