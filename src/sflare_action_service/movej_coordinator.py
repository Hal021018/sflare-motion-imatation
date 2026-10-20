"""Small move_j backend for the six imitation actions; no EMI or interpolation.

Profile/dispatch/poll constants follow the original fixed-action source. JSON retains
the points, gesture ordering and five-second presentation; EMI durations are not
silently treated as move_j timing parameters.
"""
import math
import threading
import time

from action_pipeline import ActionEvent, enforce_runtime_safety
from action_session import extract_error_codes
from emi_coordinator import (EmiCoordinator, RIGHT_ARM_IP, CONTROLLER_PORT,
                             require_ok, power_state, safe_cleanup_arm)


GAME_ACTIONS = frozenset({'like', 'handshake', 'handover', 'diagonal_point', 'raise_head', 'doubt'})
NORMAL_PROFILE = (160, 0, 80, 80, .1, .1, -1, 0, 0, 1, 80)
SMOOTH_PROFILE = (160, 0, 50, 50, .1, .1, -1, 0, 0, 1, 50)
DANCE_PROFILE = (160, 0, 60, 60, .2, .2, -1, 0, 0, 1, 100)
MOVE_TIMEOUT_MS = 5000
POLL_SEC = .1
ARRIVAL_TIMEOUT_SEC = 35.
POSITION_TOLERANCE_DEG = 1.
SPEED_TOLERANCE_RAD_S = .05
SETTLED_SAMPLES_REQUIRED = 3


def profile_for(name):
    if name in {'like', 'handshake'}:
        return NORMAL_PROFILE
    return DANCE_PROFILE if name == 'doubt' else SMOOTH_PROFILE


class MoveJCoordinator(EmiCoordinator):
    """Reuse hand/resource helpers only; never call the EMI play/plan path."""

    def _position(self):
        raw = self.right_robot.get_joint_pos(timeout=1000)
        values = list(raw.toList()) if hasattr(raw, 'toList') else list(raw)
        return enforce_runtime_safety(values[:7])

    def _emit(self, kind, state):
        if self.event_callback:
            try:
                self.event_callback(kind, state)
            except Exception as exc:
                self.log('事件发布失败: '+str(exc))

    def _check(self, cancel, powered=False):
        if cancel.is_set():
            raise RuntimeError('收到停止/急停请求，中断move_j动作')
        status = self.right_controller.get_status()
        if not status or status[0] != 0:
            raise RuntimeError('控制器急停未释放或状态未知')
        if powered and power_state(self.right_robot) != 1:
            raise RuntimeError('move_j执行期间强电状态异常')

    def _wait(self, seconds, cancel, powered):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            self._check(cancel, powered)
            cancel.wait(min(POLL_SEC, max(0., deadline-time.monotonic())))
        self._check(cancel, powered)

    def _wait_target(self, target, cancel):
        deadline = time.monotonic()+ARRIVAL_TIMEOUT_SEC
        settled = 0
        while time.monotonic() < deadline:
            self._check(cancel, True)
            position = self._position()
            raw = self.right_robot.get_joint_speed(timeout=1000)
            speed = list(raw.toList()) if hasattr(raw, 'toList') else list(raw)
            if len(speed) < 7 or not all(math.isfinite(float(v)) for v in speed[:7]):
                raise RuntimeError('move_j速度反馈无效')
            error = max(abs(a-b) for a, b in zip(position, target))
            if error <= POSITION_TOLERANCE_DEG and max(map(abs, speed[:7])) <= SPEED_TOLERANCE_RAD_S:
                settled += 1
                if settled >= SETTLED_SAMPLES_REQUIRED:
                    self._check(cancel, True)
                    return
            else:
                settled = 0
            self._wait(POLL_SEC, cancel, True)
        raise RuntimeError('move_j动作阶段到位超时')

    def _preflight(self, document):
        if document.scope != 'right' or document.name not in GAME_ACTIONS | {'gozero'}:
            raise ValueError('move_j后端仅接受游戏六动作及右臂gozero')
        if document.speed_scale != 1.:
            raise ValueError('move_j不使用EMI speed_scale，请保留1.0')
        poses = [enforce_runtime_safety(frame.right) for frame in document.keyframes]
        if not poses or any(poses[-1]):
            raise ValueError('move_j动作必须以原零位结束')
        program = document.program or ({'op': 'move', 'frame': 0},)
        for step in program:
            if step.get('op') not in {'move', 'hand', 'wait'}:
                raise ValueError('游戏move_j不支持底盘或其他程序指令')
            if step['op'] == 'move':
                index = step.get('frame')
                if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < len(poses):
                    raise ValueError('move_j关键帧索引无效')
            if step['op'] == 'wait':
                seconds = step.get('seconds')
                if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not math.isfinite(seconds) or seconds < 0:
                    raise ValueError('move_j停留时间无效')
            if step['op'] == 'hand' and step.get('action') not in {'open', 'point', 'like', 'safe_fist', 'half_heart', 'chin_rest'}:
                raise ValueError('move_j手势无效')
        if program[-1].get('op') != 'move' or program[-1]['frame'] != len(poses)-1:
            raise ValueError('move_j动作最后必须执行零位关键帧')
        if self.right_robot.get_axis_number(timeout=1000) != 7:
            raise RuntimeError('move_j要求7轴机器人')
        limits = []
        for axis in range(1, 8):
            raw = list(self.right_robot.get_limit(axis, timeout=1000))
            if len(raw) < 2 or not all(math.isfinite(float(v)) for v in raw[:2]):
                raise RuntimeError('关节限位反馈无效')
            low, high = sorted(map(float, raw[:2]))
            if low >= high:
                raise RuntimeError('关节限位区间无效')
            limits.append((low, high))
        for pose in [self._position()]+poses:
            for axis, (value, (low, high)) in enumerate(zip(pose, limits), 1):
                if not low <= value <= high:
                    raise RuntimeError(f'move_j目标或起点越界: J{axis}={value}, limit=[{low},{high}]')
        return poses, program

    def play_document(self, document, cancel_event=None):
        cancel = cancel_event if cancel_event is not None else threading.Event()
        self.last_error = None
        self.safe_to_reuse = True
        self.diagnostic_codes = frozenset()
        self.diagnostics_complete = False
        connected = powered = completed = preexisting_power = False
        errors = []
        try:
            if cancel.is_set():
                raise RuntimeError('收到停止/急停请求，中断move_j动作')
            sdk = self._load_sdk()
            self.right_controller = sdk.Controller()
            require_ok(self.right_controller.connect(RIGHT_ARM_IP, CONTROLLER_PORT), '右臂connect')
            connected = True
            require_ok(self.right_controller.set_data_unit(1), '右臂角度单位')
            self.right_robot = self.right_controller.add_robot(1)
            self._check(cancel)
            if power_state(self.right_robot) != 0:
                preexisting_power = True
                raise RuntimeError('启动move_j前机械臂已上电，拒绝接管')
            poses, program = self._preflight(document)
            require_ok(self.right_robot.attach(timeout=MOVE_TIMEOUT_MS), '右臂attach')
            if self.right_robot.get_attached_id(timeout=1000) != 1:
                raise RuntimeError('右臂绑定ID未确认为1')
            require_ok(self.right_robot.set_speed(100, timeout=1000), '右臂speed')
            require_ok(self.right_robot.set_profile(sdk.Profile(*profile_for(document.name)), timeout=1000), '右臂Profile')
            hand_events = [ActionEvent(0., 'right_hand', step['action']) for step in program if step['op'] == 'hand']
            # Preserve existing initial hand opening, including safe thumb-first ordering.
            self._init_hands(hand_events, wait=lambda seconds: self._wait(seconds, cancel, powered))
            self._check(cancel)
            powered = True  # Failed enable can still have reached the device.
            require_ok(self.right_controller.set_power_enable(True, 1, timeout=3000), '右臂上电')
            power_deadline = time.monotonic()+5.
            while power_state(self.right_robot) != 1:
                if time.monotonic() >= power_deadline:
                    raise RuntimeError('右臂上电确认超时')
                self._wait(POLL_SEC, cancel, False)
            self._check(cancel, True)
            self._emit('power', 'on')
            self.log('开始move_j动作 ['+document.name+']')
            for step in program:
                self._check(cancel, True)
                op = step['op']
                if op == 'move':
                    index = step['frame']
                    self._emit('phase', 'reset' if index == len(poses)-1 else 'moving')
                    location = sdk.LocationJ(*poses[index])
                    require_ok(self.right_robot.move_j(location, timeout=MOVE_TIMEOUT_MS), '右臂move_j')
                    self._wait_target(poses[index], cancel)
                elif op == 'wait':
                    if step.get('phase') == 'still':
                        self._emit('phase', 'still')
                    self._wait(step['seconds'], cancel, True)
                elif op == 'hand':
                    self._dispatch_hand_event(ActionEvent(0., 'right_hand', step['action']))
            completed = True
        except Exception as exc:
            errors.append(str(exc))
            self.diagnostic_codes |= extract_error_codes(exc)
            self.log('move_j执行退出: '+str(exc))
        finally:
            if powered and not completed and self.right_robot is not None:
                try:
                    require_ok(self.right_robot.abort(timeout=MOVE_TIMEOUT_MS), '右臂abort')
                except Exception as exc:
                    errors.append('停止失败: '+str(exc))
            if self.right_hand is not None and self.right_controller is not None:
                try:
                    require_ok(self.right_controller.com_close('com2'), '右手关闭串口')
                except Exception as exc:
                    errors.append(str(exc))
                    self.safe_to_reuse = False
            if connected and preexisting_power:
                # We never attached or enabled this already-powered arm. Do not
                # cut another owner's power; release only our read-only connection.
                try:
                    require_ok(self.right_controller.disconnect(), '拒绝接管后释放只读连接')
                except Exception as exc:
                    errors.append(str(exc))
                    self.safe_to_reuse = False
            elif connected:
                confirmed = safe_cleanup_arm('右臂', self.right_controller, self.right_robot,
                                             errors, emi_attempted=False)
                self.safe_to_reuse = self.safe_to_reuse and confirmed
                if confirmed:
                    self._emit('power', 'off')
                    try:
                        diagnostic = self.right_controller.check_error()
                        self.diagnostic_codes |= extract_error_codes(str(diagnostic))
                        self.diagnostics_complete = True
                        self.log('右臂控制器诊断: '+str(diagnostic))
                    except Exception as exc:
                        errors.append('诊断读取失败: '+str(exc))
                    if self.safe_to_reuse:
                        try:
                            require_ok(self.right_controller.disconnect(), '右臂disconnect')
                        except Exception as exc:
                            errors.append(str(exc))
                            self.safe_to_reuse = False
                else:
                    self.log('未确认下电，禁止断连')
            self.last_error = '; '.join(errors) if errors else None
        return completed and not errors
