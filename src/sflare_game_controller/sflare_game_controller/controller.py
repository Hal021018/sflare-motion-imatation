"""Pallas 机械臂只读自检适配与假 SDK 实现。

本模块只保留**只读**能力：读取急停、轴数、关节位置与逐轴限位。
运动与 Jog 一律交给固定动作服务（``/dual_arm_xcdl_action``），游戏进程不再
直接驱动机械臂，因此这里不再有 ``movej`` / ``release_limits`` / ``abort``。

真实适配器按“连接 → 注册机器人 → 读取 → 释放”的顺序管理资源；假适配器提供
完全相同的方法，用于自动测试且绝不会连接实体控制器。
"""

import math
import threading
from typing import Any


class SafetyStateError(RuntimeError):
    """表示急停或安全状态无效，当前硬件操作必须被拒绝。"""


class PallasGameController:
    """封装只读自检所需的 SDK 生命周期。"""

    def __init__(self, controller_ip: str) -> None:
        self.controller_ip = controller_ip
        self.ctrl = None
        self.robot = None
        self._sdk_lock = threading.RLock()

    def _connect(self) -> None:
        """延迟导入 ARM64 SDK 并建立控制器会话。"""
        if self.ctrl is not None and self.ctrl.is_connected():
            return

        # 延迟导入让 x86 开发机可以运行假 SDK 测试，而不加载 ARM64 动态库。
        from PallasSDK import Controller

        self.ctrl = Controller()
        try:
            if self.ctrl.connect(self.controller_ip) != 0:
                raise SafetyStateError('控制器连接失败')
            if self.ctrl.set_data_unit(1) != 0:
                raise SafetyStateError('无法设置角度单位为度')
            self.robot = self.ctrl.add_robot(1)
        except Exception:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        """释放机器人对象并断开控制器，清除所有失效引用。"""
        with self._sdk_lock:
            self._disconnect_locked()

    def _disconnect_locked(self) -> None:
        ctrl = self.ctrl
        try:
            if ctrl is not None and ctrl.is_connected():
                try:
                    ctrl.reset_robots()
                finally:
                    ctrl.disconnect()
        finally:
            self.robot = None
            self.ctrl = None

    @staticmethod
    def _parse_emergency_stop(status: list[int]) -> bool:
        """按协议解析状态数组第 0 项，拒绝空数组和未知值。"""
        if not status:
            raise SafetyStateError('控制器状态数组为空，无法判断急停')
        if status[0] not in (0, 1):
            raise SafetyStateError(
                f'未知急停状态值：{status[0]}，只允许 0 或 1')
        return status[0] == 1

    @staticmethod
    def _normalise_limit(values: list[float]) -> tuple[float, float]:
        """将 SDK 可能采用的任意限位顺序归一化为下限和上限。"""
        if values is None or len(values) < 2:
            raise SafetyStateError('关节限位数据缺失')
        first, second = float(values[0]), float(values[1])
        if not math.isfinite(first) or not math.isfinite(second):
            raise SafetyStateError('关节限位包含非有限数值')
        return min(first, second), max(first, second)

    def _collect_safety_state(self) -> dict[str, Any]:
        """在现有连接中读取急停、关节位置和逐轴限位。"""
        emergency_stop = self._parse_emergency_stop(self.ctrl.get_status())
        axis_count = int(self.robot.get_axis_number())
        if axis_count != 7:
            raise SafetyStateError(f'本游戏要求7轴，实际反馈{axis_count}轴')
        positions_object = self.robot.get_joint_pos()
        positions = [float(positions_object[index])
                     for index in range(axis_count)]
        if not all(math.isfinite(value) for value in positions):
            raise SafetyStateError('关节位置包含NaN/Inf，拒绝自检和评分')
        lower_limits = []
        upper_limits = []
        overtravel_axes = []

        for axis in range(1, axis_count + 1):
            lower, upper = self._normalise_limit(self.robot.get_limit(axis))
            lower_limits.append(lower)
            upper_limits.append(upper)
            position = positions[axis - 1]
            if position < lower or position > upper:
                overtravel_axes.append(axis)

        return {
            'emergency_stop_status': '1' if emergency_stop else '0',
            'overtravel_status': '1' if overtravel_axes else '0',
            'overtravel_axes': overtravel_axes,
            'positions': positions,
            'lower_limits': lower_limits,
            'upper_limits': upper_limits,
        }

    def self_check(self, reference=None) -> dict[str, Any]:
        """执行只读安全自检，并在完成后释放 SDK 连接。"""
        with self._sdk_lock:
            self._connect()
            try:
                state = self._collect_safety_state()
                if (reference is not None and state['emergency_stop_status'] == '0'
                        and state['overtravel_status'] == '0'):
                    from PallasSDK import LocationJ
                    from .pose_scoring import finite_vector
                    reference = finite_vector(reference, 7, '标准姿态')
                    for label, angles in (('reference_xyz_mm', reference),
                                          ('current_xyz_mm', state['positions'])):
                        # 同一会话、同一工具/用户坐标配置，不执行任何运动。
                        location = self.robot.to_cartesian(LocationJ.from_list(angles))
                        state[label] = finite_vector(
                            [location.x, location.y, location.z], 3, label)
                    state['cartesian_source'] = 'sdk_forward_kinematics'
                return state
            finally:
                self.disconnect()


class FakeGameController:
    """不会接触硬件的确定性控制器，用于构建、联调和自动测试。"""

    def __init__(self, axis_count: int = 7) -> None:
        self.axis_count = axis_count
        self.emergency_stop = False
        self.positions = [0.0] * axis_count
        self.lower_limits = [-180.0] * axis_count
        self.upper_limits = [180.0] * axis_count
        self.events: list[tuple] = []
        self.reference_xyz_mm = [0., 0., 0.]
        self.current_xyz_mm = [0., 0., 0.]

    def self_check(self, reference=None) -> dict[str, Any]:
        """返回可由测试修改的假安全状态。"""
        overtravel_axes = [
            axis + 1
            for axis, position in enumerate(self.positions)
            if position < self.lower_limits[axis]
            or position > self.upper_limits[axis]
        ]
        result = {
            'emergency_stop_status': '1' if self.emergency_stop else '0',
            'overtravel_status': '1' if overtravel_axes else '0',
            'overtravel_axes': overtravel_axes,
            'positions': list(self.positions),
            'lower_limits': list(self.lower_limits),
            'upper_limits': list(self.upper_limits),
        }
        if reference is not None:
            result.update(reference_xyz_mm=list(self.reference_xyz_mm),
                          current_xyz_mm=list(self.current_xyz_mm),
                          cartesian_source='fake_test_fixture')
        return result

    def disconnect(self) -> None:
        """记录资源释放事件，保持与真实适配器相同的退出接口。"""
        self.events.append(('disconnect',))
