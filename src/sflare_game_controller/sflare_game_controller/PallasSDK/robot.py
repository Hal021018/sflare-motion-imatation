"""
robot.py

Created on: 2026-4-13
Author: 0590

Robot 类 ctypes 封装，对应 pybind11 的 Robot 绑定。
通过 RobotSDK C API 操作机器人实例。
"""
import ctypes
from typing import List, Optional, Callable

from ._lib import (
    _lib, _python_callbacks,
    _make_emi_feedback_callback_wrapper,
)
from ._utils import (
    PallasException,
    _call_int, _free_double_array, _free_int_array, _check_parameter,
    MACRO_CMD_TIMEOUT, MACRO_CMD_HANDLE, MACRO_MOVE_CMD_TIMEOUT
)
from ._types import (
    Location, LocationJ, Tool, UserFrame, Profile,
    ForceModeParam, Trigger, EMIFeedbackData, IPCCommandPacket,
)


# ---------------------------------------------------------------------------
# Robot 类
# ---------------------------------------------------------------------------

class Robot:
    """
    机器人类，通过 C API 操作机器人。

    Robot 实例通常由 Controller.get_robot() 创建。
    """

    def __init__(self, robot_id: int = 1):
        """
        创建 Robot 实例。

        @param robot_id 机器人编号（1-based）
        """
        self._robot_id = robot_id
        self._controller_id = 0
        self._ptr: Optional[ctypes.c_void_p] = None

    @property
    def robot_id(self) -> int:
        return self._robot_id

    def set_ptr(self, ptr: ctypes.c_void_p):
        """设置 C 指针（由 Controller.add_robot 调用）。"""
        self._ptr = ptr
        self._controller_id = self.get_controller_id()

    @property
    def _cptr(self) -> ctypes.c_void_p:
        if self._ptr is None:
            _check_parameter(self._controller_id, f"Robot({self._robot_id}) is not added to Controller")
        return self._ptr

    def _cptr_or_self(self) -> ctypes.c_void_p:
        """返回 C 指针（若有）或自身 void*（若无）。"""
        return self._cptr

    # -------------------- 基本信息 --------------------

    def get_id(self) -> int:
        """获取机器人编号。"""
        result = ctypes.c_uint8()
        _call_int(self._controller_id, _lib.robot_get_id, self._cptr_or_self(), ctypes.byref(result))
        return result.value

    def get_controller_id(self) -> int:
        """获取所属 Controller 实例的 ID 编号。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_controller_id, self._cptr_or_self(), ctypes.byref(result))
        return result.value

    def get_axis_number(self, timeout: int = MACRO_CMD_TIMEOUT,
                    handle: int = MACRO_CMD_HANDLE) -> int:
        """获取轴数。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_axis_number, self._cptr_or_self(),
                  ctypes.byref(result), timeout, handle)
        return result.value

    # -------------------- 状态 --------------------

    def get_state(self, timeout: int = MACRO_CMD_TIMEOUT,
                  handle: int = MACRO_CMD_HANDLE) -> List[int]:
        """获取机器人状态列表。"""
        result_ptr = ctypes.POINTER(ctypes.c_int)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_state, self._cptr_or_self(),
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_int_array(result_ptr)
        return result

    def get_cartesian_pos(self, timeout: int = MACRO_CMD_TIMEOUT,
                           handle: int = MACRO_CMD_HANDLE) -> Location:
        """获取笛卡尔位置。"""
        loc = Location()
        _call_int(self._controller_id, _lib.robot_get_cartesian_pos, self._cptr_or_self(),
                  ctypes.byref(loc), timeout, handle)
        return loc

    def get_joint_pos(self, timeout: int = MACRO_CMD_TIMEOUT,
                       handle: int = MACRO_CMD_HANDLE) -> LocationJ:
        """获取关节位置。"""
        locj = LocationJ()
        _call_int(self._controller_id, _lib.robot_get_joint_pos, self._cptr_or_self(),
                  ctypes.byref(locj), timeout, handle)
        return locj

    def get_mode(self, timeout: int = MACRO_CMD_TIMEOUT,
                  handle: int = MACRO_CMD_HANDLE) -> int:
        """获取机器人模式。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_mode, self._cptr_or_self(),
                  ctypes.byref(result), timeout, handle)
        return result.value

    # -------------------- 速度 --------------------

    def set_speed(self, speed: float,
                   timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> int:
        """设置速度（%）。"""
        return _call_int(self._controller_id, _lib.robot_set_speed, self._cptr_or_self(),
                        speed, timeout, handle)

    def get_speed(self, timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> float:
        """获取速度设置值（%）。"""
        result = ctypes.c_double()
        _call_int(self._controller_id, _lib.robot_get_speed, self._cptr_or_self(),
                  ctypes.byref(result), timeout, handle)
        return result.value

    def get_joint_speed(self, timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取关节速度（rad/s）。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_joint_speed, self._cptr_or_self(),
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    def get_cartesian_speed(self, timeout: int = MACRO_CMD_TIMEOUT,
                             handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取笛卡尔速度（mm/s, rad/s）。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_cartesian_speed, self._cptr_or_self(),
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    def get_target_joint_speed(self, timeout: int = MACRO_CMD_TIMEOUT,
                             handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取目标笛卡尔速度（mm/s, rad/s）。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_target_joint_speed, self._cptr_or_self(),
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    def get_target_cartesian_speed(self, timeout: int = MACRO_CMD_TIMEOUT,
                             handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取目标笛卡尔速度（mm/s, rad/s）。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_target_cartesian_speed, self._cptr_or_self(),
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    # -------------------- 工具坐标系 --------------------

    def set_tool(self, tool,
                  timeout: int = MACRO_CMD_TIMEOUT,
                  handle: int = MACRO_CMD_HANDLE) -> int:
        """设置工具坐标系。支持传入 Tool 实例或 Tool 编号。"""
        if isinstance(tool, int):
            return _call_int(self._controller_id, _lib.robot_set_tool_index, self._cptr_or_self(),
                             tool, timeout, handle)
        return _call_int(self._controller_id, _lib.robot_set_tool, self._cptr_or_self(),
                         ctypes.byref(tool), timeout, handle)

    def get_tool(self, timeout: int = MACRO_CMD_TIMEOUT,
                  handle: int = MACRO_CMD_HANDLE) -> Tool:
        """获取当前工具坐标系。"""
        tool = Tool()
        index = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_tool, self._cptr_or_self(),
                  ctypes.byref(tool), ctypes.byref(index), timeout, handle)
        tool.index = index.value
        return tool

    # -------------------- 用户坐标系 --------------------

    def set_userframe(self, userframe,
                       timeout: int = MACRO_CMD_TIMEOUT,
                       handle: int = MACRO_CMD_HANDLE) -> int:
        """设置用户坐标系。支持传入 UserFrame 实例或 UserFrame 编号。"""
        if isinstance(userframe, int):
            return _call_int(self._controller_id, _lib.robot_set_userframe_index, self._cptr_or_self(),
                             userframe, timeout, handle)
        return _call_int(self._controller_id, _lib.robot_set_userframe, self._cptr_or_self(),
                         ctypes.byref(userframe), timeout, handle)

    def get_userframe(self, timeout: int = MACRO_CMD_TIMEOUT,
                       handle: int = MACRO_CMD_HANDLE) -> UserFrame:
        """获取当前用户坐标系。"""
        uf = UserFrame()
        index = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_userframe, self._cptr_or_self(),
                  ctypes.byref(uf), ctypes.byref(index), timeout, handle)
        uf.index = index.value
        return uf

    # -------------------- 轨迹参数 Profile --------------------

    def set_profile(self, profile: Profile,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> int:
        """设置运动轨迹参数。"""
        return _call_int(self._controller_id, _lib.robot_set_profile, self._cptr_or_self(),
                         ctypes.byref(profile), timeout, handle)

    def get_profile(self, timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> Profile:
        """获取运动轨迹参数。"""
        prof = Profile()
        _call_int(self._controller_id, _lib.robot_get_profile, self._cptr_or_self(),
                  ctypes.byref(prof), timeout, handle)
        return prof

    # -------------------- 坐标系类型 --------------------

    def set_frame_type(self, mode: int,
                        timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> int:
        """设置坐标系类型。"""
        return _call_int(self._controller_id, _lib.robot_set_frame_type, self._cptr_or_self(),
                         mode, timeout, handle)

    def get_frame_type(self, timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> int:
        """获取坐标系类型。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_frame_type, self._cptr_or_self(),
                  ctypes.byref(result), timeout, handle)
        return result.value

    # -------------------- Jog --------------------

    def set_jog_mode(self, mode: int, on_off: bool = True,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> int:
        """设置 Jog 模式。"""
        return _call_int(self._controller_id, _lib.robot_set_jog_mode, self._cptr_or_self(),
                         mode, on_off, timeout, handle)

    def get_jog_mode(self, timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> List[int]:
        """获取 Jog 模式。"""
        result_ptr = ctypes.POINTER(ctypes.c_int)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_jog_mode, self._cptr_or_self(),
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_int_array(result_ptr)
        return result

    def set_jog_distance(self, value: float,
                           timeout: int = MACRO_CMD_TIMEOUT,
                           handle: int = MACRO_CMD_HANDLE) -> int:
        """设置 Jog 步进距离（仅 STEP 模式）。"""
        return _call_int(self._controller_id, _lib.robot_set_jog_distance, self._cptr_or_self(),
                         value, timeout, handle)

    def get_jog_distance(self, timeout: int = MACRO_CMD_TIMEOUT,
                           handle: int = MACRO_CMD_HANDLE) -> float:
        """获取 Jog 步进距离。"""
        result = ctypes.c_double()
        _call_int(self._controller_id, _lib.robot_get_jog_distance, self._cptr_or_self(),
                  ctypes.byref(result), timeout, handle)
        return result.value

    # -------------------- 负载 --------------------

    def set_payload(self, payload_index: int, m: float = 0.0,
                     x: float = 0.0, y: float = 0.0, z: float = 0.0,
                     ixx: float = 0.0, iyy: float = 0.0, izz: float = 0.0,
                     ixy: float = 0.0, iyz: float = 0.0, izx: float = 0.0,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> int:
        """设置负载参数。"""
        return _call_int(self._controller_id, _lib.robot_set_payload, self._cptr_or_self(),
                         payload_index, m, x, y, z, ixx, iyy, izz, ixy, iyz, izx,
                         timeout, handle)

    def get_payload(self, timeout: int = MACRO_CMD_TIMEOUT,
                           handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取负载参数 [M, x, y, z, Ixx, Iyy, Izz, Ixy, Iyz, Izx]。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_payload, self._cptr_or_self(),
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    def set_payload_data(self, payload_index: int, m: float,
                     x: float, y: float, z: float,
                     ixx: float, iyy: float, izz: float,
                     ixy: float, iyz: float, izx: float,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> int:
        """设置负载参数。"""
        return _call_int(self._controller_id, _lib.robot_set_payload_data, self._cptr_or_self(),
                         payload_index, m, x, y, z, ixx, iyy, izz, ixy, iyz, izx,
                         timeout, handle)

    def get_payload_data(self, payload_index: int,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取负载参数 [M, x, y, z, Ixx, Iyy, Izz, Ixy, Iyz, Izx]。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_payload_data, self._cptr_or_self(), payload_index,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    # -------------------- 限位/编码器 --------------------

    def get_limit(self, axis: int,
                   timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取轴限位 [min, max]。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_limit, self._cptr_or_self(), axis,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    def get_encoder(self, axis: int, type_: int = 1,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> int:
        """获取编码器值。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_encoder, self._cptr_or_self(),
                  axis, type_, ctypes.byref(result), timeout, handle)
        return result.value

    # -------------------- 目标位置 --------------------

    def get_target_locationj(self, timeout: int = MACRO_CMD_TIMEOUT,
                              handle: int = MACRO_CMD_HANDLE) -> LocationJ:
        """获取目标关节位置。"""
        locj = LocationJ()
        _call_int(self._controller_id, _lib.robot_get_target_locationj, self._cptr_or_self(),
                  ctypes.byref(locj), timeout, handle)
        return locj

    def get_target_location(self, timeout: int = MACRO_CMD_TIMEOUT,
                              handle: int = MACRO_CMD_HANDLE) -> Location:
        """获取目标笛卡尔位置。"""
        loc = Location()
        _call_int(self._controller_id, _lib.robot_get_target_location, self._cptr_or_self(),
                  ctypes.byref(loc), timeout, handle)
        return loc

    # -------------------- 力矩/温度/电流 --------------------

    def get_joint_torque(self, axis: int = -1,
                           timeout: int = MACRO_CMD_TIMEOUT,
                           handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取关节力矩（Nm）。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_joint_torque, self._cptr_or_self(), axis,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    def get_joint_temperature(self, axis: int = -1,
                                 timeout: int = MACRO_CMD_TIMEOUT,
                                 handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取关节温度（℃）。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_joint_temperature, self._cptr_or_self(), axis,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    def get_joint_current(self, axis: int = -1,
                            timeout: int = MACRO_CMD_TIMEOUT,
                            handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取关节电流（A）。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_joint_current, self._cptr_or_self(), axis,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    def get_joint_acceleration(self, axis: int = -1,
                                  timeout: int = MACRO_CMD_TIMEOUT,
                                  handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取关节加速度。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_joint_acceleration, self._cptr_or_self(), axis,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    def get_cartesian_force(self, timeout: int = MACRO_CMD_TIMEOUT,
                              handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取笛卡尔力（N, Nm）。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_cartesian_force, self._cptr_or_self(),
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    # -------------------- 坐标转换 --------------------

    def to_cartesian(self, pos: LocationJ,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> Location:
        """关节位置转笛卡尔位置。"""
        result = Location()
        _call_int(self._controller_id, _lib.robot_to_cartesian, self._cptr_or_self(),
                  ctypes.byref(pos), ctypes.byref(result), timeout, handle)
        return result

    def to_joint(self, pos: Location,
                  timeout: int = MACRO_CMD_TIMEOUT,
                  handle: int = MACRO_CMD_HANDLE) -> LocationJ:
        """笛卡尔位置转关节位置。"""
        result = LocationJ()
        _call_int(self._controller_id, _lib.robot_to_joint, self._cptr_or_self(),
                  ctypes.byref(pos), ctypes.byref(result), timeout, handle)
        return result

    # -------------------- 安装姿态 --------------------

    def set_mount_pos(self, x: float, y: float, z: float,
                       rx: float, ry: float, rz: float,
                       timeout: int = MACRO_CMD_TIMEOUT,
                       handle: int = MACRO_CMD_HANDLE) -> int:
        """设置安装姿态。"""
        return _call_int(self._controller_id, _lib.robot_set_mount_pos, self._cptr_or_self(),
                         x, y, z, rx, ry, rz, timeout, handle)

    def get_mount_pos(self, timeout: int = MACRO_CMD_TIMEOUT,
                       handle: int = MACRO_CMD_HANDLE) -> List[float]:
        """获取安装姿态 [x, y, z, Rx, Ry, Rz]。"""
        result_ptr = ctypes.POINTER(ctypes.c_double)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_mount_pos, self._cptr_or_self(),
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_double_array(result_ptr)
        return result

    # -------------------- 碰撞保护 --------------------

    def set_collision_protection(self, on_off: int,
                                   timeout: int = MACRO_CMD_TIMEOUT,
                                   handle: int = MACRO_CMD_HANDLE) -> int:
        """设置碰撞保护（0=关闭，1=开启）。"""
        return _call_int(self._controller_id, _lib.robot_set_collision_protection, self._cptr_or_self(),
                         on_off, timeout, handle)

    def get_collision_protection(self, timeout: int = MACRO_CMD_TIMEOUT,
                                   handle: int = MACRO_CMD_HANDLE) -> bool:
        """获取碰撞保护状态。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_collision_protection, self._cptr_or_self(),
                  ctypes.byref(result), timeout, handle)
        return (result.value > 0)

    def set_collision_sensitivity(self, sensitivity: int,
                                    timeout: int = MACRO_CMD_TIMEOUT,
                                    handle: int = MACRO_CMD_HANDLE) -> int:
        """设置碰撞灵敏度（1-100）。"""
        return _call_int(self._controller_id, _lib.robot_set_collision_sensitivity, self._cptr_or_self(),
                         sensitivity, timeout, handle)

    def get_collision_sensitivity(self, timeout: int = MACRO_CMD_TIMEOUT,
                                     handle: int = MACRO_CMD_HANDLE) -> int:
        """获取碰撞灵敏度。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_collision_sensitivity, self._cptr_or_self(),
                  ctypes.byref(result), timeout, handle)
        return result.value

    # -------------------- 报警 --------------------

    def clear_battery_alarm(self, axis: int = -1,
                             timeout: int = MACRO_CMD_TIMEOUT,
                             handle: int = MACRO_CMD_HANDLE) -> int:
        """清除电池报警。"""
        return _call_int(self._controller_id, _lib.robot_clear_battery_alarm, self._cptr_or_self(),
                         axis, timeout, handle)

    # -------------------- 力控 --------------------

    def set_force_mode_param(self, *args, **kwargs) -> int:
        """设置力控参数。

        支持两种调用方式：
        1. set_force_mode_param(frame, selector, wrench, limit, source, time=0, method=0, distance=0.0, timeout=MACRO_CMD_TIMEOUT, handle=MACRO_CMD_HANDLE)
        2. set_force_mode_param(param, timeout=MACRO_CMD_TIMEOUT, handle=MACRO_CMD_HANDLE)
        """
        if len(args) == 1:
            # 第一种调用方式：传入 ForceModeParam 结构体
            param = args[0]
            timeout = kwargs.get('timeout', MACRO_CMD_TIMEOUT)
            handle = kwargs.get('handle', MACRO_CMD_HANDLE)
            return _call_int(self._controller_id, _lib.robot_set_force_mode_param, self._cptr_or_self(),
                             ctypes.byref(param), timeout, handle)
        elif len(args) >= 5:
            # 第二种调用方式：传入各个参数
            frame = args[0]
            selector = args[1]
            wrench = args[2]
            limit = args[3]
            source = args[4]
            time = args[5] if len(args) > 5 else kwargs.get('time', 0)
            method = args[6] if len(args) > 6 else kwargs.get('method', 0)
            distance = args[7] if len(args) > 7 else kwargs.get('distance', 0.0)
            timeout = args[8] if len(args) > 8 else kwargs.get('timeout', MACRO_CMD_TIMEOUT)
            handle = args[9] if len(args) > 9 else kwargs.get('handle', MACRO_CMD_HANDLE)

            # 创建并填充参数结构体
            param = ForceModeParam()
            param.frame = frame

            # 填充 selector（6个bool值）
            if len(selector) != 6:
                _check_parameter(self._controller_id, f"selector must have exactly 6 elements, got {len(selector)}")
            param.selector = selector

            # 填充 wrench（6个double值）
            if len(wrench) != 6:
                _check_parameter(self._controller_id, f"wrench must have exactly 6 elements, got {len(wrench)}")
            param.wrench = wrench

            # 填充 limit（6个double值）
            if len(limit) != 6:
                _check_parameter(self._controller_id, f"limit must have exactly 6 elements, got {len(limit)}")
            param.limit = limit

            param.source = source
            param.time = time
            param.method = method
            param.distance = distance

            return _call_int(self._controller_id, _lib.robot_set_force_mode_param, self._cptr_or_self(),
                             ctypes.byref(param), timeout, handle)
        else:
            _check_parameter(self._controller_id, "set_force_mode_param() requires either 1 argument (ForceModeParam) or at least 5 arguments (frame, selector, wrench, limit, source)")

    def get_force_mode_param(self, timeout: int = MACRO_CMD_TIMEOUT,
                               handle: int = MACRO_CMD_HANDLE) -> ForceModeParam:
        """获取力控参数。"""
        param = ForceModeParam()
        _call_int(self._controller_id, _lib.robot_get_force_mode_param, self._cptr_or_self(),
                  ctypes.byref(param), timeout, handle)
        return param

    def set_force_mode(self, mode: int,
                        timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> int:
        """设置力控模式。"""
        return _call_int(self._controller_id, _lib.robot_set_force_mode, self._cptr_or_self(),
                         mode, timeout, handle)

    def set_free_drive_mode(self, on_off: int,
                              timeout: int = MACRO_CMD_TIMEOUT,
                              handle: int = MACRO_CMD_HANDLE) -> int:
        """设置示教模式（0=关闭，1=开启）。"""
        return _call_int(self._controller_id, _lib.robot_set_free_drive_mode, self._cptr_or_self(),
                         on_off, timeout, handle)

    # -------------------- 操作模式 --------------------

    def set_operate_mode(self, mode: int,
                           timeout: int = MACRO_CMD_TIMEOUT,
                           handle: int = MACRO_CMD_HANDLE) -> int:
        """设置操作模式。"""
        return _call_int(self._controller_id, _lib.robot_set_operate_mode, self._cptr_or_self(),
                         mode, timeout, handle)

    def get_operate_mode(self, timeout: int = MACRO_CMD_TIMEOUT,
                           handle: int = MACRO_CMD_HANDLE) -> int:
        """获取操作模式。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_operate_mode, self._cptr_or_self(),
                  ctypes.byref(result), timeout, handle)
        return result.value

    # -------------------- 绑定/分离 --------------------

    def attach(self, timeout: int = MACRO_CMD_TIMEOUT,
                handle: int = MACRO_CMD_HANDLE) -> int:
        """绑定机器人。"""
        return _call_int(self._controller_id, _lib.robot_attach, self._cptr_or_self(), timeout, handle)

    def detach(self, timeout: int = MACRO_CMD_TIMEOUT,
                handle: int = MACRO_CMD_HANDLE) -> int:
        """分离机器人。"""
        return _call_int(self._controller_id, _lib.robot_detach, self._cptr_or_self(), timeout, handle)

    def get_attached_id(self, timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> int:
        """获取绑定机器人 ID 号，若绑定，则与机器人 ID 相同，若未绑定，ID 号为0。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_attached_id, self._cptr_or_self(),
                  ctypes.byref(result), timeout, handle)
        return result.value

    # -------------------- 抱闸 --------------------

    def set_axis_brake(self, axis: int, state: int,
                        timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> int:
        """设置轴抱闸状态（0=抱紧，1=松开）。"""
        return _call_int(self._controller_id, _lib.robot_set_axis_brake, self._cptr_or_self(),
                         axis, state, timeout, handle)

    def get_axis_brake(self, timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> List[int]:
        """获取各轴抱闸状态列表。"""
        result_ptr = ctypes.POINTER(ctypes.c_int)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_axis_brake, self._cptr_or_self(),
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_int_array(result_ptr)
        return result

    # -------------------- 轴标定 --------------------

    def set_axis_calib(self, axis: int, value: int,
                        timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> int:
        """设置轴标定值。"""
        return _call_int(self._controller_id, _lib.robot_set_axis_calib, self._cptr_or_self(),
                         axis, value, timeout, handle)

    def get_axis_calib(self, axis: int,
                        timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> int:
        """获取轴标定值。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_axis_calib, self._cptr_or_self(), axis,
                  ctypes.byref(result), timeout, handle)
        return result.value

    # -------------------- 运动 --------------------

    def abort(self, timeout: int = MACRO_CMD_TIMEOUT,
               handle: int = MACRO_CMD_HANDLE) -> int:
        """立即停止所有运动。"""
        return _call_int(self._controller_id, _lib.robot_abort, self._cptr_or_self(), timeout, handle)

    def move_l(self, pos,
                timeout: int = MACRO_MOVE_CMD_TIMEOUT,
                handle: int = MACRO_CMD_HANDLE) -> int:
        """
        直线运动（对应 C++ Robot::move_l，支持 Location / LocationJ 重载）。

        @param pos      目标点位（Location 或 LocationJ）
        @param timeout  超时（ms）
        @param handle   指令句柄
        @return 0 表示成功，负值表示错误码
        """
        if isinstance(pos, LocationJ):
            return _call_int(self._controller_id, _lib.robot_movel_locationj, self._cptr_or_self(),
                             ctypes.byref(pos), timeout, handle)
        return _call_int(self._controller_id, _lib.robot_movel_location, self._cptr_or_self(),
                         ctypes.byref(pos), timeout, handle)

    def move_j(self, pos,
                timeout: int = MACRO_MOVE_CMD_TIMEOUT,
                handle: int = MACRO_CMD_HANDLE) -> int:
        """
        关节运动（对应 C++ Robot::move_j，支持 Location / LocationJ 重载）。

        @param pos      目标点位（Location 或 LocationJ）
        @param timeout  超时（ms）
        @param handle   指令句柄
        @return 0 表示成功，负值表示错误码
        """
        if isinstance(pos, LocationJ):
            return _call_int(self._controller_id, _lib.robot_movej_locationj, self._cptr_or_self(),
                             ctypes.byref(pos), timeout, handle)
        return _call_int(self._controller_id, _lib.robot_movej_location, self._cptr_or_self(),
                         ctypes.byref(pos), timeout, handle)

    def move_arc(self, pos1, pos2,
                  timeout: int = MACRO_MOVE_CMD_TIMEOUT,
                  handle: int = MACRO_CMD_HANDLE) -> int:
        """
        圆弧运动（对应 C++ Robot::move_arc，支持 Location / LocationJ 重载）。

        @param pos1 / pos2  途经点和终点（Location 或 LocationJ，类型需一致）
        @param timeout      超时（ms）
        @param handle       指令句柄
        @return 0 表示成功，负值表示错误码
        """
        if isinstance(pos1, LocationJ):
            return _call_int(self._controller_id, _lib.robot_move_arc_locationj, self._cptr_or_self(),
                             ctypes.byref(pos1), ctypes.byref(pos2), timeout, handle)
        return _call_int(self._controller_id, _lib.robot_move_arc_location, self._cptr_or_self(),
                         ctypes.byref(pos1), ctypes.byref(pos2), timeout, handle)

    def move_circle(self, pos1, pos2,
                     timeout: int = MACRO_MOVE_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> int:
        """
        整圆运动（对应 C++ Robot::move_circle，支持 Location / LocationJ 重载）。

        @param pos1 / pos2  圆上两点（Location 或 LocationJ，类型需一致）
        @param timeout      超时（ms）
        @param handle       指令句柄
        @return 0 表示成功，负值表示错误码
        """
        if isinstance(pos1, LocationJ):
            return _call_int(self._controller_id, _lib.robot_move_circle_locationj, self._cptr_or_self(),
                             ctypes.byref(pos1), ctypes.byref(pos2), timeout, handle)
        return _call_int(self._controller_id, _lib.robot_move_circle_location, self._cptr_or_self(),
                         ctypes.byref(pos1), ctypes.byref(pos2), timeout, handle)

    def move_axis(self, axis: int, angle: float,
                   timeout: int = MACRO_MOVE_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> int:
        """单轴运动。"""
        return _call_int(self._controller_id, _lib.robot_move_axis, self._cptr_or_self(),
                         axis, angle, timeout, handle)

    def move_along(self, direction: str, speed: float,
                     timeout_ms: int = 0,
                     timeout: int = 0,
                     handle: int = MACRO_CMD_HANDLE) -> int:
        """沿方向连续 Jog。"""
        return _call_int(self._controller_id, _lib.robot_move_along, self._cptr_or_self(),
                         direction.encode("utf-8"), speed, timeout_ms, timeout, handle)

    def move_stop_jog(self, timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> int:
        """停止 Jog。"""
        return _call_int(self._controller_id, _lib.robot_move_stop_jog, self._cptr_or_self(), timeout, handle)

    def move_wait_for_eom(self) -> int:
        """等待运动完成。"""
        return _call_int(self._controller_id, _lib.robot_move_wait_for_eom, self._cptr_or_self())

    def move_l_offset(self, *args, **kwargs) -> int:
        """直线偏移运动。
        支持三种调用方式：
        1. move_l_offset(x, y, z, yaw=0, pitch=0, roll=0, timeout=MACRO_CMD_TIMEOUT, handle=MACRO_CMD_HANDLE)
        2. move_l_offset(x, y, z, yaw, pitch, roll, location, timeout=MACRO_CMD_TIMEOUT, handle=MACRO_CMD_HANDLE)
        3. move_l_offset(x, y, z, yaw, pitch, roll, locationj, timeout=MACRO_CMD_TIMEOUT, handle=MACRO_CMD_HANDLE)
        """
        from ._types import Location, LocationJ

        # 处理参数
        if len(args) >= 3:
            x, y, z = args[0], args[1], args[2]
            yaw = args[3] if len(args) > 3 else kwargs.get('yaw', 0)
            pitch = args[4] if len(args) > 4 else kwargs.get('pitch', 0)
            roll = args[5] if len(args) > 5 else kwargs.get('roll', 0)

            # 检查是否有目标位置参数
            if len(args) > 6:
                target = args[6]
                timeout = args[7] if len(args) > 7 else kwargs.get('timeout', MACRO_MOVE_CMD_TIMEOUT)
                handle = args[8] if len(args) > 8 else kwargs.get('handle', MACRO_CMD_HANDLE)

                # 根据目标位置类型调用不同的方法
                if isinstance(target, Location):
                    return _call_int(self._controller_id, _lib.robot_move_l_offset_location, self._cptr_or_self(),
                                     x, y, z, yaw, pitch, roll, ctypes.byref(target), timeout, handle)
                elif isinstance(target, LocationJ):
                    return _call_int(self._controller_id, _lib.robot_move_l_offset_locationj, self._cptr_or_self(),
                                     x, y, z, yaw, pitch, roll, ctypes.byref(target), timeout, handle)
                else:
                    _check_parameter(self._controller_id, "move_l_offset requires the 7th argument to be Location or LocationJ")
            else:
                # 第一种调用方式
                timeout = kwargs.get('timeout', MACRO_MOVE_CMD_TIMEOUT)
                handle = kwargs.get('handle', MACRO_CMD_HANDLE)
                return _call_int(self._controller_id, _lib.robot_move_l_offset, self._cptr_or_self(),
                                 x, y, z, yaw, pitch, roll, timeout, handle)
        else:
            _check_parameter(self._controller_id, "move_l_offset requires at least 3 arguments (x, y, z)")

    def move_l_offset_jm(self, *args, **kwargs) -> int:
        """直线偏移运动。
        支持三种调用方式：
        1. move_l_offset_jm(x, y, z, yaw=0, pitch=0, roll=0, timeout=MACRO_CMD_TIMEOUT, handle=MACRO_CMD_HANDLE)
        2. move_l_offset_jm(x, y, z, yaw, pitch, roll, location, timeout=MACRO_CMD_TIMEOUT, handle=MACRO_CMD_HANDLE)
        3. move_l_offset_jm(x, y, z, yaw, pitch, roll, locationj, timeout=MACRO_CMD_TIMEOUT, handle=MACRO_CMD_HANDLE)
        """
        from ._types import Location, LocationJ

        # 处理参数
        if len(args) >= 3:
            x, y, z = args[0], args[1], args[2]
            yaw = args[3] if len(args) > 3 else kwargs.get('yaw', 0)
            pitch = args[4] if len(args) > 4 else kwargs.get('pitch', 0)
            roll = args[5] if len(args) > 5 else kwargs.get('roll', 0)

            # 检查是否有目标位置参数
            if len(args) > 6:
                target = args[6]
                timeout = args[7] if len(args) > 7 else kwargs.get('timeout', MACRO_MOVE_CMD_TIMEOUT)
                handle = args[8] if len(args) > 8 else kwargs.get('handle', MACRO_CMD_HANDLE)

                # 根据目标位置类型调用不同的方法
                if isinstance(target, Location):
                    return _call_int(self._controller_id, _lib.robot_move_l_offset_jm_location, self._cptr_or_self(),
                                     x, y, z, yaw, pitch, roll, ctypes.byref(target), timeout, handle)
                elif isinstance(target, LocationJ):
                    return _call_int(self._controller_id, _lib.robot_move_l_offset_jm_locationj, self._cptr_or_self(),
                                     x, y, z, yaw, pitch, roll, ctypes.byref(target), timeout, handle)
                else:
                    _check_parameter(self._controller_id, "move_l_offset_jm requires the 7th argument to be Location or LocationJ")
            else:
                # 第一种调用方式
                timeout = kwargs.get('timeout', MACRO_MOVE_CMD_TIMEOUT)
                handle = kwargs.get('handle', MACRO_CMD_HANDLE)
                return _call_int(self._controller_id, _lib.robot_move_l_offset_jm, self._cptr_or_self(),
                                 x, y, z, yaw, pitch, roll, timeout, handle)
        else:
            _check_parameter(self._controller_id, "move_l_offset_jm requires at least 3 arguments (x, y, z)")

    # -------------------- 触发器 --------------------

    def move_trigger(self, trigger_or_type, ref_point_or_none=None,
                     condition: float = 0, out_type: int = 0,
                     index: int = 0, value: int = 0, num: int = 1,
                     delay_time: float = 0,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> int:
        """
        运动触发器（支持 Trigger 结构体或独立参数）。

        参数形式 1（Trigger 结构体）:
            move_trigger(trigger: Trigger, timeout=..., handle=...)

        参数形式 2（独立参数）:
            move_trigger(type, ref_point, condition, out_type, index, value,
                         num=1, delay_time=0, timeout=..., handle=...)
        """
        if isinstance(trigger_or_type, Trigger):
            t = trigger_or_type
            return _call_int(self._controller_id, _lib.robot_move_trigger, self._cptr_or_self(),
                             t.type, t.refPoint, t.condition, t.outType,
                             t.index, t.value, t.num, t.delayTime, timeout, handle)
        return _call_int(self._controller_id, _lib.robot_move_trigger, self._cptr_or_self(),
                         trigger_or_type, ref_point_or_none, condition, out_type,
                         index, value, num, delay_time, timeout, handle)

    # -------------------- 运动连续类型 --------------------

    def get_move_continue_type(self, timeout: int = MACRO_CMD_TIMEOUT,
                                  handle: int = MACRO_CMD_HANDLE) -> int:
        """获取运动连续类型。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.robot_get_move_continue_type, self._cptr_or_self(),
                  ctypes.byref(result), timeout, handle)
        return result.value

    def set_move_continue_type(self, type_: int,
                                  timeout: int = MACRO_CMD_TIMEOUT,
                                  handle: int = MACRO_CMD_HANDLE) -> int:
        """设置运动连续类型。"""
        return _call_int(self._controller_id, _lib.robot_set_move_continue_type, self._cptr_or_self(),
                         type_, timeout, handle)

    # -------------------- EMI --------------------

    def set_emi_feedback_callback(self, callback: Callable[[EMIFeedbackData], None]) -> int:
        """
        设置 EMI 反馈数据回调。

        @param callback 回调函数，签名为: callback(data: EMIFeedbackData) -> None
        """
        wrapper = _make_emi_feedback_callback_wrapper(callback)
        _python_callbacks["emi_feedback"][id(self)] = wrapper
        return _call_int(self._controller_id, _lib.robot_set_emi_feedback_callback, self._cptr_or_self(), wrapper)

    def emi_enable(self, is_enable: int,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> int:
        """EMI 使能。"""
        return _call_int(self._controller_id, _lib.robot_emi_enable, self._cptr_or_self(),
                         is_enable, timeout, handle)

    def emi_enable_buffer(self, is_enable: int,
                            timeout: int = MACRO_CMD_TIMEOUT,
                            handle: int = MACRO_CMD_HANDLE) -> int:
        """EMI Buffer 使能。"""
        return _call_int(self._controller_id, _lib.robot_emi_enable_buffer, self._cptr_or_self(),
                         is_enable, timeout, handle)

    def emi_set_parameter(self, scenes: int, period: float,
                           repeat: int, check_receive_timeout: int,
                           timeout: int = MACRO_CMD_TIMEOUT,
                           handle: int = MACRO_CMD_HANDLE) -> int:
        """EMI 参数设置（与 pybind11 对齐）。"""
        return _call_int(self._controller_id, _lib.robot_emi_set_parameter, self._cptr_or_self(),
                         scenes, period, repeat, check_receive_timeout, timeout, handle)

    def emi_run(self, timeout: int = MACRO_CMD_TIMEOUT,
                  handle: int = MACRO_CMD_HANDLE) -> int:
        """EMI 运行（超时+句柄对齐 pybind11）。"""
        return _call_int(self._controller_id, _lib.robot_emi_run, self._cptr_or_self(), timeout, handle)

    def emi_stop(self, timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> int:
        """EMI 停止。"""
        return _call_int(self._controller_id, _lib.robot_emi_stop, self._cptr_or_self(), timeout, handle)

    def emi_start(self, ctrl_word: int, scenes: int, period: float,
                    repeat: int, check_receive_timeout: int = 0,
                    timeout: int = MACRO_CMD_TIMEOUT,
                    handle: int = MACRO_CMD_HANDLE) -> int:
        """EMI 启动（对应 pybind11 emi_start）。"""
        return _call_int(self._controller_id, _lib.robot_emi_start, self._cptr_or_self(),
                         ctrl_word, scenes, period, repeat, check_receive_timeout,
                         timeout, handle)

    def emi_end(self, timeout: int = MACRO_CMD_TIMEOUT,
                 handle: int = MACRO_CMD_HANDLE) -> int:
        """EMI 结束。"""
        return _call_int(self._controller_id, _lib.robot_emi_end, self._cptr_or_self(), timeout, handle)

    def emi_set_data(self, data_type: int,
                     data: Optional[List[float]],
                     acc_data: Optional[List[float]] = [0] * 4,
                     time: float = 0.0,
                     do_index: Optional[List[int]] = [0] * 16,
                     do_value: int = 0,
                     do_time_percent: Optional[List[float]] = [0] * 16,
                     vel_data: Optional[List[float]] = [0] * 12,
                     force_data: Optional[List[float]] = [0] * 12) -> int:
        """
        设置 EMI 运动数据（对应 pybind11 emi_set_data 重载）。

        @param data_type 数据类型
        @param data 位置/速度数据（双精度浮点数组）, 12 个元素
        @param acc_data 加速度数据, 4 个元素
        @param time 时间
        @param do_index DO 索引数组, 16 个元素
        @param do_value DO 值（位掩码）
        @param do_time_percent DO 时间百分比数组, 16 个元素
        @param vel_data 速度数据, 12 个元素
        @param force_data 力数据, 12 个元素
        @return 错误码，0 表示成功
        """
        def to_c_arr(val, ctype):
            arr = (ctype * len(val))(*val)
            return arr

        if data == None or len(data) != 12:
            _check_parameter(self._controller_id, "List size must be 12")
        if acc_data == None or len(acc_data) != 4:
            _check_parameter(self._controller_id, "List size must be 4")
        if do_index == None or len(do_index) != 16 or do_time_percent == None or len(do_time_percent) != 16:
            _check_parameter(self._controller_id, "List size must be 16")
        if vel_data == None or len(vel_data) != 12 or force_data == None or len(force_data) != 12:
            _check_parameter(self._controller_id, "List size must be 12")

        data_arr = to_c_arr(data, ctypes.c_double)
        acc_arr = to_c_arr(acc_data, ctypes.c_double)
        do_idx_arr = to_c_arr(do_index, ctypes.c_ushort) if do_index else ctypes.POINTER(ctypes.c_ushort)()
        do_tp_arr = to_c_arr(do_time_percent, ctypes.c_float) if do_time_percent else ctypes.POINTER(ctypes.c_float)()
        vel_arr = to_c_arr(vel_data, ctypes.c_double)
        force_arr = to_c_arr(force_data, ctypes.c_double)
        data_len = len(data) if data else 0
        do_len = len(do_index) if do_index else 0
        # 注意：ctypes 数组传递给 POINTER(type) 参数时自动转换为指针，
        # 无需额外 byref() —— byref(arr) 会产生 double pointer，是错误的。
        return _lib.robot_emi_set_data(
            self._cptr_or_self(), data_type,
            data_arr,  # ctypes 数组自动 decay 为 double*
            acc_arr,  # double*
            time,
            ctypes.cast(do_idx_arr, ctypes.POINTER(ctypes.c_ushort)),  # 空指针需要 cast
            do_len,
            ctypes.cast(do_tp_arr, ctypes.POINTER(ctypes.c_float)),
            vel_arr,  # double*
            force_arr)  # double*

    # -------------------- 伺服控制 --------------------

    def servo_j(self, q: List[float], a: float = 1.0, v: float = 1.0,
                  t: float = 0.1, lookahead_time: float = 0.1,
                  gain: float = 1.0) -> int:
        """伺服关节位置控制。"""
        arr = (ctypes.c_double * len(q))(*q)
        return _lib.robot_servo_j(self._cptr_or_self(), arr, a, v, t, lookahead_time, gain)

    def servo_l(self, pose: List[float], a: float = 1.0, v: float = 1.0,
                  t: float = 0.1, lookahead_time: float = 0.1,
                  gain: float = 1.0) -> int:
        """伺服直线位置控制。"""
        arr = (ctypes.c_double * len(pose))(*pose)
        return _lib.robot_servo_l(self._cptr_or_self(), arr, a, v, t, lookahead_time, gain)

    def speed_j(self, q_vel: List[float], rotation_acc: float = 1.0,
                  trans_acc: float = 1.0, t: float = 0.1) -> int:
        """速度控制（关节空间）。"""
        arr = (ctypes.c_double * len(q_vel))(*q_vel)
        return _lib.robot_speed_j(self._cptr_or_self(), arr, rotation_acc, trans_acc, t)

    def speed_l(self, c_vel: List[float], rotation_acc: float = 1.0,
                  trans_acc: float = 1.0, t: float = 0.1) -> int:
        """速度控制（笛卡尔空间）。"""
        arr = (ctypes.c_double * len(c_vel))(*c_vel)
        return _lib.robot_speed_l(self._cptr_or_self(), arr, rotation_acc, trans_acc, t)

    # -------------------- 回零 --------------------

    def set_home(self, timeout: int = MACRO_CMD_TIMEOUT,
                  handle: int = MACRO_CMD_HANDLE) -> int:
        """设置机器人回零 - 回零位置由机器人的配置文件决定，采用编码器的机器人执行回零，机械臂不会移动，XYZ机械臂会移动。"""
        return _call_int(self._controller_id, _lib.robot_set_home, self._cptr_or_self(), timeout, handle)

    # -------------------- 门型运动 --------------------

    def move_jump(self, point, height: float) -> int:
        """
        门型运动，机器人运动一个门型轨迹，先运动到当前位置的Z轴指定高度，再移动到目标点相同Z轴高度，最后移动到目标点。

        @param point 目标点位（Location 或 LocationJ）
        @param height 目标偏移相对Z轴零点高度
        @return 0 表示成功，负值表示错误码
        """
        if isinstance(point, LocationJ):
            return _call_int(self._controller_id, _lib.robot_move_jump_locationj, self._cptr_or_self(),
                             ctypes.byref(point), height)
        return _call_int(self._controller_id, _lib.robot_move_jump_location, self._cptr_or_self(),
                         ctypes.byref(point), height)

    # -------------------- 紧急命令 --------------------

    def emg_cmd(self, cmd: int, ip: str = "",
                 timeout: int = MACRO_CMD_TIMEOUT,
                 handle: int = MACRO_CMD_HANDLE) -> int:
        """
        多机器人临时指令

        @param cmd 指令代码
        @param ip IP地址
        @param timeout 超时（ms）
        @param handle 指令句柄
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(self._controller_id, _lib.robot_emg_cmd, self._cptr_or_self(),
                         cmd, ip.encode("utf-8"), timeout, handle)
