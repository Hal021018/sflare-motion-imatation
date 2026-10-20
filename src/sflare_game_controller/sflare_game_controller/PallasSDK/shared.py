"""
shared.py

Created on: 2026-4-13
Author: 0590

shared 模块 ctypes 封装。
按照 C++ namespace shared 中各静态工具类结构组织：

    SharedJog          -> class SharedJog
    SharedToolFrame    -> class SharedToolFrame
    SharedUserFrame    -> class SharedUserFrame
    SharedZone         -> class SharedZone
    SharedIO           -> class SharedIO
    SharedGlobalVariable -> class SharedGlobalVariable
    SharedLatch        -> class SharedLatch
    SharedModbusTCP    -> class SharedModbusTCP
    SharedCommon       -> class SharedCommon

所有类均只提供 @staticmethod，与 C++ 侧保持一致(delete 构造函数)。
"""

import ctypes
from typing import List, Optional, Tuple, Dict

from ._utils import _call_int, _call_int_no_ctrl
from ._lib import _lib, _JogParams_C
from ._types import (
    Frame, Location, LocationJ, ZoneConfig,
    LatchConfig, LatchResult,
)

# ---------------------------------------------------------------------------
# SolidifiedInfo 相关 ctypes 结构体
# ---------------------------------------------------------------------------

class _SolidifiedInfoItem_C(ctypes.Structure):
    """对应 SolidifiedInfoItem_C"""
    pass  # 稍后定义字段

class _SolidifiedInfoModule_C(ctypes.Structure):
    """对应 SolidifiedInfoModule_C"""
    pass  # 稍后定义字段

# 定义字段(需要引用上面的结构体)
_SolidifiedInfoItem_C._fields_ = [
    ("getMethod", ctypes.c_char_p),
    ("address",   ctypes.c_char_p),
    ("space",     ctypes.c_char_p),
    ("idn",       ctypes.c_char_p),
    ("idnIndex",  ctypes.c_char_p),
    ("name",      ctypes.c_char_p),
    ("value",     ctypes.c_char_p),
]

_SolidifiedInfoModule_C._fields_ = [
    ("moduleType", ctypes.c_char_p),
    ("moduleName", ctypes.c_char_p),
    ("dataType",   ctypes.c_int),  # SolidifiedDataType_C
]

# ---------------------------------------------------------------------------
# 辅助 ctypes 结构体(仅本模块内部使用)
# ---------------------------------------------------------------------------

class _ToolFrameInfo_C(ctypes.Structure):
    """对应 ToolFrameInfo_C(ToolFrameExportC.h)"""
    _fields_ = [
        ("index",  ctypes.c_int),
        ("name",   ctypes.c_char_p),
        ("x",      ctypes.c_double),
        ("y",      ctypes.c_double),
        ("z",      ctypes.c_double),
        ("yaw",    ctypes.c_double),
        ("pitch",  ctypes.c_double),
        ("roll",   ctypes.c_double),
    ]


class _ActivedToolFrameRes_C(ctypes.Structure):
    """对应 ActivedToolFrameRes_C (ToolFrameExportC.h)"""
    _fields_ = [
        ("actived_index", ctypes.c_int),
        ("x",             ctypes.c_double),
        ("y",             ctypes.c_double),
        ("z",             ctypes.c_double),
        ("yaw",           ctypes.c_double),
        ("pitch",         ctypes.c_double),
        ("roll",          ctypes.c_double),
    ]


class _UserFrameInfo_C(ctypes.Structure):
    """对应 UserFrameInfo_C(UserFrameExportC.h)"""
    _fields_ = [
        ("index",  ctypes.c_int),
        ("name",   ctypes.c_char_p),
        ("x",      ctypes.c_double),
        ("y",      ctypes.c_double),
        ("z",      ctypes.c_double),
        ("yaw",    ctypes.c_double),
        ("pitch",  ctypes.c_double),
        ("roll",   ctypes.c_double),
    ]


class _ActivedUserFrameRes_C(ctypes.Structure):
    """对应 ActivedUserFrameRes_C(UserFrameExportC.h)"""
    _fields_ = [
        ("actived_index", ctypes.c_int),
        ("x",             ctypes.c_double),
        ("y",             ctypes.c_double),
        ("z",             ctypes.c_double),
        ("yaw",           ctypes.c_double),
        ("pitch",         ctypes.c_double),
        ("roll",          ctypes.c_double),
    ]


# FILE_PATH_LENGTH = 256(与 RobotTypesExportC.h 保持一致)
_FILE_PATH_LENGTH = 256

class _ZoneConfigWithNote_C(ctypes.Structure):
    """对应 ZoneConfigWithNote_C(ZoneExportC.h)"""
    _fields_ = [
        ("config", ZoneConfig),
        ("note",   ctypes.c_char * _FILE_PATH_LENGTH),
    ]


class _IONoteItem_C(ctypes.Structure):
    """对应 IONoteItem_C(SharedTypesExportC.h)"""
    _fields_ = [
        ("index", ctypes.c_int),
        ("note",  ctypes.c_char_p),
    ]


class _IOInfo_C(ctypes.Structure):
    """对应 IOInfo_C"""
    _fields_ = [
        ("index",   ctypes.c_int),
        ("io_type", ctypes.c_int),
        ("note",    ctypes.c_char_p),
    ]


class _IOGroup_C(ctypes.Structure):
    """对应 IOGroup_C"""
    _fields_ = [
        ("io_group",   ctypes.c_char_p),
        ("items",      ctypes.POINTER(_IOInfo_C)),
        ("item_count", ctypes.c_int),
    ]


class _GlobalVariableValue_C(ctypes.Structure):
    """对应 GlobalVariableValue_C"""
    _fields_ = [
        ("int_value",    ctypes.c_int),
        ("bool_value",   ctypes.c_bool),
        ("string_value", ctypes.c_char_p),
        ("double_value", ctypes.c_double),
    ]


class _GlobalVariableInfo_C(ctypes.Structure):
    """对应 GlobalVariableInfo_C"""
    _fields_ = [
        ("index",         ctypes.c_int),
        ("variable_type", ctypes.c_int),
        ("value",         _GlobalVariableValue_C),
        ("note",          ctypes.c_char_p),
        ("has_value",     ctypes.c_bool),
        ("has_note",      ctypes.c_bool),
    ]


class _GlobalVariableFavorite_C(ctypes.Structure):
    """对应 GlobalVariableFavorite_C"""
    _fields_ = [
        ("index",          ctypes.c_int),
        ("name",           ctypes.c_char_p),
        ("variables",      ctypes.POINTER(_GlobalVariableInfo_C)),
        ("variable_count", ctypes.c_int),
    ]


class _LatchConfigType_C(ctypes.Structure):
    """对应 LatchConfigType_C(含 latch_count 字段)"""
    _fields_ = [
        ("latch_id",          ctypes.c_int),
        ("enable",            ctypes.c_bool),
        ("target",            ctypes.c_int),
        ("target_type",       ctypes.c_int),
        ("target_robot_id",   ctypes.c_int),
        ("target_encoder_id", ctypes.c_int),
        ("target_site_id",    ctypes.c_int),
        ("src",               ctypes.c_int),
        ("trigger_cfg",       ctypes.c_int),
        ("delay_time",        ctypes.c_double),
        ("queue_size",        ctypes.c_int),
        ("alarm_threshold",   ctypes.c_int),
        ("conveyor_id",       ctypes.c_int),
        ("latch_count",       ctypes.c_int),
    ]


class _ModbusTCPParams_C(ctypes.Structure):
    """对应 ModbusTCPParams_C"""
    _fields_ = [
        ("scan_time",    ctypes.c_int),
        ("scan_timeout", ctypes.c_int),
        ("split_amount", ctypes.c_int),
        ("is_continuous", ctypes.c_bool),
    ]


class _ModbusTCPNode_C(ctypes.Structure):
    """对应 ModbusTCPNode_C"""
    _fields_ = [
        ("ip",        ctypes.c_char_p),
        ("port",      ctypes.c_int),
        ("di_offset", ctypes.c_int),
        ("do_offset", ctypes.c_int),
        ("di_amount", ctypes.c_int),
        ("do_amount", ctypes.c_int),
        ("enable",    ctypes.c_bool),
    ]


class _ModbusTCPIOItem_C(ctypes.Structure):
    """对应 ModbusTCPIOItem_C"""
    _fields_ = [
        ("io_start",   ctypes.c_int),
        ("io_end",     ctypes.c_int),
        ("addr_start", ctypes.c_int),
        ("addr_end",   ctypes.c_int),
    ]


class _ModbusTCPIOGroup_C(ctypes.Structure):
    """对应 ModbusTCPIOGroup_C"""
    _fields_ = [
        ("din_groups",      ctypes.POINTER(_ModbusTCPIOItem_C)),
        ("din_group_count", ctypes.c_int),
        ("dout_groups",     ctypes.POINTER(_ModbusTCPIOItem_C)),
        ("dout_group_count", ctypes.c_int),
    ]


class _ModbusTCPRes_C(ctypes.Structure):
    """对应 ModbusTCPRes_C"""
    _fields_ = [
        ("public_params",   _ModbusTCPParams_C),
        ("nodes",           ctypes.POINTER(_ModbusTCPNode_C)),
        ("node_count",      ctypes.c_int),
        ("io_groups",       ctypes.POINTER(_ModbusTCPIOGroup_C)),
        ("io_group_count",  ctypes.c_int),
    ]


class _ModbusTCPIOCheckState_C(ctypes.Structure):
    """对应 ModbusTCPIOCheckState_C"""
    _fields_ = [
        ("index",         ctypes.c_int),
        ("has_value",     ctypes.c_bool),
        ("value",         ctypes.c_bool),
        ("error_code",    ctypes.c_int),
        ("error_message", ctypes.c_char_p),
    ]


# ---------------------------------------------------------------------------
# 内部辅助：从 controller / robot 实例取 C 指针
# ---------------------------------------------------------------------------

def _ctrl_ptr(controller):
    return controller._cptr


def _robot_ptr(robot):
    return robot._cptr


# ---------------------------------------------------------------------------
# class SharedJog  (对应 C++ shared::SharedJog)
# ---------------------------------------------------------------------------

class SharedJog:
    """
    关节点动(Jog)控制。
    对应 C++: namespace shared { class SharedJog }
    所有方法均为 @staticmethod，与 C++ 侧一致。
    """

    @staticmethod
    def start(controller, robot,
              mode: int, frame_type: int,
              direction: str, speed: float,
              distance: float = 0.0,
              timestamp: int = 0) -> int:
        """
        开始 Jog 运动。

        @param controller  Controller 实例
        @param robot       Robot 实例
        @param mode        JogMode.Off / Continuous / Step
        @param frame_type  FrameType.Joint / World / Tool / UserFrame / Base
        @param direction   方向字符串(如 "x+"、"j1-")
        @param speed       速度(%)
        @param distance    步进距离(Step 模式使用，默认 0.0)
        @param timestamp   时间戳，连续 Jog 心跳(ms，默认 0)
        @return 0 表示成功，负值表示错误码
        """
        params = _JogParams_C()
        params.mode = mode
        params.frame_type = frame_type
        params.direction = direction.encode("utf-8")
        params.speed = speed
        params.distance = distance
        params.timestamp = timestamp
        return _call_int(_lib.jog_start,
                         _ctrl_ptr(controller),
                         _robot_ptr(robot),
                         ctypes.byref(params))

    @staticmethod
    def stop(controller, robot) -> int:
        """
        停止 Jog 运动。

        @param controller  Controller 实例
        @param robot       Robot 实例
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.jog_stop,
                         _ctrl_ptr(controller),
                         _robot_ptr(robot))

    @staticmethod
    def move_l(controller, robot, move_type: int, pos: Location, pos_j: LocationJ) -> int:
        """
        移动到指定位置（根据 move_type 选择笛卡尔或轴坐标）。

        @param controller  Controller 实例
        @param robot       Robot 实例
        @param move_type   移动类型：0=笛卡尔坐标，1=轴坐标
        @param pos         笛卡尔坐标（Location）
        @param pos_j       轴坐标（LocationJ）
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.jog_move_l,
                         _ctrl_ptr(controller),
                         _robot_ptr(robot),
                         move_type,
                         ctypes.byref(pos),
                         ctypes.byref(pos_j))

    def __init__(self):
        raise TypeError("SharedJog 不可实例化，请直接调用静态方法")


# ---------------------------------------------------------------------------
# class SharedToolFrame  (对应 C++ shared::SharedToolFrame)
# ---------------------------------------------------------------------------

class SharedToolFrame:
    """
    工具坐标系管理。
    对应 C++: namespace shared { class SharedToolFrame }
    """

    @staticmethod
    def get_list(controller) -> List[_ToolFrameInfo_C]:
        """
        获取工具坐标列表。

        @param controller  Controller 实例
        @return ToolFrameInfo_C 列表
        """
        frames_ptr = ctypes.POINTER(_ToolFrameInfo_C)()
        count = ctypes.c_int()
        _call_int(_lib.tool_frame_get_list,
                  _ctrl_ptr(controller),
                  ctypes.byref(frames_ptr),
                  ctypes.byref(count))
        n = count.value
        result = [frames_ptr[i] for i in range(n)]
        _lib.tool_frame_free_list(frames_ptr, n)
        return result

    @staticmethod
    def get_actived(controller, robot) -> _ActivedToolFrameRes_C:
        """
        获取当前激活的工具坐标。

        @param controller  Controller 实例
        @param robot       Robot 实例
        @return ActivedToolFrameRes_C
        """
        result = _ActivedToolFrameRes_C()
        _call_int(_lib.tool_frame_get_actived,
                  _ctrl_ptr(controller),
                  _robot_ptr(robot),
                  ctypes.byref(result))
        return result

    @staticmethod
    def update_frames(controller,
                      frames: List[_ToolFrameInfo_C],
                      save_to_file: bool = False) -> int:
        """
        更新工具坐标信息。

        @param controller   Controller 实例
        @param frames       ToolFrameInfo_C 列表
        @param save_to_file 是否保存到文件
        @return 0 表示成功，负值表示错误码
        """
        n = len(frames)
        arr = (_ToolFrameInfo_C * n)(*frames)
        return _call_int(_lib.tool_frame_update_frames,
                         _ctrl_ptr(controller),
                         arr, n, save_to_file)

    @staticmethod
    def set_tool(controller, robot, frame_index: int) -> int:
        """
        激活工具坐标(通过编号)。

        @param controller   Controller 实例
        @param robot        Robot 实例
        @param frame_index  工具坐标编号(0-based)
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.tool_frame_set_tool,
                         _ctrl_ptr(controller),
                         _robot_ptr(robot),
                         frame_index)

    @staticmethod
    def calibrate_two_point(controller,
                             p1: Location,
                             p2: Location) -> _ToolFrameInfo_C:
        """
        工具坐标两点标定。

        @param controller  Controller 实例
        @param p1          第一个点(Location)
        @param p2          第二个点(Location)
        @return 标定结果 ToolFrameInfo_C
        """
        result = _ToolFrameInfo_C()
        _call_int(_lib.tool_frame_calibrate_two_point,
                  _ctrl_ptr(controller),
                  ctypes.byref(p1),
                  ctypes.byref(p2),
                  ctypes.byref(result))
        return result

    @staticmethod
    def calibrate_four_point(controller,
                              p1: Location, p2: Location,
                              p3: Location, p4: Location) -> _ToolFrameInfo_C:
        """
        工具坐标四点标定。

        @param controller  Controller 实例
        @param p1 ~ p4     四个标定点(Location)
        @return 标定结果 ToolFrameInfo_C
        """
        result = _ToolFrameInfo_C()
        _call_int(_lib.tool_frame_calibrate_four_point,
                  _ctrl_ptr(controller),
                  ctypes.byref(p1),
                  ctypes.byref(p2),
                  ctypes.byref(p3),
                  ctypes.byref(p4),
                  ctypes.byref(result))
        return result

    def __init__(self):
        raise TypeError("SharedToolFrame 不可实例化，请直接调用静态方法")


# ---------------------------------------------------------------------------
# class SharedUserFrame  (对应 C++ shared::SharedUserFrame)
# ---------------------------------------------------------------------------

class SharedUserFrame:
    """
    用户坐标系管理。
    对应 C++: namespace shared { class SharedUserFrame }
    """

    @staticmethod
    def get_list(controller) -> List[_UserFrameInfo_C]:
        """
        获取用户坐标列表。

        @param controller  Controller 实例
        @return UserFrameInfo_C 列表
        """
        frames_ptr = ctypes.POINTER(_UserFrameInfo_C)()
        count = ctypes.c_int()
        _call_int(_lib.user_frame_get_list,
                  _ctrl_ptr(controller),
                  ctypes.byref(frames_ptr),
                  ctypes.byref(count))
        n = count.value
        result = [frames_ptr[i] for i in range(n)]
        _lib.user_frame_free_list(frames_ptr, n)
        return result

    @staticmethod
    def get_actived(controller, robot) -> _ActivedUserFrameRes_C:
        """
        获取当前激活的用户坐标。

        @param controller  Controller 实例
        @param robot       Robot 实例
        @return ActivedUserFrameRes_C
        """
        result = _ActivedUserFrameRes_C()
        _call_int(_lib.user_frame_get_actived,
                  _ctrl_ptr(controller),
                  _robot_ptr(robot),
                  ctypes.byref(result))
        return result

    @staticmethod
    def update_frames(controller,
                      frames: List[_UserFrameInfo_C],
                      save_to_file: bool = False) -> int:
        """
        更新用户坐标信息。

        @param controller   Controller 实例
        @param frames       UserFrameInfo_C 列表
        @param save_to_file 是否保存到文件
        @return 0 表示成功，负值表示错误码
        """
        n = len(frames)
        arr = (_UserFrameInfo_C * n)(*frames)
        return _call_int(_lib.user_frame_update_frames,
                         _ctrl_ptr(controller),
                         arr, n, save_to_file)

    @staticmethod
    def set_userframe(controller, robot, frame_index: int) -> int:
        """
        激活用户坐标(通过编号)。

        @param controller   Controller 实例
        @param robot        Robot 实例
        @param frame_index  用户坐标编号(0-based)
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.user_frame_set_userframe,
                         _ctrl_ptr(controller),
                         _robot_ptr(robot),
                         frame_index)

    @staticmethod
    def calibrate(controller,
                  p0: Location,
                  px: Location,
                  py: Location) -> _UserFrameInfo_C:
        """
        用户坐标系三点标定。

        @param controller  Controller 实例
        @param p0          原点
        @param px          X 轴方向点
        @param py          Y 轴方向点
        @return 标定结果 UserFrameInfo_C
        """
        result = _UserFrameInfo_C()
        _call_int(_lib.user_frame_calibrate,
                  _ctrl_ptr(controller),
                  ctypes.byref(p0),
                  ctypes.byref(px),
                  ctypes.byref(py),
                  ctypes.byref(result))
        return result

    def __init__(self):
        raise TypeError("SharedUserFrame 不可实例化，请直接调用静态方法")


# ---------------------------------------------------------------------------
# class SharedZone  (对应 C++ shared::SharedZone)
# ---------------------------------------------------------------------------

class SharedZone:
    """
    干涉区(Zone)管理。
    对应 C++: namespace shared { class SharedZone }
    """

    @staticmethod
    def get_zone_list(controller) -> List[Tuple[ZoneConfig, str]]:
        """
        获取干涉区列表。

        @param controller  Controller 实例
        @return (ZoneConfig, note) 元组列表
        """
        result_ptr = ctypes.POINTER(_ZoneConfigWithNote_C)()
        count = ctypes.c_int()
        _call_int(_lib.zone_get_zone_list,
                  _ctrl_ptr(controller),
                  ctypes.byref(result_ptr),
                  ctypes.byref(count))
        n = count.value
        zones = [(result_ptr[i].config,
                  result_ptr[i].note.decode("utf-8", errors="replace"))
                 for i in range(n)]
        _lib.zone_free_zone_list(result_ptr, n)
        return zones

    @staticmethod
    def get_zone_config(controller, zone_id: int) -> Tuple[ZoneConfig, str]:
        """
        获取指定干涉区配置。

        @param controller  Controller 实例
        @param zone_id     干涉区 ID
        @return (ZoneConfig, note)
        """
        result = _ZoneConfigWithNote_C()
        _call_int(_lib.zone_get_zone_config,
                  _ctrl_ptr(controller),
                  zone_id,
                  ctypes.byref(result))
        return result.config, result.note.decode("utf-8", errors="replace")

    @staticmethod
    def set_zone_config(controller,
                        zone_config: ZoneConfig,
                        zone_note: str = "",
                        save_to_file: bool = False) -> int:
        """
        更新指定干涉区配置。

        @param controller   Controller 实例
        @param zone_config  ZoneConfig 实例
        @param zone_note    备注(最多 255 字符)
        @param save_to_file 是否持久化到文件
        @return 0 表示成功，负值表示错误码
        """
        item = _ZoneConfigWithNote_C()
        item.config = zone_config
        note_bytes = zone_note.encode("utf-8")[:_FILE_PATH_LENGTH - 1]
        ctypes.memmove(item.note, note_bytes, len(note_bytes))
        return _call_int(_lib.zone_set_zone_config,
                         _ctrl_ptr(controller),
                         ctypes.byref(item),
                         save_to_file)

    def __init__(self):
        raise TypeError("SharedZone 不可实例化，请直接调用静态方法")


# ---------------------------------------------------------------------------
# class SharedIO  (对应 C++ shared::SharedIO)
# ---------------------------------------------------------------------------

class SharedIO:
    """
    I/O 管理。
    对应 C++: namespace shared { class SharedIO }
    """

    @staticmethod
    def get_io_list(controller,
                    io_types: Optional[List[int]] = None,
                    with_notes: bool = False) -> List[_IOGroup_C]:
        """
        获取 I/O 列表。

        @param controller  Controller 实例
        @param io_types    需要获取的 IOType_C 列表(None 表示所有类型)
        @param with_notes  是否填充备注信息
        @return IOGroup_C 列表
        """
        if io_types:
            types_arr = (ctypes.c_int * len(io_types))(*io_types)
            types_ptr = ctypes.cast(types_arr, ctypes.POINTER(ctypes.c_int))
            type_count = len(io_types)
        else:
            types_ptr = ctypes.POINTER(ctypes.c_int)()
            type_count = 0

        result_ptr = ctypes.POINTER(_IOGroup_C)()
        count = ctypes.c_int()
        _call_int(_lib.io_get_io_list,
                  _ctrl_ptr(controller),
                  types_ptr, type_count, with_notes,
                  ctypes.byref(result_ptr),
                  ctypes.byref(count))
        n = count.value
        groups = [result_ptr[i] for i in range(n)]
        _lib.io_free_io_list(result_ptr, n)
        return groups

    @staticmethod
    def get_io_groups(controller) -> List[_IOGroup_C]:
        """
        获取 I/O 分组信息。

        @param controller  Controller 实例
        @return IOGroup_C 列表
        """
        result_ptr = ctypes.POINTER(_IOGroup_C)()
        count = ctypes.c_int()
        _call_int(_lib.io_get_io_groups,
                  _ctrl_ptr(controller),
                  ctypes.byref(result_ptr),
                  ctypes.byref(count))
        n = count.value
        groups = [result_ptr[i] for i in range(n)]
        _lib.io_free_io_groups(result_ptr, n)
        return groups

    @staticmethod
    def update_io_groups(controller, groups: List[_IOGroup_C]) -> int:
        """
        更新 I/O 分组信息。

        @param controller  Controller 实例
        @param groups      IOGroup_C 列表
        @return 0 表示成功，负值表示错误码
        """
        n = len(groups)
        arr = (_IOGroup_C * n)(*groups)
        return _call_int(_lib.io_update_io_groups,
                         _ctrl_ptr(controller),
                         arr, n)

    @staticmethod
    def get_io_value(controller, index: int) -> bool:
        """
        获取 I/O 当前值。

        @param controller  Controller 实例
        @param index       I/O 索引
        @return 当前值(True/False)
        """
        result = ctypes.c_bool()
        _call_int(_lib.io_get_io_value,
                  _ctrl_ptr(controller),
                  index,
                  ctypes.byref(result))
        return result.value

    @staticmethod
    def set_io_value(controller, index: int, value: bool,
                     has_force: bool = False, force: bool = False) -> int:
        """
        设置 I/O 值。

        @param controller  Controller 实例
        @param index       I/O 索引
        @param value       目标值(True/False)
        @param has_force   是否指定强制参数
        @param force       强制开关(has_force=True 时生效)
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.io_set_io_value,
                         _ctrl_ptr(controller),
                         index, value, has_force, force)

    @staticmethod
    def get_io_force(controller, index: int) -> bool:
        """
        获取 I/O 强制状态。

        @param controller  Controller 实例
        @param index       I/O 索引
        @return True 表示强制，False 表示未强制
        """
        result = ctypes.c_bool()
        _call_int(_lib.io_get_io_force,
                  _ctrl_ptr(controller),
                  index,
                  ctypes.byref(result))
        return result.value

    @staticmethod
    def set_io_force(controller, index: int, value: bool) -> int:
        """
        设置 I/O 强制状态。

        @param controller  Controller 实例
        @param index       I/O 索引
        @param value       True=根据当前电平设为强制，False=取消强制
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.io_set_io_force,
                         _ctrl_ptr(controller),
                         index, value)

    @staticmethod
    def get_io_notes(controller) -> List[_IONoteItem_C]:
        """
        获取所有 I/O 备注。

        @param controller  Controller 实例
        @return IONoteItem_C 列表
        """
        result_ptr = ctypes.POINTER(_IONoteItem_C)()
        count = ctypes.c_int()
        _call_int(_lib.io_get_io_notes,
                  _ctrl_ptr(controller),
                  ctypes.byref(result_ptr),
                  ctypes.byref(count))
        n = count.value
        items = [result_ptr[i] for i in range(n)]
        _lib.io_free_io_notes(result_ptr, n)
        return items

    @staticmethod
    def update_io_notes(controller, items: List[_IONoteItem_C]) -> int:
        """
        更新 I/O 备注。

        @param controller  Controller 实例
        @param items       IONoteItem_C 列表
        @return 0 表示成功，负值表示错误码
        """
        n = len(items)
        arr = (_IONoteItem_C * n)(*items)
        return _call_int(_lib.io_update_io_notes,
                         _ctrl_ptr(controller),
                         arr, n)

    def __init__(self):
        raise TypeError("SharedIO 不可实例化，请直接调用静态方法")


# ---------------------------------------------------------------------------
# class SharedGlobalVariable  (对应 C++ shared::SharedGlobalVariable)
# ---------------------------------------------------------------------------

class SharedGlobalVariable:
    """
    全局变量管理。
    对应 C++: namespace shared { class SharedGlobalVariable }
    """

    @staticmethod
    def get_global_variable_list(controller,
                                  variable_type: int,
                                  page_number: int) -> List[_GlobalVariableInfo_C]:
        """
        获取全局变量列表。

        @param controller    Controller 实例
        @param variable_type GlobalVariableType_C 枚举值
        @param page_number   页码(从 1 开始)
        @return GlobalVariableInfo_C 列表
        """
        result_ptr = ctypes.POINTER(_GlobalVariableInfo_C)()
        count = ctypes.c_int()
        _call_int(_lib.global_variable_get_global_variable_list,
                  _ctrl_ptr(controller),
                  variable_type, page_number,
                  ctypes.byref(result_ptr),
                  ctypes.byref(count))
        n = count.value
        items = [result_ptr[i] for i in range(n)]
        _lib.global_variable_free_global_variable_list(result_ptr, n)
        return items

    @staticmethod
    def get_global_variable(controller,
                             variable_type: int,
                             index: int) -> _GlobalVariableInfo_C:
        """
        获取单个全局变量。

        @param controller    Controller 实例
        @param variable_type GlobalVariableType_C 枚举值
        @param index         变量索引
        @return GlobalVariableInfo_C
        """
        result = _GlobalVariableInfo_C()
        _call_int(_lib.global_variable_get_global_variable,
                  _ctrl_ptr(controller),
                  variable_type, index,
                  ctypes.byref(result))
        return result

    @staticmethod
    def update_global_variables(controller,
                                 variables: List[_GlobalVariableInfo_C],
                                 save_to_file: bool = False) -> int:
        """
        更新全局变量(包括备注和值)。

        @param controller   Controller 实例
        @param variables    GlobalVariableInfo_C 列表
        @param save_to_file 是否保存到文件
        @return 0 表示成功，负值表示错误码
        """
        n = len(variables)
        arr = (_GlobalVariableInfo_C * n)(*variables)
        return _call_int(_lib.global_variable_update_global_variables,
                         _ctrl_ptr(controller),
                         arr, n, save_to_file)

    @staticmethod
    def get_favorite_info_from_idn(controller,
                                    favorite_index: int) -> _GlobalVariableFavorite_C:
        """
        从 IDN 获取收藏夹信息。

        @param controller     Controller 实例
        @param favorite_index 收藏夹索引(范围 [1, 16])
        @return GlobalVariableFavorite_C
        """
        result = _GlobalVariableFavorite_C()
        _call_int(_lib.global_variable_get_favorite_info_from_idn,
                  _ctrl_ptr(controller),
                  favorite_index,
                  ctypes.byref(result))
        return result

    @staticmethod
    def update_favorite_info_to_idn(controller,
                                     favorite: _GlobalVariableFavorite_C) -> int:
        """
        往 IDN 写入收藏夹信息。

        @param controller Controller 实例
        @param favorite   GlobalVariableFavorite_C
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.global_variable_update_favorite_info_to_idn,
                         _ctrl_ptr(controller),
                         ctypes.byref(favorite))

    @staticmethod
    def get_favorite_list(controller) -> List[_GlobalVariableFavorite_C]:
        """
        获取全局变量收藏夹列表。

        @param controller  Controller 实例
        @return GlobalVariableFavorite_C 列表
        """
        result_ptr = ctypes.POINTER(_GlobalVariableFavorite_C)()
        count = ctypes.c_int()
        _call_int(_lib.global_variable_get_favorite_list,
                  _ctrl_ptr(controller),
                  ctypes.byref(result_ptr),
                  ctypes.byref(count))
        n = count.value
        items = [result_ptr[i] for i in range(n)]
        _lib.global_variable_free_favorite_list(result_ptr, n)
        return items

    @staticmethod
    def rename_favorite(controller,
                         favorite_index: int,
                         favorite_name: str,
                         save_to_file: bool = False) -> int:
        """
        重命名收藏夹。

        @param controller     Controller 实例
        @param favorite_index 收藏夹索引
        @param favorite_name  新的收藏夹名称
        @param save_to_file   是否保存到文件
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.global_variable_rename_favorite,
                         _ctrl_ptr(controller),
                         favorite_index,
                         favorite_name.encode("utf-8"),
                         save_to_file)

    @staticmethod
    def update_favorite(controller,
                         favorite_index: int,
                         favorite_name: str,
                         variables: List[_GlobalVariableInfo_C],
                         save_to_file: bool = False) -> int:
        """
        更新收藏夹内容(不更新变量值及备注)。

        @param controller     Controller 实例
        @param favorite_index 收藏夹索引
        @param favorite_name  收藏夹名称
        @param variables      变量列表(最多 16 个)
        @param save_to_file   是否保存到文件
        @return 0 表示成功，负值表示错误码
        """
        n = len(variables)
        arr = (_GlobalVariableInfo_C * n)(*variables)
        return _call_int(_lib.global_variable_update_favorite,
                         _ctrl_ptr(controller),
                         favorite_index,
                         favorite_name.encode("utf-8"),
                         arr, n, save_to_file)

    def __init__(self):
        raise TypeError("SharedGlobalVariable 不可实例化，请直接调用静态方法")


# ---------------------------------------------------------------------------
# class SharedLatch  (对应 C++ shared::SharedLatch)
# ---------------------------------------------------------------------------

class SharedLatch:
    """
    锁存器(Latch)管理。
    对应 C++: namespace shared { class SharedLatch }
    """

    @staticmethod
    def get_latch_list(controller) -> List[_LatchConfigType_C]:
        """
        获取 Latch 列表。

        @param controller  Controller 实例
        @return LatchConfigType_C 列表(含 latch_count)
        """
        result_ptr = ctypes.POINTER(_LatchConfigType_C)()
        count = ctypes.c_int()
        _call_int(_lib.latch_get_list,
                  _ctrl_ptr(controller),
                  ctypes.byref(result_ptr),
                  ctypes.byref(count))
        n = count.value
        return [result_ptr[i] for i in range(n)]

    @staticmethod
    def get_latch_config(controller, latch_id: int) -> _LatchConfigType_C:
        """
        获取单个 Latch 配置。

        @param controller  Controller 实例
        @param latch_id    Latch ID
        @return LatchConfigType_C
        """
        result = _LatchConfigType_C()
        _call_int(_lib.latch_get_config,
                  _ctrl_ptr(controller),
                  latch_id,
                  ctypes.byref(result))
        return result

    @staticmethod
    def set_latch_config(controller,
                          latch_info: LatchConfig,
                          save_to_file: bool = False) -> int:
        """
        更新单个 Latch 配置。

        @param controller   Controller 实例
        @param latch_info   LatchConfig 实例(来自 _types.py)
        @param save_to_file 是否保存到文件
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.latch_set_config,
                         _ctrl_ptr(controller),
                         ctypes.byref(latch_info),
                         save_to_file)

    @staticmethod
    def get_latch_count(controller, latch_id: int) -> int:
        """
        获取 Latch 触发计数。

        @param controller  Controller 实例
        @param latch_id    Latch ID
        @return 触发次数
        """
        result = ctypes.c_int()
        _call_int(_lib.latch_get_count,
                  _ctrl_ptr(controller),
                  latch_id,
                  ctypes.byref(result))
        return result.value

    @staticmethod
    def get_latch_result(controller,
                          latch_id: int,
                          index: int,
                          mode: int,
                          is_del: bool) -> LatchResult:
        """
        获取单个 Latch 触发结果。

        @param controller  Controller 实例
        @param latch_id    Latch ID
        @param index       获取第几个结果(1 表示最新)
        @param mode        LatchResultMode_C(0=Cartesian, 1=Joint, 2=Encoder)
        @param is_del      是否删除该结果
        @return LatchResult(来自 _types.py)
        """
        result = LatchResult()
        _call_int(_lib.latch_get_result,
                  _ctrl_ptr(controller),
                  latch_id, index, mode, is_del,
                  ctypes.byref(result))
        return result

    @staticmethod
    def get_latch_result_list(controller,
                               latch_id: int,
                               length: int,
                               mode: int,
                               is_del: bool) -> List[LatchResult]:
        """
        获取 Latch 触发结果列表。

        @param controller  Controller 实例
        @param latch_id    Latch ID
        @param length      获取结果数量
        @param mode        LatchResultMode_C
        @param is_del      是否删除
        @return LatchResult 列表
        """
        result_ptr = ctypes.POINTER(LatchResult)()
        count = ctypes.c_int()
        _call_int(_lib.latch_get_result_list,
                  _ctrl_ptr(controller),
                  latch_id, length, mode, is_del,
                  ctypes.byref(result_ptr),
                  ctypes.byref(count))
        n = count.value
        return [result_ptr[i] for i in range(n)]

    @staticmethod
    def set_latch_flush(controller, latch_id: int) -> int:
        """
        清空 Latch 队列。

        @param controller  Controller 实例
        @param latch_id    Latch ID
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.latch_set_flush,
                         _ctrl_ptr(controller),
                         latch_id)

    @staticmethod
    def set_latch_delay_time(controller,
                              latch_id: int,
                              delay_time: float) -> int:
        """
        设置 Latch 延迟时间。

        @param controller  Controller 实例
        @param latch_id    Latch ID
        @param delay_time  延迟时间(单位：秒)
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.latch_set_delay_time,
                         _ctrl_ptr(controller),
                         latch_id,
                         ctypes.c_double(delay_time))

    def __init__(self):
        raise TypeError("SharedLatch 不可实例化，请直接调用静态方法")


# ---------------------------------------------------------------------------
# class SharedModbusTCP  (对应 C++ shared::SharedModbusTCP)
# ---------------------------------------------------------------------------

class SharedModbusTCP:
    """
    Modbus TCP 配置管理。
    对应 C++: namespace shared { class SharedModbusTCP }
    """

    @staticmethod
    def get_modbus_tcp_settings(controller) -> _ModbusTCPRes_C:
        """
        获取 Modbus TCP 配置。

        @param controller  Controller 实例
        @return ModbusTCPRes_C
        """
        result = _ModbusTCPRes_C()
        _call_int(_lib.modbus_tcp_get_settings,
                  _ctrl_ptr(controller),
                  ctypes.byref(result))
        return result

    @staticmethod
    def update_modbus_tcp_settings(controller,
                                    public_params: _ModbusTCPParams_C,
                                    nodes: List[_ModbusTCPNode_C]) -> int:
        """
        更新 Modbus TCP 配置。

        @param controller   Controller 实例
        @param public_params 公共参数
        @param nodes         从站节点列表
        @return 0 表示成功，负值表示错误码
        """
        n = len(nodes)
        arr = (_ModbusTCPNode_C * n)(*nodes) if n > 0 else (_ModbusTCPNode_C * 1)()
        return _call_int(_lib.modbus_tcp_update_settings,
                         _ctrl_ptr(controller),
                         ctypes.byref(public_params),
                         1,
                         arr,
                         n)

    @staticmethod
    def extend_modbus_slave_amount(controller, target_amount: int) -> int:
        """
        扩展 Modbus TCP 从站数到目标数量。

        @param controller    Controller 实例
        @param target_amount 目标从站数量
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.modbus_tcp_extend_slave_amount,
                         _ctrl_ptr(controller),
                         target_amount)

    def __init__(self):
        raise TypeError("SharedModbusTCP 不可实例化，请直接调用静态方法")


# ---------------------------------------------------------------------------
# class SharedCommon  (对应 C++ shared::SharedCommon)
# ---------------------------------------------------------------------------

class SharedCommon:
    """
    通用工具函数。
    对应 C++: namespace shared { class SharedCommon }
    """

    @staticmethod
    def save_application_idn_to_file(controller) -> int:
        """
        保存应用层 IDN 到文件。

        @param controller  Controller 实例
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.common_save_application_idn_to_file,
                         _ctrl_ptr(controller))

    @staticmethod
    def save_pallas_idn_to_file(controller) -> int:
        """
        保存系统层 IDN 到文件。

        @param controller  Controller 实例
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.common_save_pallas_idn_to_file,
                         _ctrl_ptr(controller))

    @staticmethod
    def save_all_idn_to_file(controller) -> int:
        """
        同时保存应用层与系统层 IDN 到文件。

        @param controller  Controller 实例
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(_lib.common_save_all_idn_to_file,
                         _ctrl_ptr(controller))

    def __init__(self):
        raise TypeError("SharedCommon 不可实例化，请直接调用静态方法")


# ---------------------------------------------------------------------------
# class SharedSolidifiedInfo  (对应 C++ shared::SharedSolidifiedInfo)
# ---------------------------------------------------------------------------

class SharedSolidifiedInfo:
    """
    固化信息管理。
    对应 C++: namespace shared { class SharedSolidifiedInfo }
    """

    @staticmethod
    def initialize(xml_file_path: str = "Resources/SolidifiedInfo/SolidifiedInfo.xml") -> int:
        """
        初始化固化信息模块(从 XML 文件加载)。

        @param xml_file_path  XML 文件路径，默认为 "Resources/SolidifiedInfo/SolidifiedInfo.xml"
        @return 0 表示成功，负值表示错误码
        """
        return _call_int_no_ctrl(_lib.solidified_info_initialize,
                                 xml_file_path.encode("utf-8"))

    @staticmethod
    def is_initialized() -> bool:
        """
        检查是否已初始化。

        @return True 表示已初始化，False 表示未初始化
        """
        result = _call_int_no_ctrl(_lib.solidified_info_is_initialized)
        return result == 1

    @staticmethod
    def get_modules() -> List[_SolidifiedInfoModule_C]:
        """
        获取所有模块元信息列表。

        @return SolidifiedInfoModule_C 列表
        """
        modules_ptr = ctypes.c_void_p()
        count = ctypes.c_int()
        _call_int_no_ctrl(_lib.solidified_info_get_modules,
                          ctypes.byref(modules_ptr),
                          ctypes.byref(count))

        n = count.value
        if n == 0 or not modules_ptr.value:
            return []

        # 将 void* 转换为 SolidifiedInfoModule_C 指针
        module_array = ctypes.cast(modules_ptr.value,
                                   ctypes.POINTER(_SolidifiedInfoModule_C))

        # 在释放前读取并保存字符串数据
        result = []

        for i in range(n):
            # 先读取所有字段值
            module_type_bytes = module_array[i].moduleType
            module_name_bytes = module_array[i].moduleName
            data_type = module_array[i].dataType

            # 创建新的结构体实例并赋值
            item = _SolidifiedInfoModule_C()
            if module_type_bytes:
                # c_char_p 可以直接赋值 bytes
                item.moduleType = module_type_bytes
            else:
                item.moduleType = None
            if module_name_bytes:
                item.moduleName = module_name_bytes
            else:
                item.moduleName = None
            item.dataType = data_type
            result.append(item)

        # 释放 C 层内存
        _lib.solidified_info_free_modules(modules_ptr.value, n)
        return result


    @staticmethod
    def get_solidified_info(controller, moduleType: str, index: int = 0,
                           ctrl_ip: str = "") -> List[_SolidifiedInfoItem_C]:
        """
        根据模块类型获取特定模块的固化信息项列表。

        @param controller   Controller 实例
        @param moduleType   模块类型
        @param index        模块索引(用于数组类型，默认 0)
        @param ctrl_ip      控制器 IP 地址(可选)
        @return SolidifiedInfoItem_C 列表
        """
        items_ptr = ctypes.c_void_p()
        count = ctypes.c_int()
        # 确保 moduleType 是 bytes 类型
        if isinstance(moduleType, str):
            module_type_bytes = moduleType.encode("utf-8")
        else:
            module_type_bytes = moduleType

        _call_int(controller._controller_id,
                  _lib.solidified_info_get_solidified_info,
                  _ctrl_ptr(controller),
                  module_type_bytes,
                  ctypes.c_int(index),
                  ctrl_ip.encode("utf-8") if ctrl_ip else b"",
                  ctypes.byref(items_ptr),
                  ctypes.byref(count))

        n = count.value
        if n == 0 or not items_ptr.value:
            return []

        # 将 void* 转换为 SolidifiedInfoItem_C 指针
        item_array = ctypes.cast(items_ptr.value,
                                 ctypes.POINTER(_SolidifiedInfoItem_C))

        # 在释放前读取并保存字符串数据
        result = []
        for i in range(n):
            item = _SolidifiedInfoItem_C()
            # 直接复制字段值，不需要 create_string_buffer
            for field_name, field_type in item._fields_:
                ptr = getattr(item_array[i], field_name)
                if ptr:
                    try:
                        # c_char_p 类型的值需要通过 .value 获取
                        if hasattr(ptr, 'value'):
                            content_bytes = ptr.value if ptr.value else b""
                        elif isinstance(ptr, (bytes, bytearray)):
                            # bytes 或 bytearray 类型，直接使用
                            content_bytes = bytes(ptr)
                        else:
                            # 尝试作为指针处理
                            content_bytes = ctypes.string_at(ptr)

                        # 直接赋值 bytes，ctypes 会自动处理
                        setattr(item, field_name, content_bytes if content_bytes else None)
                    except Exception as e:
                        print(f"[DEBUG] 处理字段 {field_name} 时出错: {e}, ptr={ptr!r}, type={type(ptr)}")
                        setattr(item, field_name, None)
                else:
                    setattr(item, field_name, None)
            result.append(item)

        # 释放 C 层内存
        _lib.solidified_info_free_items(items_ptr.value, n)
        return result

    @staticmethod
    def export_info(controller, file_path: str, ctrl_ip: str = "") -> int:
        """
        导出固化信息到文件。

        @param controller   Controller 实例
        @param file_path    导出文件路径
        @param ctrl_ip      控制器 IP 地址(可选)
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(controller._controller_id,
                        _lib.solidified_info_export_info,
                        _ctrl_ptr(controller),
                        file_path.encode("utf-8"),
                        ctrl_ip.encode("utf-8") if ctrl_ip else b"")

    @staticmethod
    def import_info(controller, file_path: str, ctrl_ip: str = "") -> int:
        """
        从文件导入固化信息。

        @param controller   Controller 实例
        @param file_path    导入文件路径
        @param ctrl_ip      控制器 IP 地址(可选)
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(controller._controller_id,
                        _lib.solidified_info_import_info,
                        _ctrl_ptr(controller),
                        file_path.encode("utf-8"),
                        ctrl_ip.encode("utf-8") if ctrl_ip else b"")

    @staticmethod
    def backup(controller, ctrl_ip: str = "") -> int:
        """
        备份同步固化信息。

        @param controller   Controller 实例
        @param ctrl_ip      控制器 IP 地址(可选)
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(controller._controller_id,
                        _lib.solidified_info_backup,
                        _ctrl_ptr(controller),
                        ctrl_ip.encode("utf-8") if ctrl_ip else b"")

    @staticmethod
    def recover(controller, node_id: int, ctrl_ip: str = "") -> int:
        """
        还原固化信息。

        @param controller   Controller 实例
        @param node_id      节点ID
        @param ctrl_ip      控制器 IP 地址(可选)
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(controller._controller_id,
                        _lib.solidified_info_recover,
                        _ctrl_ptr(controller),
                        node_id,
                        ctrl_ip.encode("utf-8") if ctrl_ip else b"")

    @staticmethod
    def write(controller, address: str, space: int, value: str,
              ctrl_ip: str = "") -> int:
        """
        写入固化信息。

        @param controller   Controller 实例
        @param address      地址
        @param space        空间大小(字节)
        @param value        值
        @param ctrl_ip      控制器 IP 地址(可选)
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(controller._controller_id,
                        _lib.solidified_info_write,
                        _ctrl_ptr(controller),
                        address.encode("utf-8"),
                        space,
                        value.encode("utf-8"),
                        ctrl_ip.encode("utf-8") if ctrl_ip else b"")

    @staticmethod
    def read(controller, address: str, space: int,
             ctrl_ip: str = "") -> str:
        """
        读取固化信息。

        @param controller   Controller 实例
        @param address      地址
        @param space        空间大小(字节)
        @param ctrl_ip      控制器 IP 地址(可选)
        @return 读取的值
        """
        value_ptr = ctypes.c_char_p()
        _call_int(controller._controller_id,
                 _lib.solidified_info_read,
                 _ctrl_ptr(controller),
                 address.encode("utf-8"),
                 space,
                 ctrl_ip.encode("utf-8") if ctrl_ip else b"",
                 ctypes.byref(value_ptr))

        result = value_ptr.value.decode("utf-8") if value_ptr.value else ""
        _lib.solidified_info_free_string(value_ptr.value)
        return result

    @staticmethod
    def flush(controller, ctrl_ip: str = "") -> int:
        """
        烧录刷新固化信息到控制器。

        @param controller   Controller 实例
        @param ctrl_ip      控制器 IP 地址(可选)
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(controller._controller_id,
                        _lib.solidified_info_flush,
                        _ctrl_ptr(controller),
                        ctrl_ip.encode("utf-8") if ctrl_ip else b"")

    def __init__(self):
        raise TypeError("SharedSolidifiedInfo 不可实例化，请直接调用静态方法")
