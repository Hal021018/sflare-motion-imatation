"""
pallas_py

Created on: 2026-4-13
Author: 0590

RobotSDK Python ctypes 封装包。

使用方式（与原 pybind11 版本完全一致）：

    from pallas_py import Controller, Robot, PallasException
    from pallas_py import JogMode, FrameType, ZoneEnable
    from pallas_py import SharedJog, SharedToolFrame, SharedUserFrame
    from pallas_py import SharedZone, SharedIO, SharedGlobalVariable
    from pallas_py import SharedLatch, SharedModbusTCP, SharedCommon

    controller = Controller()
    controller.connect("192.168.1.10")

    robot = controller.add_robot(1)
    joint_pos = robot.get_joint_pos()
    robot.move_l(joint_pos)          # 支持 Location / LocationJ 重载

    controller.disconnect()

与 pybind11 版本的区别：
- 不需要系统安装特定版本的 Python 开发头文件
- 不需要安装 pybind11
- 只需预编译的 RobotSDK 共享库（.dll / .so / .dylib）
- 支持 Python 3.7+
"""

from .controller import Controller
from .robot import Robot
from .shared import (
    SharedJog,
    SharedToolFrame,
    SharedUserFrame,
    SharedZone,
    SharedIO,
    SharedGlobalVariable,
    SharedLatch,
    SharedModbusTCP,
    SharedCommon,
    SharedSolidifiedInfo,
)
from ._types import (
    Frame, Tool, UserFrame, Location, LocationJ,
    Profile, ForceModeParam,
    NodeItem, BrowseRespItem,
    LatchConfig, LatchResult,
    ZoneConfig, EMIFeedbackData,
    Trigger, IPCCommandPacket,
    QLVFile,
)
from ._enums import (
    ExceptionLevel, LogLevel, ThreadCmd, ThreadState, ThreadProperty,
    OperateMode, HiddenDataType, FileSystemCmd,
    FirmwareUpdateStep, ConfigType, ConfigUpdateStep, MTDUpdateStep,
    TriggerType, TriggerRefPoint, TriggerOutType,
    ComPort, ModbusDword, SystemErrorLevel, SystemErrorKey,
    LatchTargetType, LatchTriggerCfgType,
    ZoneEnable, ZoneType, ValidAreaType, CheckPart, EnableMethod, EulerType,
    JogMode, FrameType, GlobalType,
)
from ._utils import PallasException

__version__ = "2.0.0"

__all__ = [
    # 核心类
    "Controller",
    "Robot",
    "PallasException",

    # shared 静态工具类（对应 C++ namespace shared）
    "SharedJog",
    "SharedToolFrame",
    "SharedUserFrame",
    "SharedZone",
    "SharedIO",
    "SharedGlobalVariable",
    "SharedLatch",
    "SharedModbusTCP",
    "SharedCommon",
    "SharedSolidifiedInfo",

    # 类型
    "Frame",
    "Tool",
    "UserFrame",
    "Location",
    "LocationJ",
    "Profile",
    "ForceModeParam",
    "NodeItem",
    "BrowseRespItem",
    "LatchConfig",
    "LatchResult",
    "ZoneConfig",
    "EMIFeedbackData",
    "IPCCommandPacket",
    "Trigger",
    "QLVFile",

    # 枚举
    "ExceptionLevel",
    "LogLevel",
    "ThreadCmd",
    "ThreadState",
    "ThreadProperty",
    "OperateMode",
    "HiddenDataType",
    "FileSystemCmd",
    "FirmwareUpdateStep",
    "ConfigType",
    "ConfigUpdateStep",
    "MTDUpdateStep",
    "TriggerType",
    "TriggerRefPoint",
    "TriggerOutType",
    "ComPort",
    "ModbusDword",
    "SystemErrorLevel",
    "SystemErrorKey",
    "LatchTargetType",
    "LatchTriggerCfgType",
    "ZoneEnable",
    "ZoneType",
    "ValidAreaType",
    "CheckPart",
    "EnableMethod",
    "EulerType",
    "JogMode",
    "FrameType",
    "GlobalType",

    # 版本
    "__version__",
]
