"""
_lib.py

Created on: 2026-4-13
Author: 0590

动态加载 RobotSDK C 共享库，注册所有 C API 函数签名。
Python ctypes 封装层底层模块。
"""

import os
import sys
import ctypes
from pathlib import Path

from ._types import (  # noqa: F401, F403
    Frame, Tool, UserFrame, Location, LocationJ, Profile,
    EMIFeedbackData, QLVFile, ForceModeParam, BrowseRespItem,
    SolutionInfo, ProjectInfo, IPCCommandPacket,
)

# ---------------------------------------------------------------------------
# 库文件查找
# ---------------------------------------------------------------------------

def _find_library():
    """
    在以下位置查找 RobotSDK 共享库：
    1. PALLAS_SDK_LIB 环境变量指定的路径
    2. python_ctypes 包目录自身（推荐放置位置）
    3. python_ctypes 父目录的 build/ 子目录
    4. 系统 PATH
    """
    env_path = os.environ.get("PALLAS_SDK_LIB")
    if env_path:
        p = Path(env_path)
        if p.is_file():
            return str(p)
        if p.is_dir():
            for lib in p.glob("*RobotSDK*"):
                return str(lib)

    # 尝试 python_ctypes/ 自身目录（方便与用户脚本放一起）
    pallas_py_dir = Path(__file__).parent.resolve()
    if sys.platform == "win32":
        for lib in pallas_py_dir.glob("*.dll"):
            return str(lib)
        for lib in pallas_py_dir.glob("*.lib"):
            return str(lib)
    elif sys.platform == "linux":
        for lib in pallas_py_dir.glob("*.so"):
            return str(lib)
    elif sys.platform == "darwin":
        for lib in pallas_py_dir.glob("*.dylib"):
            return str(lib)

    # 尝试 python_ctypes 父目录的 build/ 子目录
    build_dir = pallas_py_dir.parent / "build"
    if build_dir.exists() and build_dir.is_dir():
        if sys.platform == "win32":
            for lib in build_dir.glob("*PallasSDK.dll"):
                return str(lib)
            for lib in build_dir.glob("*PallasSDK.lib"):
                return str(lib)
        elif sys.platform == "linux":
            for lib in build_dir.glob("*libPallasSDK.so"):
                return str(lib)
        elif sys.platform == "darwin":
            for lib in build_dir.glob("*PallasSDK.dylib"):
                return str(lib)

    # 回退：直接找系统路径中的 RobotSDK
    if sys.platform == "win32":
        return "PallasSDK.dll"
    elif sys.platform == "linux":
        return "libPallasSDK.so"
    else:
        return "libPallasSDK.dylib"


_lib_path = _find_library()

try:
    _lib = ctypes.CDLL(_lib_path)
except OSError as e:
    raise OSError(
        f"无法加载 PallasSDK 共享库（{_lib_path}）：{e}\n"
        "请确认库文件已编译并放置在正确路径，或设置 PALLAS_SDK_LIB 环境变量。"
    )


# ---------------------------------------------------------------------------
# 回调类型定义（供 set_*_callback 系列函数使用）
# ---------------------------------------------------------------------------

# HiddenCallback_C: void (*)(int type, int robotId, const char** values, int valuesCount)
_HiddenCallbackCFuncType = ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_int,
                                              ctypes.POINTER(ctypes.c_char_p), ctypes.c_int)

# QRLCallback_C: void (*)(const char* qrlData)
_QRLCallbackCFuncType = ctypes.CFUNCTYPE(None, ctypes.c_char_p)

# AsyncCommandCallback_C: void (*)(uint32_t handle, int errorId, const char* result)
_AsyncCommandCallbackCFuncType = ctypes.CFUNCTYPE(None, ctypes.c_uint32, ctypes.c_int, ctypes.c_char_p)

# FileProgressCallback_C: void (*)(const char*, int)
_FileProgressCallbackCFuncType = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int)

# EMIFeedbackCallback_C: void (*)(const EMIFeedbackData* data)
# EMIFeedbackData 结构体定义在 _types.py，已在文件顶部导入
_EMIFeedbackCallbackCFuncType = ctypes.CFUNCTYPE(None, ctypes.POINTER(EMIFeedbackData))


# ---------------------------------------------------------------------------
# Python 回调存储（防止被 GC 回收）
# ---------------------------------------------------------------------------
# ctypes 回调函数在被传递给 C 后必须保持活跃，这里用字典持有引用
_python_callbacks = {
    "hidden": {},
    "qrl": {},
    "async_command": {},
    "file_progress": {},
    "emi_feedback": {},
}


def _make_hidden_callback_wrapper(py_callback):
    """为 Python 回调创建 C 回调包装器。"""
    def wrapper(data_type, robot_id, values, values_count):
        try:
            vals = [values[i].decode() if values[i] else "" for i in range(values_count)]
            py_callback(data_type, robot_id, vals)
        except Exception:
            pass  # 回调中的异常不传播到 C
    return _HiddenCallbackCFuncType(wrapper)


def _make_qrl_callback_wrapper(py_callback):
    """为 Python QRL 回调创建 C 回调包装器。"""
    def wrapper(qrl_data):
        try:
            data = qrl_data.decode() if qrl_data else ""
            py_callback(data)
        except Exception:
            pass
    return _QRLCallbackCFuncType(wrapper)


def _make_async_command_callback_wrapper(py_callback):
    """为 Python 异步命令回调创建 C 回调包装器。"""
    def wrapper(handle, error_id, result):
        try:
            res = result.decode() if result else ""
            py_callback(handle, error_id, res)
        except Exception:
            pass
    return _AsyncCommandCallbackCFuncType(wrapper)


def _make_file_progress_callback_wrapper(py_callback):
    """为 Python 文件进度回调创建 C 回调包装器。"""
    def wrapper(filename, progress):
        try:
            fn = filename.decode() if filename else ""
            py_callback(fn, progress)
        except Exception:
            pass
    return _FileProgressCallbackCFuncType(wrapper)


def _make_emi_feedback_callback_wrapper(py_callback):
    """为 Python EMI 反馈回调创建 C 回调包装器。"""
    def wrapper(data_ptr):
        try:
            py_callback(data_ptr.contents)
        except Exception:
            pass
    return _EMIFeedbackCallbackCFuncType(wrapper)


# ---------------------------------------------------------------------------
# Controller 函数签名注册
# ---------------------------------------------------------------------------

def _restype_int(lib, name):
    f = getattr(lib, name)
    f.restype = ctypes.c_int
    return f

def _arg_restype_int(lib, name, *argtypes):
    f = getattr(lib, name)
    f.argtypes = list(argtypes)
    f.restype = ctypes.c_int
    return f

def _arg_restype_void(lib, name, *argtypes):
    f = getattr(lib, name)
    f.argtypes = list(argtypes)
    f.restype = None
    return f

# Logger 日志接口（用于 Python ctypes 调用）
_arg_restype_void(_lib, "logger_log", ctypes.c_int, ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p)

# --- void* 创建/销毁 ---
_arg_restype_int(_lib, "controller_create", ctypes.c_void_p, ctypes.c_bool)
_arg_restype_int(_lib, "controller_destroy", ctypes.c_void_p)

# --- 连接 ---
_arg_restype_int(_lib, "controller_connect", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int)
_arg_restype_int(_lib, "controller_disconnect", ctypes.c_void_p)
_arg_restype_int(_lib, "controller_is_connected", ctypes.c_void_p, ctypes.POINTER(ctypes.c_bool))
_arg_restype_int(_lib, "controller_get_id", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))

# --- 版本 ---
_arg_restype_int(_lib, "controller_get_sdk_version", ctypes.c_void_p, ctypes.POINTER(ctypes.c_char_p))
_arg_restype_int(_lib, "controller_get_version", ctypes.c_void_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_int, ctypes.c_uint32)

# --- 机器人管理 ---
_arg_restype_int(_lib, "controller_add_robot", ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p)
_arg_restype_int(_lib, "controller_get_robot", ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p)
_arg_restype_int(_lib, "controller_reset_robots", ctypes.c_void_p)
_arg_restype_int(_lib, "controller_move_robots", ctypes.c_void_p, ctypes.POINTER(IPCCommandPacket))

# --- 单位 ---
_arg_restype_int(_lib, "controller_set_data_unit", ctypes.c_void_p, ctypes.c_int)

# --- 登录 ---
_arg_restype_int(_lib, "controller_login", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_logout", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)

# --- 控制 ---
_arg_restype_int(_lib, "controller_set_power_enable", ctypes.c_void_p, ctypes.c_bool, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_do", ctypes.c_void_p, ctypes.c_int, ctypes.c_bool, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_dio", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_bool), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_io_pulse", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_ao", ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_aio", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_uint32)

# --- 状态 ---
_arg_restype_int(_lib, "controller_get_status", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_int)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_thread_command", ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_thread_state", ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_activated_project_status", ctypes.c_void_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_int, ctypes.c_uint32)

# --- 隐藏数据上报 ---
_arg_restype_int(_lib, "controller_set_hidden_on", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_hidden_off", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_hidden_period", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)

# --- 日志 ---
_arg_restype_int(_lib, "controller_set_logger_level", ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "controller_set_logger_file_size", ctypes.c_void_p, ctypes.c_size_t)
_arg_restype_int(_lib, "controller_set_logger_file_path", ctypes.c_void_p, ctypes.c_char_p)

# --- IDN 读写 ---
_arg_restype_int(_lib, "controller_idn_read", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p,
                                              ctypes.POINTER(ctypes.c_char_p), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_idn_write", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p,
                                               ctypes.c_int, ctypes.c_uint32)

# --- 通用信息 ---
_arg_restype_int(_lib, "controller_get_robot_number", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_cpu", ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_memory", ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_mac", ctypes.c_void_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_login_count", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_monitor_time", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_product_identity", ctypes.c_void_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_time", ctypes.c_void_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_time", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_clear_variables", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_ip", ctypes.c_void_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_flush_buffer", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_load_project", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_load_project", ctypes.c_void_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_compile_load_project", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_clear_alarm", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_sd_number", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_sd_format", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_sd_repair", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_close_led", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_update_flag", ctypes.c_void_p,
    ctypes.POINTER(ctypes.POINTER(ctypes.c_int)), ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_update_load_flag", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_reboot", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_check_error", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.POINTER(ctypes.c_char_p)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 断点 ---
_arg_restype_int(_lib, "controller_add_breakpoint", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p,
    ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_remove_breakpoint", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p,
    ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_breakpoints", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_char_p)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_clear_breakpoint", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_add_condition_breakpoint", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_remove_condition_breakpoint", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_clear_condition_breakpoint", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_condition_breakpoint", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_char_p)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 变量 ---
_arg_restype_int(_lib, "controller_set_variable", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_variable", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p,
    ctypes.POINTER(ctypes.c_char_p), ctypes.c_int, ctypes.c_uint32)

# --- 线程属性 ---
_arg_restype_int(_lib, "controller_add_thread_property", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_remove_thread_property", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)

# --- 文件传输 ---
_arg_restype_int(_lib, "controller_connect_tftp_server", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int)
_arg_restype_int(_lib, "controller_disconnect_tftp_server", ctypes.c_void_p)
_arg_restype_int(_lib, "controller_download_file", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint16, ctypes.c_uint8)
_arg_restype_int(_lib, "controller_upload_file", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint16, ctypes.c_uint8)

# --- QSIP 扫描 ---
_arg_restype_int(_lib, "controller_qsip_scan", ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(ctypes.POINTER(BrowseRespItem)),
    ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "controller_qsip_set_scan_interface", ctypes.c_void_p, ctypes.c_char_p)
_arg_restype_int(_lib, "free_browse_result", ctypes.c_void_p, ctypes.c_int)

# --- QSIP 文件系统 ---
_arg_restype_int(_lib, "controller_qsip_file_system_operate", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p,
    ctypes.c_uint16, ctypes.c_uint8, ctypes.POINTER(ctypes.POINTER(ctypes.c_char_p)), ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "free_file_system_result", ctypes.POINTER(ctypes.c_char_p), ctypes.c_int)

# --- QSIP 铭牌 ---
_arg_restype_int(_lib, "controller_qsip_get_scan_extension_info", ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_uint8)),
    ctypes.POINTER(ctypes.POINTER(ctypes.c_char_p)), ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "free_nameplate_result", ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_char_p), ctypes.c_int)

# --- QSIP Sercos IDN ---
_arg_restype_int(_lib, "controller_qsip_sercos_idn_read", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p))
_arg_restype_int(_lib, "controller_qsip_sercos_idn_write", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p)

# --- QSIP IDN ---
_arg_restype_int(_lib, "controller_qsip_idn_read",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p))
_arg_restype_int(_lib, "controller_qsip_idn_write", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p)

# --- QSIP MTD ---
_arg_restype_int(_lib, "controller_qsip_mtd_read",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint16, ctypes.POINTER(ctypes.c_char_p))
_arg_restype_int(_lib, "controller_qsip_mtd_write",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint16, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_qsip_mtd_flush", ctypes.c_void_p, ctypes.c_void_p)

# --- QSIP IP / 重启 ---
_arg_restype_int(_lib, "controller_qsip_set_ip", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_bool)
_arg_restype_int(_lib, "controller_qsip_reboot", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32)

# --- QSIP 终端 ---
_arg_restype_int(_lib, "controller_qsip_terminal_open", ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_int)
_arg_restype_int(_lib, "controller_qsip_terminal_input", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_qsip_terminal_output", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p))
_arg_restype_int(_lib, "controller_qsip_terminal_close", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int)

# --- QSIP 升级与复位 ---
_arg_restype_int(_lib, "controller_qsip_clear_update_flags", ctypes.c_void_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_qsip_firmware_update", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_bool)
_arg_restype_int(_lib, "controller_qsip_firmware_update_manual", ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_bool)
_arg_restype_int(_lib, "controller_qsip_config_update_manual", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_bool)
_arg_restype_int(_lib, "controller_qsip_mtd_update_manual", ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p)

# --- I/O 强制 ---
_arg_restype_int(_lib, "controller_set_io_force", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_io_force", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- GInt / GBool / GDouble / GString ---
_arg_restype_int(_lib, "controller_set_gint", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_gint", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_gbool", ctypes.c_void_p, ctypes.c_int, ctypes.c_bool, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_gbool", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_bool), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_gdouble", ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_gdouble", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_gstring", ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_gstring", ctypes.c_void_p, ctypes.c_int,
    ctypes.POINTER(ctypes.c_char_p), ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_global_variable", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
    ctypes.POINTER(ctypes.POINTER(ctypes.c_char_p)), ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 解决方案/项目/文件操作 ---
_arg_restype_int(_lib, "controller_create_solution", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_delete_solution", ctypes.c_void_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_rename_solution", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_set_main_project", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_create_project", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_delete_project", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_rename_project", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_set_main_file", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_create_qrl_file", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_delete_qrl_file", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_rename_qrl_file", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_get_file_content", ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p))
_arg_restype_int(_lib, "controller_save_file_content", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "controller_download_folder", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint16, ctypes.c_uint8)
_arg_restype_int(_lib, "controller_upload_folder", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint16, ctypes.c_uint8)

# --- 内存释放 ---
_arg_restype_int(_lib, "free_string", ctypes.c_char_p)
_arg_restype_int(_lib, "free_int_array", ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "free_bool_array", ctypes.POINTER(ctypes.c_bool))
_arg_restype_int(_lib, "free_string_array", ctypes.POINTER(ctypes.c_char_p), ctypes.c_int)
_arg_restype_int(_lib, "free_project_info", ctypes.POINTER(ProjectInfo), ctypes.c_int)
_arg_restype_int(_lib, "free_solution_info", ctypes.POINTER(SolutionInfo), ctypes.c_int)
_arg_restype_int(_lib, "free_qlv_file", ctypes.c_void_p)
_arg_restype_int(_lib, "free_browse_result", ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "free_file_system_result", ctypes.POINTER(ctypes.c_char_p), ctypes.c_int)
_arg_restype_int(_lib, "free_nameplate_result", ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_char_p), ctypes.c_int)

# --- 解决方案/项目查询 ---
_arg_restype_int(_lib, "controller_get_pallas_solution_list", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(SolutionInfo)),
    ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "controller_get_pallas_solution", ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_bool),
    ctypes.POINTER(ctypes.c_char_p), ctypes.POINTER(ctypes.c_char_p), ctypes.POINTER(ctypes.POINTER(ProjectInfo)), ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "controller_get_pallas_project", ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p),
    ctypes.POINTER(ctypes.c_char_p), ctypes.POINTER(ctypes.POINTER(ctypes.c_char_p)), ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.POINTER(ctypes.c_char_p)), ctypes.POINTER(ctypes.c_int))

# --- Latch ---
_arg_restype_int(_lib, "controller_get_latch_config_number", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_latch_count", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_latch_config", ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_latch_id", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int),
    ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_latch_config", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_latch_flush", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_latch_delay_time", ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_get_latch_result", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_bool,
    ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)

# --- Zone ---
_arg_restype_int(_lib, "controller_get_zone_config", ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "controller_set_zone_config", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)

# --- 线程属性 ---
_arg_restype_int(_lib, "controller_get_thread_property", ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 回调设置 ---
# 注意：set_*_callback 系列函数的 restype 是 int，返回 0 表示成功
# 其参数为 controller + C 函数指针（通过 argtypes 声明）
_callback_func_type = ctypes.c_void_p  # C 函数指针
_restype_int(_lib, "controller_set_hidden_callback").argtypes = [ctypes.c_void_p, _HiddenCallbackCFuncType]
_restype_int(_lib, "controller_set_qrl_callback").argtypes = [ctypes.c_void_p, _QRLCallbackCFuncType]
_restype_int(_lib, "controller_set_async_command_callback").argtypes = [ctypes.c_void_p, _AsyncCommandCallbackCFuncType]
_restype_int(_lib, "controller_set_file_progress_callback").argtypes = [ctypes.c_void_p, _FileProgressCallbackCFuncType]

# --- QLV 文件 ---
_arg_restype_int(_lib, "controller_get_qlv_file_content",
    ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(QLVFile))
_arg_restype_int(_lib, "controller_save_qlv_file_content",
    ctypes.c_void_p, ctypes.c_char_p, ctypes.POINTER(QLVFile))


# ---------------------------------------------------------------------------
# Robot 函数签名注册
# ---------------------------------------------------------------------------

# --- 基本信息 ---
_arg_restype_int(_lib, "robot_get_id", ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint8))
_arg_restype_int(_lib, "robot_get_controller_id", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "robot_get_axis_number", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 状态 ---
_arg_restype_int(_lib, "robot_get_state", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_int)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_cartesian_pos", ctypes.c_void_p, ctypes.POINTER(Location), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_joint_pos", ctypes.c_void_p, ctypes.POINTER(LocationJ), ctypes.c_int, ctypes.c_uint32)

# --- 速度/工具/坐标系/Profile ---
_arg_restype_int(_lib, "robot_set_speed", ctypes.c_void_p, ctypes.c_double, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_speed", ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_tool", ctypes.c_void_p, ctypes.POINTER(Tool), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_tool_index", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_tool", ctypes.c_void_p, ctypes.POINTER(Tool), ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_userframe", ctypes.c_void_p, ctypes.POINTER(UserFrame), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_userframe_index", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_userframe", ctypes.c_void_p, ctypes.POINTER(UserFrame), ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_profile", ctypes.c_void_p, ctypes.POINTER(Profile), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_profile", ctypes.c_void_p, ctypes.POINTER(Profile), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_frame_type", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_frame_type", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- Jog ---
_arg_restype_int(_lib, "robot_set_jog_mode", ctypes.c_void_p, ctypes.c_int, ctypes.c_bool, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_jog_mode", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_int)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_jog_distance", ctypes.c_void_p, ctypes.c_double, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_jog_distance", ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_uint32)

# --- 负载 ---
_arg_restype_int(_lib, "robot_set_payload_data",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_payload_data", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_payload",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_payload", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 限位/编码器 ---
_arg_restype_int(_lib, "robot_get_limit", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_encoder", ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 目标位置 ---
_arg_restype_int(_lib, "robot_get_target_locationj", ctypes.c_void_p, ctypes.POINTER(LocationJ), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_target_location", ctypes.c_void_p, ctypes.POINTER(Location), ctypes.c_int, ctypes.c_uint32)

# --- 速度/力矩/温度/电流 ---
_arg_restype_int(_lib, "robot_get_joint_speed", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_cartesian_speed", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_target_joint_speed", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_target_cartesian_speed", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_joint_torque", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_joint_temperature", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_joint_current", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_joint_acceleration", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_cartesian_force", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 模式 ---
_arg_restype_int(_lib, "robot_get_mode", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 坐标转换 ---
_arg_restype_int(_lib, "robot_to_cartesian", ctypes.c_void_p, ctypes.POINTER(LocationJ), ctypes.POINTER(Location), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_to_joint", ctypes.c_void_p, ctypes.POINTER(Location), ctypes.POINTER(LocationJ), ctypes.c_int, ctypes.c_uint32)

# --- 报警 ---
_arg_restype_int(_lib, "robot_clear_battery_alarm", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)

# --- 安装姿态 ---
_arg_restype_int(_lib, "robot_set_mount_pos",
    ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_mount_pos", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),
    ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 碰撞保护 ---
_arg_restype_int(_lib, "robot_set_collision_protection", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_collision_protection", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_collision_sensitivity", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_collision_sensitivity", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 力控 ---
_arg_restype_int(_lib, "robot_set_force_mode_param", ctypes.c_void_p, ctypes.POINTER(ForceModeParam), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_force_mode_param", ctypes.c_void_p, ctypes.POINTER(ForceModeParam), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_force_mode", ctypes.c_void_p, ctypes.c_uint8, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_free_drive_mode", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)

# --- 操作模式 ---
_arg_restype_int(_lib, "robot_set_operate_mode", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_operate_mode", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 绑定/分离 ---
_arg_restype_int(_lib, "robot_attach", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_detach", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_attached_id", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 抱闸/标定 ---
_arg_restype_int(_lib, "robot_set_axis_brake", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_axis_brake", ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_int)), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_axis_calib", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_get_axis_calib", ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)

# --- 运动 ---
_arg_restype_int(_lib, "robot_abort", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_movel_location", ctypes.c_void_p, ctypes.POINTER(Location),
    ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_movel_locationj", ctypes.c_void_p, ctypes.POINTER(LocationJ),
    ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_movej_location", ctypes.c_void_p, ctypes.POINTER(Location),
    ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_movej_locationj", ctypes.c_void_p, ctypes.POINTER(LocationJ),
    ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_arc_location", ctypes.c_void_p, ctypes.POINTER(Location),
    ctypes.POINTER(Location), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_arc_locationj", ctypes.c_void_p, ctypes.POINTER(LocationJ),
    ctypes.POINTER(LocationJ), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_circle_location", ctypes.c_void_p, ctypes.POINTER(Location),
    ctypes.POINTER(Location), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_circle_locationj", ctypes.c_void_p, ctypes.POINTER(LocationJ),
    ctypes.POINTER(LocationJ), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_axis", ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_along", ctypes.c_void_p, ctypes.c_char_p, ctypes.c_double, ctypes.c_int,
    ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_stop_jog", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_wait_for_eom", ctypes.c_void_p)
_arg_restype_int(_lib, "robot_move_l_offset_jm",
    ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_l_offset_jm_location",
    ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.POINTER(Location), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_l_offset_jm_locationj",
    ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.POINTER(LocationJ), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_l_offset",
    ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_l_offset_location",
    ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.POINTER(Location), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_l_offset_locationj",
    ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.POINTER(LocationJ), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_trigger",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_double, ctypes.c_int, ctypes.c_int, ctypes.c_int,
    ctypes.c_int, ctypes.c_double, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_move_jump_location", ctypes.c_void_p, ctypes.POINTER(Location),
    ctypes.c_double)
_arg_restype_int(_lib, "robot_move_jump_locationj", ctypes.c_void_p, ctypes.POINTER(LocationJ),
    ctypes.c_double)
_arg_restype_int(_lib, "robot_get_move_continue_type", ctypes.c_void_p, ctypes.POINTER(ctypes.c_int), ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_set_move_continue_type", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)

# --- 多机器人控制 ---
_arg_restype_int(_lib, "robot_emg_cmd", ctypes.c_void_p, ctypes.c_uint8, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32)

# --- EMI ---
_arg_restype_int(_lib, "robot_emi_enable", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_emi_enable_buffer", ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_emi_set_parameter", ctypes.c_void_p, ctypes.c_uint8, ctypes.c_float,
    ctypes.c_short, ctypes.c_uint8, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_emi_run", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_emi_stop", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_emi_start",ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint8, ctypes.c_float,
    ctypes.c_short, ctypes.c_uint8, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_emi_end", ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32)
_arg_restype_int(_lib, "robot_emi_set_data", ctypes.c_void_p, ctypes.c_uint8, ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double), ctypes.c_double, ctypes.POINTER(ctypes.c_ushort), ctypes.c_ushort,
    ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double))
_restype_int(_lib, "robot_set_emi_feedback_callback").argtypes = [ctypes.c_void_p, _EMIFeedbackCallbackCFuncType]

# --- 伺服 ---
_arg_restype_int(_lib, "robot_servo_j",
    ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double, ctypes.c_double)
_arg_restype_int(_lib, "robot_servo_l",
    ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_double, ctypes.c_double,
    ctypes.c_double, ctypes.c_double, ctypes.c_double)
_arg_restype_int(_lib, "robot_speed_j",
    ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_double, ctypes.c_double, ctypes.c_double)
_arg_restype_int(_lib, "robot_speed_l",
    ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_double, ctypes.c_double, ctypes.c_double)


# ---------------------------------------------------------------------------
# Jog shared 函数签名
# ---------------------------------------------------------------------------

class _JogParams_C(ctypes.Structure):
    _fields_ = [
        ("mode",       ctypes.c_int),
        ("frame_type", ctypes.c_int),
        ("direction",  ctypes.c_char_p),
        ("speed",      ctypes.c_double),
        ("distance",   ctypes.c_double),
        ("timestamp",  ctypes.c_int64),
    ]


_arg_restype_int(_lib, "jog_start", ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(_JogParams_C))
_arg_restype_int(_lib, "jog_stop", ctypes.c_void_p, ctypes.c_void_p)
_arg_restype_int(_lib, "jog_move_l", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(Location), ctypes.POINTER(LocationJ))

# ---------------------------------------------------------------------------
# Zone shared 函数签名
# ---------------------------------------------------------------------------

_arg_restype_int(_lib, "zone_get_zone_list", ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "zone_free_zone_list", ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "zone_get_zone_config", ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p)
_arg_restype_int(_lib, "zone_set_zone_config", ctypes.c_void_p, ctypes.c_void_p, ctypes.c_bool)

# ---------------------------------------------------------------------------
# ToolFrame shared 函数签名
# ---------------------------------------------------------------------------

_arg_restype_int(_lib, "tool_frame_get_list",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "tool_frame_free_list",
    ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "tool_frame_get_actived",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p)
_arg_restype_int(_lib, "tool_frame_update_frames",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_bool)
_arg_restype_int(_lib, "tool_frame_set_tool",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "tool_frame_calibrate_two_point",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p)
_arg_restype_int(_lib, "tool_frame_calibrate_four_point",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p,
    ctypes.c_void_p, ctypes.c_void_p)

# ---------------------------------------------------------------------------
# UserFrame shared 函数签名
# ---------------------------------------------------------------------------

_arg_restype_int(_lib, "user_frame_get_list",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "user_frame_free_list",
    ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "user_frame_get_actived",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p)
_arg_restype_int(_lib, "user_frame_update_frames",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_bool)
_arg_restype_int(_lib, "user_frame_set_userframe",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "user_frame_calibrate",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p)

# ---------------------------------------------------------------------------
# IO shared 函数签名
# ---------------------------------------------------------------------------

_arg_restype_int(_lib, "io_get_io_list",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_bool,
    ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "io_get_io_groups",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "io_update_io_groups",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "io_free_io_list",   ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "io_free_io_groups", ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "io_get_io_value",
    ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_bool))
_arg_restype_int(_lib, "io_set_io_value",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_bool, ctypes.c_bool, ctypes.c_bool)
_arg_restype_int(_lib, "io_get_io_force",
    ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_bool))
_arg_restype_int(_lib, "io_set_io_force",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_bool)
_arg_restype_int(_lib, "io_get_io_notes",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "io_update_io_notes",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "io_free_io_notes",  ctypes.c_void_p, ctypes.c_int)

# ---------------------------------------------------------------------------
# GlobalVariable shared 函数签名
# ---------------------------------------------------------------------------

_arg_restype_int(_lib, "global_variable_get_global_variable_list",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_int,
    ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "global_variable_get_global_variable",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_void_p)
_arg_restype_int(_lib, "global_variable_update_global_variables",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_bool)
_arg_restype_int(_lib, "global_variable_get_favorite_info_from_idn",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p)
_arg_restype_int(_lib, "global_variable_update_favorite_info_to_idn",
    ctypes.c_void_p, ctypes.c_void_p)
_arg_restype_int(_lib, "global_variable_get_favorite_list",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "global_variable_rename_favorite",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_bool)
_arg_restype_int(_lib, "global_variable_update_favorite",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p,
    ctypes.c_void_p, ctypes.c_int, ctypes.c_bool)
_arg_restype_int(_lib, "global_variable_free_global_variable_list",
    ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "global_variable_free_favorite_list",
    ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "global_variable_free_favorite", ctypes.c_void_p)

# ---------------------------------------------------------------------------
# Latch shared 函数签名
# ---------------------------------------------------------------------------

_arg_restype_int(_lib, "latch_get_list",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "latch_get_config",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p)
_arg_restype_int(_lib, "latch_set_config",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_bool)
_arg_restype_int(_lib, "latch_get_count",
    ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "latch_get_result",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_bool, ctypes.c_void_p)
_arg_restype_int(_lib, "latch_get_result_list",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_bool,
    ctypes.c_void_p, ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "latch_set_flush",
    ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "latch_set_delay_time",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_double)

# ---------------------------------------------------------------------------
# ModbusTCP shared 函数签名
# ---------------------------------------------------------------------------

_arg_restype_int(_lib, "modbus_tcp_get_settings",
    ctypes.c_void_p, ctypes.c_void_p)
_arg_restype_int(_lib, "modbus_tcp_update_settings",
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int)
_arg_restype_int(_lib, "modbus_tcp_extend_slave_amount",
    ctypes.c_void_p, ctypes.c_int)

# ---------------------------------------------------------------------------
# Common shared 函数签名
# ---------------------------------------------------------------------------

_arg_restype_int(_lib, "common_save_application_idn_to_file", ctypes.c_void_p)
_arg_restype_int(_lib, "common_save_pallas_idn_to_file",      ctypes.c_void_p)
_arg_restype_int(_lib, "common_save_all_idn_to_file",         ctypes.c_void_p)


# ---------------------------------------------------------------------------
# SharedSolidifiedInfo 函数签名
# ---------------------------------------------------------------------------

_arg_restype_int(_lib, "solidified_info_initialize", ctypes.c_char_p)
_arg_restype_int(_lib, "solidified_info_is_initialized")
_arg_restype_int(_lib, "solidified_info_get_modules",
    ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_int))
_arg_restype_int(_lib, "solidified_info_get_solidified_info",
    ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p,
    ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_int))
_lib.solidified_info_free_modules.argtypes = [ctypes.c_void_p, ctypes.c_int]
_lib.solidified_info_free_modules.restype = None
_lib.solidified_info_free_items.argtypes = [ctypes.c_void_p, ctypes.c_int]
_lib.solidified_info_free_items.restype = None
_arg_restype_int(_lib, "solidified_info_export_info",
    ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "solidified_info_import_info",
    ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "solidified_info_backup",
    ctypes.c_void_p, ctypes.c_char_p)
_arg_restype_int(_lib, "solidified_info_recover",
    ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p)
_arg_restype_int(_lib, "solidified_info_write",
    ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p)
_arg_restype_int(_lib, "solidified_info_read",
    ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p,
    ctypes.POINTER(ctypes.c_char_p))
_lib.solidified_info_free_string.argtypes = [ctypes.c_char_p]
_lib.solidified_info_free_string.restype = None
_arg_restype_int(_lib, "solidified_info_flush",
    ctypes.c_void_p, ctypes.c_char_p)


# ---------------------------------------------------------------------------
# Utils 工具函数签名（对应 RobotTypesExportC.h）
# ---------------------------------------------------------------------------

_arg_restype_int(_lib, "utils_frame_inv",
    ctypes.POINTER(Frame), ctypes.POINTER(Frame))

_arg_restype_int(_lib, "utils_frame_mul",
    ctypes.POINTER(Frame), ctypes.POINTER(Frame), ctypes.POINTER(Frame))

_arg_restype_int(_lib, "utils_frame_dist",
    ctypes.POINTER(Frame), ctypes.POINTER(Frame), ctypes.POINTER(ctypes.c_double))

_arg_restype_int(_lib, "utils_frame_euler_transfer",
    ctypes.POINTER(Frame), ctypes.c_int, ctypes.c_int, ctypes.POINTER(Frame))

_arg_restype_int(_lib, "utils_tool_calib",
    ctypes.POINTER(Frame), ctypes.c_int, ctypes.POINTER(Frame), ctypes.POINTER(ctypes.c_double))

_arg_restype_int(_lib, "utils_userframe_calib",
    ctypes.POINTER(Frame), ctypes.POINTER(Frame), ctypes.POINTER(Frame), ctypes.POINTER(Frame))


# ---------------------------------------------------------------------------
# 导出
# ---------------------------------------------------------------------------
__all__ = [
    "_lib", "PallasException", "_lib_path", "_JogParams_C",
    # 新增回调相关
    "_python_callbacks",
    "_make_hidden_callback_wrapper",
    "_make_qrl_callback_wrapper",
    "_make_async_command_callback_wrapper",
    "_make_file_progress_callback_wrapper",
    "_make_emi_feedback_callback_wrapper",
]
