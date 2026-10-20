"""
controller.py

Created on: 2026-4-13
Author: 0590

Controller 类 ctypes 封装，对应 pybind11 的 Controller 绑定。
通过 RobotSDK C API 操作控制器实例。
"""
import ctypes
from typing import List, Callable

from ._lib import (
    _lib,
    _python_callbacks,
    _make_hidden_callback_wrapper,
    _make_qrl_callback_wrapper,
    _make_async_command_callback_wrapper,
    _make_file_progress_callback_wrapper,
)
from ._utils import (
    PallasException, _call_int, _ensure_bytes, _ensure_str, _check_parameter,
    _free_string, _free_string_array, _free_int_array, _free_bool_array,
    MACRO_CMD_TIMEOUT, MACRO_CMD_HANDLE
)
from ._types import (
    Location, LocationJ, Tool, UserFrame, Profile,
    BrowseRespItem, NodeItem, LatchConfig, LatchResult,
    ZoneConfig, EMIFeedbackData, QLVFile,
    SolutionInfo, ProjectInfo, IPCCommandPacket,
)
from .robot import Robot  # noqa: F401


# ---------------------------------------------------------------------------
# Controller 类
# ---------------------------------------------------------------------------

class Controller:
    """
    控制器类，通过 C API 操作机器人控制器。

    示例：
        from pallas_py import Controller

        controller = Controller()
        controller.connect("192.168.1.10", 2094)
        controller.login()

        robot = controller.add_robot(1)
        pos = robot.get_joint_pos()
        print(pos)

        controller.logout()
        controller.disconnect()
    """

    def __init__(self, auto_init: bool = True):
        """
        创建 Controller 实例。

        @param auto_init 实例化对象是否自动获取控制权(login)和绑定机器人(attach)。
                         默认自动获取控制权和绑定机器人。
        """
        self._ptr = None
        self._controller_id = 0
        self.create(auto_init)

    # -------------------- 生命周期 --------------------

    def create(self, auto_init):
        """创建底层控制器对象（若 auto_init=False 时调用）。"""
        if self._ptr is not None:
            return
        self._ptr = ctypes.c_void_p()
        _call_int(self._controller_id, _lib.controller_create, ctypes.byref(self._ptr), auto_init)
        self._controller_id = self.get_id()

    def destroy(self):
        """销毁底层控制器对象。"""
        if self._ptr is not None:
            _lib.controller_destroy(self._ptr)
            self._ptr = None

    def __del__(self):
        try:
            self.destroy()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.destroy()

    @property
    def _cptr(self) -> ctypes.c_void_p:
        if self._ptr is None:
            _check_parameter(self._controller_id, "Controller not initialized, please call create() first")
        return self._ptr

    # -------------------- 连接 --------------------

    def connect(self, ip: str, port: int = 2094) -> int:
        """
        连接到机器人控制器。

        @param ip 控制器 IP 地址
        @param port 端口号，默认 2094
        @return 错误码，0 表示成功
        """
        return _call_int(self._controller_id, _lib.controller_connect, self._cptr, _ensure_bytes(ip), port)

    def disconnect(self) -> int:
        """断开与控制器的连接。"""
        return _call_int(self._controller_id, _lib.controller_disconnect, self._cptr)

    def is_connected(self) -> bool:
        """返回是否已连接。"""
        result = ctypes.c_bool()
        _call_int(self._controller_id, _lib.controller_is_connected, self._cptr, ctypes.byref(result))
        return result.value

    def get_id(self) -> int:
        """获取 Controller 实例的 ID 编号，从 1 开始。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_id, self._cptr, ctypes.byref(result))
        return result.value

    # -------------------- 版本 --------------------

    def get_sdk_version(self) -> str:
        """获取 SDK 版本号（包含 Python 接口版本）。"""
        version_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_get_sdk_version, self._cptr, ctypes.byref(version_ptr))
        sdk_ver = _ensure_str(version_ptr.value)
        _free_string(version_ptr)
        # 动态导入版本号避免循环导入
        from . import __version__
        return f"{sdk_ver}; PythonSDK_V{__version__}"

    def get_controller_version(self, timeout: int = MACRO_CMD_TIMEOUT,
                               handle: int = MACRO_CMD_HANDLE) -> str:
        """
        获取控制器固件版本。

        @param timeout 超时时间(ms)
        @param handle 命令句柄
        @return 控制器固件版本字符串
        """
        version_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_get_version,
                  self._cptr, ctypes.byref(version_ptr), timeout, handle)
        pallas_ver = _ensure_str(version_ptr.value)
        _free_string(version_ptr)
        return pallas_ver

    # -------------------- 机器人管理 --------------------

    def add_robot(self, robot_id: int) -> Robot:
        """
        向控制器注册机器人并返回 Robot 对象。

        @param robot_id 机器人编号(1-based)
        @return Robot 实例，其 C 指针已正确指向 C++ Robot 对象
        """
        robot_ptr_out = ctypes.c_void_p(0)
        _call_int(self._controller_id, _lib.controller_add_robot, self._cptr, robot_id, ctypes.byref(robot_ptr_out))
        robot = Robot()
        robot.set_ptr(robot_ptr_out)
        return robot

    def get_robot(self, robot_id: int = 1) -> Robot:
        """
        获取已注册的 Robot 实例。

        @param robot_id 机器人编号（1-based）
        @return Robot 对象
        """
        robot_ptr_out = ctypes.c_void_p(0)
        _call_int(self._controller_id, _lib.controller_get_robot, self._cptr, robot_id, ctypes.byref(robot_ptr_out))
        robot = Robot()
        robot.set_ptr(robot_ptr_out)
        return robot

    def reset_robots(self) -> int:
        """清除所有已注册的机器人。"""
        return _call_int(self._controller_id, _lib.controller_reset_robots, self._cptr)

    def get_robot_number(self, timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> int:
        """获取控制器下已登录的机器人数量。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_robot_number, self._cptr, ctypes.byref(result), timeout, handle)
        return result.value

 # -------------------- 多机器人控制 --------------------

    def move_robots(self, command: IPCCommandPacket) -> int:
        """
        多机器人控制指令

        @param command 多机器人控制数据（IPCCommandPacket 结构体或字典）
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(self._controller_id, _lib.controller_move_robots, self._cptr, ctypes.byref(command))

    # -------------------- 单位 --------------------

    def set_data_unit(self, is_degrees_millimeter: int) -> int:
        """设置数据单位（0=弧度/mm，1=角度/mm）。"""
        return _call_int(self._controller_id, _lib.controller_set_data_unit, self._cptr,
                         is_degrees_millimeter)

    # -------------------- 登录 --------------------

    def login(self, timeout: int = MACRO_CMD_TIMEOUT,
              handle: int = MACRO_CMD_HANDLE) -> int:
        """登录控制器。"""
        return _call_int(self._controller_id, _lib.controller_login, self._cptr, timeout, handle)

    def logout(self, timeout: int = MACRO_CMD_TIMEOUT,
               handle: int = MACRO_CMD_HANDLE) -> int:
        """登出控制器。"""
        return _call_int(self._controller_id, _lib.controller_logout, self._cptr, timeout, handle)

    # -------------------- 电源/IO --------------------

    def set_power_enable(self, on_off: bool, robot_id: int = 1,
                          timeout: int = MACRO_CMD_TIMEOUT,
                          handle: int = MACRO_CMD_HANDLE) -> int:
        """设置机器人电源使能。"""
        return _call_int(self._controller_id, _lib.controller_set_power_enable,
                         self._cptr, on_off, robot_id, timeout, handle)

    def set_do(self, index: int, value: bool,
               timeout: int = MACRO_CMD_TIMEOUT,
               handle: int = MACRO_CMD_HANDLE) -> int:
        """设置数字输出。"""
        return _call_int(self._controller_id, _lib.controller_set_do, self._cptr, index, value, timeout, handle)

    def get_dio(self, index: int, timeout: int = MACRO_CMD_TIMEOUT,
                handle: int = MACRO_CMD_HANDLE) -> bool:
        """获取数字 IO 状态。"""
        result = ctypes.c_bool()
        _call_int(self._controller_id, _lib.controller_get_dio, self._cptr, index, ctypes.byref(result), timeout, handle)
        return result.value

    def set_ao(self, index: int, value: float, timeout: int = MACRO_CMD_TIMEOUT,
                 handle: int = MACRO_CMD_HANDLE) -> None:
        """设置模拟量输出（只支持 AOUT）。

        @param index I/O 索引号，如 70101
        @param value 模拟量数值
        """
        _call_int(self._controller_id, _lib.controller_set_ao, self._cptr, index, value, timeout, handle)

    def get_aio(self, index: int, timeout: int = MACRO_CMD_TIMEOUT,
                handle: int = MACRO_CMD_HANDLE) -> float:
        """获取模拟量 I/O 状态（支持 AIN、AOUT 和 SAIN）。

        @param index I/O 索引号，如 60101
        @return 模拟量数值
        """
        result = ctypes.c_double()
        _call_int(self._controller_id, _lib.controller_get_aio, self._cptr, index, ctypes.byref(result), timeout, handle)
        return result.value

    # -------------------- 线程/状态 --------------------

    def get_status(self, timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> List[int]:
        """获取系统状态列表。"""
        result_ptr = ctypes.POINTER(ctypes.c_int)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_status, self._cptr,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_int_array(result_ptr)
        return result

    def set_thread_command(self, cmd: int, name: str,
                           timeout: int = MACRO_CMD_TIMEOUT,
                           handle: int = MACRO_CMD_HANDLE) -> int:
        """向指定线程发送命令（Start/Stop/Abort/Pause/Retry/Continue）。"""
        return _call_int(self._controller_id, _lib.controller_set_thread_command,
                         self._cptr, cmd, _ensure_bytes(name), timeout, handle)

    def get_thread_state(self, name: str,
                         timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> int:
        """获取指定线程的状态。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_thread_state,
                  self._cptr, _ensure_bytes(name), ctypes.byref(result), timeout, handle)
        return result.value

    def get_activated_project_status(self, timeout: int = MACRO_CMD_TIMEOUT,
                                      handle: int = MACRO_CMD_HANDLE) -> str:
        """获取当前激活项目状态。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_get_activated_project_status, self._cptr, ctypes.byref(result_ptr), timeout, handle)
        activated_project_status = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return activated_project_status

    # -------------------- 隐藏数据上报 --------------------

    def set_hidden_on(self, data_type: int,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> int:
        """开启指定类型的隐藏数据主动上报。"""
        return _call_int(self._controller_id, _lib.controller_set_hidden_on, self._cptr, data_type, timeout, handle)

    def set_hidden_off(self, data_type: int,
                       timeout: int = MACRO_CMD_TIMEOUT,
                       handle: int = MACRO_CMD_HANDLE) -> int:
        """关闭指定类型的隐藏数据主动上报。"""
        return _call_int(self._controller_id, _lib.controller_set_hidden_off, self._cptr, data_type, timeout, handle)

    def set_hidden_period(self, period: int,
                           timeout: int = MACRO_CMD_TIMEOUT,
                           handle: int = MACRO_CMD_HANDLE) -> int:
        """设置隐藏数据上报周期（ms）。"""
        return _call_int(self._controller_id, _lib.controller_set_hidden_period, self._cptr, period, timeout, handle)

    # -------------------- 日志 --------------------

    def set_logger_level(self, level: int) -> int:
        """设置日志等级（0=Debug ~ 5=Fatal）。"""
        return _call_int(self._controller_id, _lib.controller_set_logger_level, self._cptr, level)

    def set_logger_file_size(self, size: int) -> int:
        """设置日志文件大小上限（字节）。"""
        return _lib.controller_set_logger_file_size(self._cptr, size)

    def set_logger_file_path(self, path: str) -> int:
        """设置日志文件路径。"""
        return _lib.controller_set_logger_file_path(self._cptr, _ensure_bytes(path))

    # -------------------- IDN 读写 --------------------

    def idn_read(self, module: str, index: str = "1.0.0", value: str = "",
                 timeout: int = MACRO_CMD_TIMEOUT,
                 handle: int = MACRO_CMD_HANDLE) -> str:
        """
        读取 IDN 参数。
        @return 字符串值
        """
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_idn_read,
                  self._cptr, _ensure_bytes(module), _ensure_bytes(index), _ensure_bytes(value),
                  ctypes.byref(result_ptr), timeout, handle)
        result = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return result

    def idn_write(self, module: str, index: str, value: str,
                  timeout: int = MACRO_CMD_TIMEOUT,
                  handle: int = MACRO_CMD_HANDLE) -> int:
        """写入 IDN 参数。"""
        return _call_int(self._controller_id, _lib.controller_idn_write,
                         self._cptr, _ensure_bytes(module), _ensure_bytes(index), _ensure_bytes(value),
                         timeout, handle)

    # -------------------- 系统信息 --------------------

    def get_cpu(self, timeout: int = MACRO_CMD_TIMEOUT,
                handle: int = MACRO_CMD_HANDLE) -> float:
        """获取控制器 CPU 使用率（%）。"""
        result = ctypes.c_double()
        _call_int(self._controller_id, _lib.controller_get_cpu, self._cptr, ctypes.byref(result), timeout, handle)
        return result.value

    def get_memory(self, timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> float:
        """获取控制器内存使用率（%）。"""
        result = ctypes.c_double()
        _call_int(self._controller_id, _lib.controller_get_memory, self._cptr, ctypes.byref(result), timeout, handle)
        return result.value

    def get_mac(self, timeout: int = MACRO_CMD_TIMEOUT,
                handle: int = MACRO_CMD_HANDLE) -> str:
        """获取控制器 MAC 地址。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_get_mac, self._cptr, ctypes.byref(result_ptr), timeout, handle)
        mac = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return mac

    def get_login_count(self, timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> int:
        """返回已登录次数（>1 表示有其他上位机已登录）。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_login_count, self._cptr, ctypes.byref(result), timeout, handle)
        return result.value

    def get_monitor_time(self, timeout: int = MACRO_CMD_TIMEOUT,
                          handle: int = MACRO_CMD_HANDLE) -> int:
        """获取机器人运行时间（ms）。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_monitor_time, self._cptr, ctypes.byref(result), timeout, handle)
        return result.value

    def get_product_identity(self, timeout: int = MACRO_CMD_TIMEOUT,
                              handle: int = MACRO_CMD_HANDLE) -> str:
        """获取产品标识。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_get_product_identity, self._cptr, ctypes.byref(result_ptr), timeout, handle)
        product_identity = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return product_identity

    def get_time(self, timeout: int = MACRO_CMD_TIMEOUT,
                 handle: int = MACRO_CMD_HANDLE) -> str:
        """获取控制器时间，格式 "yyyy-MM-dd HH:mm:ss.SSS"。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_get_time, self._cptr, ctypes.byref(result_ptr), timeout, handle)
        time_str = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return time_str

    def set_time(self, time_str: str,
                 timeout: int = MACRO_CMD_TIMEOUT,
                 handle: int = MACRO_CMD_HANDLE) -> int:
        """设置控制器时间。"""
        return _call_int(self._controller_id, _lib.controller_set_time, self._cptr, _ensure_bytes(time_str), timeout, handle)

    def clear_variables(self, timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> int:
        """清空变量。"""
        return _call_int(self._controller_id, _lib.controller_clear_variables, self._cptr, timeout, handle)

    def get_ip(self, timeout: int = MACRO_CMD_TIMEOUT,
               handle: int = MACRO_CMD_HANDLE) -> str:
        """获取控制器 IP 地址。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_get_ip, self._cptr, ctypes.byref(result_ptr), timeout, handle)
        ip = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return ip

    def flush_buffer(self, timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> int:
        """清空指令缓存（包括阻塞和非阻塞指令）。"""
        return _call_int(self._controller_id, _lib.controller_flush_buffer, self._cptr, timeout, handle)

    # -------------------- 项目加载 --------------------

    def set_load_project(self, solution_name: str,
                         timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> int:
        """设置加载的解决方案。"""
        return _call_int(self._controller_id, _lib.controller_set_load_project,
                         self._cptr, _ensure_bytes(solution_name), timeout, handle)

    def get_load_project(self, timeout: int = MACRO_CMD_TIMEOUT,
                          handle: int = MACRO_CMD_HANDLE) -> str:
        """获取当前加载的解决方案名称。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_get_load_project, self._cptr, ctypes.byref(result_ptr), timeout, handle)
        solution_name = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return solution_name

    def compile_load_project(self, timeout: int = MACRO_CMD_TIMEOUT,
                              handle: int = MACRO_CMD_HANDLE) -> int:
        """编译当前加载的项目。"""
        return _call_int(self._controller_id, _lib.controller_compile_load_project, self._cptr, timeout, handle)

    # -------------------- 报警 --------------------

    def clear_alarm(self, robot_index: int = 1, axis_index: int = 0,
                    error_type: int = 1,
                    timeout: int = MACRO_CMD_TIMEOUT,
                    handle: int = MACRO_CMD_HANDLE) -> int:
        """清除报警。"""
        return _call_int(self._controller_id, _lib.controller_clear_alarm,
                         self._cptr, robot_index, axis_index, error_type, timeout, handle)

    # -------------------- SD 卡 --------------------

    def get_sd_number(self, timeout: int = MACRO_CMD_TIMEOUT,
                       handle: int = MACRO_CMD_HANDLE) -> int:
        """获取 SD 卡数量。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_sd_number, self._cptr, ctypes.byref(result), timeout, handle)
        return result.value

    def sd_format(self, timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> int:
        """格式化 SD 卡。"""
        return _call_int(self._controller_id, _lib.controller_sd_format, self._cptr, timeout, handle)

    def sd_repair(self, timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> int:
        """修复 SD 卡。"""
        return _call_int(self._controller_id, _lib.controller_sd_repair, self._cptr, timeout, handle)

    # -------------------- LED --------------------

    def close_led(self, timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> int:
        """关闭机器人本体 LED 灯。"""
        return _call_int(self._controller_id, _lib.controller_close_led, self._cptr, timeout, handle)

    # -------------------- 升级标志 --------------------

    def get_update_flag(self, timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> List[int]:
        """获取升级标志 [加载标志, 启动标志, 升级标志, 自动升级标志]。"""
        result_ptr = ctypes.POINTER(ctypes.c_int)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_update_flag, self._cptr,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_int_array(result_ptr)
        return result

    def set_update_load_flag(self, flag: int,
                              timeout: int = MACRO_CMD_TIMEOUT,
                              handle: int = MACRO_CMD_HANDLE) -> int:
        """设置升级的启动标志。"""
        return _call_int(self._controller_id, _lib.controller_set_update_load_flag, self._cptr, flag, timeout, handle)

    def reboot(self, timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> int:
        """重启控制器。"""
        return _call_int(self._controller_id, _lib.controller_reboot, self._cptr, timeout, handle)

    def check_error(self, level: int = 0, key: int = 0,
                        timeout: int = MACRO_CMD_TIMEOUT,
	                    handle: int = MACRO_CMD_HANDLE) -> List[str]:
        """获取错误列表。"""
        result_ptr = ctypes.POINTER(ctypes.c_char_p)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_check_error, self._cptr, level, key,
	              ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [_ensure_str(result_ptr[i]) for i in range(count)]
        _free_string_array(result_ptr, count)
        return result

    # -------------------- 断点 --------------------

    def add_breakpoint(self, solution_name: str, project_name: str,
                       file_name: str, line_number: int,
                       condition: str = "",
                       timeout: int = MACRO_CMD_TIMEOUT,
                       handle: int = MACRO_CMD_HANDLE) -> int:
        """添加断点。"""
        return _call_int(self._controller_id, _lib.controller_add_breakpoint,
                         self._cptr, _ensure_bytes(solution_name), _ensure_bytes(project_name),
                         _ensure_bytes(file_name), line_number, _ensure_bytes(condition),
                         timeout, handle)

    def remove_breakpoint(self, solution_name: str, project_name: str,
                          file_name: str, line_number: int,
                          timeout: int = MACRO_CMD_TIMEOUT,
                          handle: int = MACRO_CMD_HANDLE) -> int:
        """删除断点。"""
        return _call_int(self._controller_id, _lib.controller_remove_breakpoint,
                         self._cptr, _ensure_bytes(solution_name), _ensure_bytes(project_name),
                         _ensure_bytes(file_name), line_number, timeout, handle)

    def get_breakpoints(self, timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> List[str]:
        """获取断点列表。"""
        result_ptr = ctypes.POINTER(ctypes.c_char_p)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_breakpoints, self._cptr,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [_ensure_str(result_ptr[i]) for i in range(count)]
        _free_string_array(result_ptr, count)
        return result

    def clear_breakpoint(self, solution_name: str = "", project_name: str = "",
                          timeout: int = MACRO_CMD_TIMEOUT,
                          handle: int = MACRO_CMD_HANDLE) -> int:
        """清空断点。"""
        return _call_int(self._controller_id, _lib.controller_clear_breakpoint,
                         self._cptr, _ensure_bytes(solution_name), _ensure_bytes(project_name),
                         timeout, handle)

    # -------------------- 条件断点 --------------------

    def add_condition_breakpoint(self, thread_name: str, condition: str,
                                 timeout: int = MACRO_CMD_TIMEOUT,
                                 handle: int = MACRO_CMD_HANDLE) -> int:
        """添加条件断点。"""
        return _call_int(self._controller_id, _lib.controller_add_condition_breakpoint,
                         self._cptr, _ensure_bytes(thread_name), _ensure_bytes(condition),
                         timeout, handle)

    def remove_condition_breakpoint(self, thread_name: str, condition: str,
                                     timeout: int = MACRO_CMD_TIMEOUT,
                                     handle: int = MACRO_CMD_HANDLE) -> int:
        """删除条件断点。"""
        return _call_int(self._controller_id, _lib.controller_remove_condition_breakpoint,
                         self._cptr, _ensure_bytes(thread_name), _ensure_bytes(condition),
                         timeout, handle)

    def clear_condition_breakpoint(self, timeout: int = MACRO_CMD_TIMEOUT,
                                    handle: int = MACRO_CMD_HANDLE) -> int:
        """清空条件断点。"""
        return _call_int(self._controller_id, _lib.controller_clear_condition_breakpoint, self._cptr, timeout, handle)

    def get_condition_breakpoint(self, timeout: int = MACRO_CMD_TIMEOUT,
                                  handle: int = MACRO_CMD_HANDLE) -> List[str]:
        """获取条件断点列表。"""
        result_ptr = ctypes.POINTER(ctypes.c_char_p)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_condition_breakpoint, self._cptr,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [_ensure_str(result_ptr[i]) for i in range(count)]
        _free_string_array(result_ptr, count)
        return result

    # -------------------- 变量 --------------------

    def set_variable(self, thread_name: str, variable_name: str, value: str,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> int:
        """设置变量值（支持 local/global）。"""
        return _call_int(self._controller_id, _lib.controller_set_variable,
                         self._cptr, _ensure_bytes(thread_name), _ensure_bytes(variable_name),
                         _ensure_bytes(value), timeout, handle)

    def get_variable(self, thread_name: str, variable_name: str,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> str:
        """获取变量值（支持 local/global）。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_get_variable,
                  self._cptr, _ensure_bytes(thread_name), _ensure_bytes(variable_name),
                  ctypes.byref(result_ptr), timeout, handle)
        variable = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return variable

    # -------------------- TFTP 文件传输 --------------------

    def connect_tftp_server(self, server_ip: str, server_port: int = 69):
        """连接到控制器的 TFTP 服务端。"""
        _call_int(self._controller_id, _lib.controller_connect_tftp_server,
                  self._cptr, _ensure_bytes(server_ip), server_port)

    def disconnect_tftp_server(self):
        """断开 TFTP 连接。"""
        _call_int(self._controller_id, _lib.controller_disconnect_tftp_server, self._cptr)

    def download_file(self, remote_filename: str, local_filename: str,
                       node_id: int = 1, core_id: int = 1):
        """下载控制器上的文件到本地。"""
        _call_int(self._controller_id, _lib.controller_download_file, self._cptr,
                         _ensure_bytes(remote_filename), _ensure_bytes(local_filename),
                         node_id, core_id)

    def upload_file(self, local_filename: str, remote_filename: str,
                     node_id: int = 1, core_id: int = 1):
        """上传本地文件到控制器。"""
        _call_int(self._controller_id, _lib.controller_upload_file, self._cptr,
                         _ensure_bytes(local_filename), _ensure_bytes(remote_filename),
                         node_id, core_id)

    # -------------------- QSIP 扫描 --------------------

    def qsip_set_scan_interface(self, local_ip: str):
        """设置本地 IP 地址（扫描前调用）。"""
        _call_int(self._controller_id, _lib.controller_qsip_set_scan_interface, self._cptr, _ensure_bytes(local_ip))

    def qsip_scan(self, target_ip: str = "") -> List[BrowseRespItem]:
        """
        扫描网络节点。
        @return BrowseRespItem 列表
        """
        result_ptr = ctypes.POINTER(BrowseRespItem)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_qsip_scan, self._cptr, _ensure_bytes(target_ip),
                  ctypes.byref(result_ptr), ctypes.byref(result_count))
        count = result_count.value
        items = []
        for i in range(count):
            raw_item = result_ptr[i]
            item = BrowseRespItem()
            item._identifier = raw_item._identifier
            item._macAddress = raw_item._macAddress
            item._ipAddress = raw_item._ipAddress
            item._subnet = raw_item._subnet
            item._gateway = raw_item._gateway
            item._modelName = raw_item._modelName
            item.nodeCount = raw_item.nodeCount
            if raw_item.nodes and raw_item.nodeCount > 0:
                nodes_array = (NodeItem * raw_item.nodeCount)()
                for j in range(raw_item.nodeCount):
                    nodes_array[j] = raw_item.nodes[j]
                item.nodes = ctypes.cast(nodes_array, ctypes.POINTER(NodeItem))
            else:
                item.nodes = ctypes.POINTER(NodeItem)()
            print(item)
            items.append(item)
        _lib.free_browse_result(result_ptr, count)
        return items

    # -------------------- QSIP 文件系统 --------------------

    def qsip_file_system_operate(self, target: BrowseRespItem,
                                  cmd: int, param: str = "",
                                  node_id: int = 1, core_id: int = 1) -> List[str]:
        """
        执行 QSIP 文件系统操作。
        @return result_list
        """
        result_ptr = ctypes.POINTER(ctypes.c_char_p)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_qsip_file_system_operate,
            self._cptr, ctypes.byref(target), cmd, _ensure_bytes(param),
            node_id, core_id, ctypes.byref(result_ptr), ctypes.byref(result_count))
        count = result_count.value
        result_list = [_ensure_str(result_ptr[i]) for i in range(count)]
        _lib.free_file_system_result(result_ptr, count)
        return result_list

    # -------------------- QSIP 铭牌 --------------------

    def qsip_get_scan_extension_info(self, target: BrowseRespItem) -> dict:
        """
        获取铭牌信息。
        @return node_dict, 其中 node_dict 是 {node_id: node_name} 的字典
        """
        node_ids_ptr = ctypes.POINTER(ctypes.c_uint8)()
        node_names_ptr = ctypes.POINTER(ctypes.c_char_p)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_qsip_get_scan_extension_info,
                  self._cptr, ctypes.byref(target),
                  ctypes.byref(node_ids_ptr), ctypes.byref(node_names_ptr),
                  ctypes.byref(result_count))
        count = result_count.value
        node_dict = {node_ids_ptr[i]: _ensure_str(node_names_ptr[i]) for i in range(count)}
        _lib.free_nameplate_result(node_ids_ptr, node_names_ptr, count)
        return node_dict

    # -------------------- QSIP Sercos IDN --------------------

    def qsip_sercos_idn_read(self, target: BrowseRespItem, sercos_idn: str) -> str:
        """读取 Sercos IDN。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_qsip_sercos_idn_read,
                        self._cptr, ctypes.byref(target), _ensure_bytes(sercos_idn), ctypes.byref(result_ptr))
        result_string = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return result_string

    def qsip_sercos_idn_write(self, target: BrowseRespItem,
                               sercos_idn: str, value: str):
        """写入 Sercos IDN。"""
        _call_int(self._controller_id, _lib.controller_qsip_sercos_idn_write,
                         self._cptr, ctypes.byref(target),
                         _ensure_bytes(sercos_idn), _ensure_bytes(value))

    # -------------------- QSIP IDN --------------------

    def qsip_idn_read(self, target: BrowseRespItem,
                       module: str, index: str = "1.0.0",
                       value: str = "0") -> str:
        """读取 QSIP IDN。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_qsip_idn_read, self._cptr, ctypes.byref(target),
                       _ensure_bytes(module), _ensure_bytes(index), _ensure_bytes(value),
                       ctypes.byref(result_ptr))
        result_string = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return result_string

    def qsip_idn_write(self, target: BrowseRespItem,
                         module: str, index: str, value: str):
        """写入 QSIP IDN。"""
        _call_int(self._controller_id, _lib.controller_qsip_idn_write,
                         self._cptr, ctypes.byref(target),
                         _ensure_bytes(module), _ensure_bytes(index), _ensure_bytes(value))

    # -------------------- QSIP MTD --------------------

    def qsip_mtd_read(self, target: BrowseRespItem,
                       addr: str, length: int) -> str:
        """读取固化信息。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_qsip_mtd_read,
                        self._cptr, ctypes.byref(target), _ensure_bytes(addr), length,
                        ctypes.byref(result_ptr))
        result_string = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return result_string

    def qsip_mtd_write(self, target: BrowseRespItem,
                        addr: str, length: int, value: str):
        """写入固化信息（缓存）。"""
        _call_int(self._controller_id, _lib.controller_qsip_mtd_write,
                         self._cptr, ctypes.byref(target),
                         _ensure_bytes(addr), length, _ensure_bytes(value))

    def qsip_mtd_flush(self, target: BrowseRespItem):
        """固化信息烧录 Flash。"""
        _call_int(self._controller_id, _lib.controller_qsip_mtd_flush, self._cptr, ctypes.byref(target))

    # -------------------- QSIP 修改 IP / 重启 --------------------

    def qsip_set_ip(self, target: BrowseRespItem,
                     ip: str, subnet: str, gateway: str,
                     persistent: bool = True):
        """设置 QSIP 节点 IP 地址。"""
        _call_int(self._controller_id, _lib.controller_qsip_set_ip,
                         self._cptr, ctypes.byref(target),
                         _ensure_bytes(ip), _ensure_bytes(subnet), _ensure_bytes(gateway),
                         persistent)

    def qsip_reboot(self, target: BrowseRespItem,
                     delay_ms: int = 100):
        """重启 QSIP 节点。"""
        _call_int(self._controller_id, _lib.controller_qsip_reboot, self._cptr, ctypes.byref(target), delay_ms)

    # -------------------- QSIP 终端 --------------------

    def qsip_terminal_open(self, target: BrowseRespItem, cols: int = 80, rows: int = 24) -> int:
        """
        打开 QSIP 终端会话。
        @param target 目标设备信息
        @param cols 终端列数，默认 80
        @param rows 终端行数，默认 24
        @return 会话 ID
        """
        session_id = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_qsip_terminal_open, self._cptr, ctypes.byref(target),
                  ctypes.byref(session_id), cols, rows)
        return session_id.value

    def qsip_terminal_input(self, target: BrowseRespItem, session_id: int, input_cmd: str):
        """
        向 QSIP 终端发送命令。
        @param target 目标设备信息
        @param session_id 会话 ID
        @param input_cmd 要发送的命令
        """
        _call_int(self._controller_id, _lib.controller_qsip_terminal_input, self._cptr, ctypes.byref(target),
                  session_id, _ensure_bytes(input_cmd))

    def qsip_terminal_output(self, target: BrowseRespItem, session_id: int) -> str:
        """
        获取 QSIP 终端输出。
        @param target 目标设备信息
        @param session_id 会话 ID
        @return 终端输出内容
        """
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_qsip_terminal_output, self._cptr, ctypes.byref(target),
                  session_id, ctypes.byref(result_ptr))
        result_string = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return result_string

    def qsip_terminal_close(self, target: BrowseRespItem, session_id: int):
        """
        关闭 QSIP 终端会话。
        @param target 目标设备信息
        @param session_id 会话 ID
        """
        _call_int(self._controller_id, _lib.controller_qsip_terminal_close, self._cptr, ctypes.byref(target), session_id)

    # -------------------- QSIP 升级与复位 --------------------

    def qsip_clear_update_flags(self, target_ip: str = ""):
        """
        复位升级标志。在一键式升级失败需要复位的情况下使用。

        @param target_ip 指定机器人控制器的 IP 地址，缺省则使用 SDK 当前连接的 IP
        @return 执行成功（0），执行失败（异常代码）
        @note 正常升级过程中不建议使用。
        """
        _call_int(self._controller_id, _lib.controller_qsip_clear_update_flags,
                        self._cptr, target_ip.encode() if target_ip else None)

    def qsip_firmware_update(self, firmware_file: str, target_ip: str = "",
                              auto_merge: bool = True):
        """
        自动一键式升级（阻塞）。

        @param firmware_file 一键式升级的固件路径，文件扩展名为 zip
        @param target_ip 指定机器人控制器的 IP 地址，缺省则使用 SDK 当前连接的 IP
        @param auto_merge 是否自动整合原配置文件，默认为 true
        @return 执行成功（0），执行失败（异常代码）
        @note 执行时间较长（通常超过10分钟），为阻塞调用。
        """
        _call_int(self._controller_id, _lib.controller_qsip_firmware_update,
                          self._cptr, firmware_file.encode(),
                          target_ip.encode() if target_ip else None, auto_merge)

    def qsip_firmware_update_manual(self, step: int, target_ip: str = "",
                                     firmware_file: str = "", auto_merge: bool = True):
        """
        手动一键式升级（分步骤）。

        @param step 升级步骤（FirmwareUpdateStep.Upload/Upgrade/Backup）
        @param target_ip 指定机器人控制器的 IP 地址，缺省则使用 SDK 当前连接的 IP
        @param firmware_file 固件路径（Upload 步骤时使用）
        @param auto_merge 是否自动整合原配置文件（Upload 步骤时使用）
        @return 执行成功（0），执行失败（异常代码）
        @note Upload 和 Upgrade 完成后需要重启机器人控制器。
        """
        _call_int(self._controller_id, _lib.controller_qsip_firmware_update_manual,
                          self._cptr, step,
                          target_ip.encode() if target_ip else None,
                          firmware_file.encode() if firmware_file else None, auto_merge)

    def qsip_config_update_manual(self, config_type: int, step: int,
                                   param: str = "", target_ip: str = "",
                                   auto_merge: bool = True):
        """
        手动配置文件升级和操作（分步骤）。

        @param config_type 配置文件类型（ConfigType.System/Application）
        @param step 操作步骤（ConfigUpdateStep.Upload/Enable/Backup/Delete/Sync/Save）
        @param param 各步骤参数，详见 Controller.h qsip_config_update_manual 文档
        @param target_ip 指定机器人控制器的 IP 地址，缺省则使用 SDK 当前连接的 IP
        @param auto_merge 是否自动整合原配置文件
        @return 执行成功（0），执行失败（异常代码）
        """
        _call_int(self._controller_id, _lib.controller_qsip_config_update_manual,
                          self._cptr, config_type, step,
                          param.encode() if param else None,
                          target_ip.encode() if target_ip else None, auto_merge)

    def qsip_mtd_update_manual(self, step: int, param: str = "",
                                 target_ip: str = ""):
        """
        手动固化信息烧录和操作（分步骤）。

        @param step 操作步骤（MTDUpdateStep.Import/Export/Backup/Recover）
        @param param 各步骤参数，详见 Controller.h qsip_mtd_update_manual 文档
        @param target_ip 指定机器人控制器的 IP 地址，缺省则使用 SDK 当前连接的 IP
        @return 执行成功（0），执行失败（异常代码）
        """
        _call_int(self._controller_id, _lib.controller_qsip_mtd_update_manual,
                          self._cptr, step,
                          param.encode() if param else None,
                          target_ip.encode() if target_ip else None)

    # -------------------- I/O 强制 --------------------

    def set_io_force(self, index: int, force: int,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> int:
        """强制设置 I/O 状态。"""
        return _call_int(self._controller_id, _lib.controller_set_io_force,
                         self._cptr, index, force, timeout, handle)

    def get_io_force(self, index: int,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> int:
        """获取 I/O 强制状态。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_io_force,
                  self._cptr, index, ctypes.byref(result), timeout, handle)
        return result.value

    # -------------------- 全局变量 --------------------

    def set_gint(self, index: int, value: int,
                  timeout: int = MACRO_CMD_TIMEOUT,
                  handle: int = MACRO_CMD_HANDLE) -> int:
        """设置全局整数变量 GInt[index]。"""
        return _call_int(self._controller_id, _lib.controller_set_gint, self._cptr, index, value, timeout, handle)

    def get_gint(self, index: int,
                  timeout: int = MACRO_CMD_TIMEOUT,
                  handle: int = MACRO_CMD_HANDLE) -> int:
        """获取全局整数变量 GInt[index]。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_gint, self._cptr, index, ctypes.byref(result), timeout, handle)
        return result.value

    def set_gbool(self, index: int, value: bool,
                   timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> int:
        """设置全局布尔变量 GBool[index]。"""
        return _call_int(self._controller_id, _lib.controller_set_gbool, self._cptr, index, value, timeout, handle)

    def get_gbool(self, index: int,
                   timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> bool:
        """获取全局布尔变量 GBool[index]。"""
        result = ctypes.c_bool()
        _call_int(self._controller_id, _lib.controller_get_gbool, self._cptr, index, ctypes.byref(result), timeout, handle)
        return result.value

    def set_gdouble(self, index: int, value: float,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> int:
        """设置全局双精度变量 GDouble[index]。"""
        return _call_int(self._controller_id, _lib.controller_set_gdouble, self._cptr, index, value, timeout, handle)

    def get_gdouble(self, index: int,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> float:
        """获取全局双精度变量 GDouble[index]。"""
        result = ctypes.c_double()
        _call_int(self._controller_id, _lib.controller_get_gdouble, self._cptr, index, ctypes.byref(result), timeout, handle)
        return result.value

    def set_gstring(self, index: int, value: str,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> int:
        """设置全局字符串变量 GString[index]。"""
        return _call_int(self._controller_id, _lib.controller_set_gstring, self._cptr, index, _ensure_bytes(value), timeout, handle)

    def get_gstring(self, index: int,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> str:
        """获取全局字符串变量 GString[index]。"""
        result_ptr = ctypes.c_char_p()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_gstring,
                  self._cptr, index, ctypes.byref(result_ptr), ctypes.byref(result_count),
                  timeout, handle)
        result = _ensure_str(result_ptr.value) if result_ptr.value else ""
        _free_string(result_ptr)
        return result

    def get_global_variable(self, type_: int, start_index: int, count: int,
                             timeout: int = MACRO_CMD_TIMEOUT,
                             handle: int = MACRO_CMD_HANDLE) -> List[str]:
        """批量获取全局变量的值。"""
        result_ptr = ctypes.POINTER(ctypes.c_char_p)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_global_variable,
                  self._cptr, type_, start_index, count,
                  ctypes.byref(result_ptr), ctypes.byref(result_count),
                  timeout, handle)
        cnt = result_count.value
        result = [_ensure_str(result_ptr[i]) for i in range(cnt)]
        _free_string_array(result_ptr, cnt)
        return result

    # -------------------- 解决方案/项目 --------------------

    def create_solution(self, solution_name: str, template_path: str = "") -> int:
        """创建解决方案。"""
        return _call_int(self._controller_id, _lib.controller_create_solution,
                         self._cptr, _ensure_bytes(solution_name), _ensure_bytes(template_path))

    def delete_solution(self, solution_name: str) -> int:
        """删除解决方案。"""
        return _call_int(self._controller_id, _lib.controller_delete_solution, self._cptr, _ensure_bytes(solution_name))

    def rename_solution(self, old_name: str, new_name: str) -> int:
        """重命名解决方案。"""
        return _call_int(self._controller_id, _lib.controller_rename_solution,
                         self._cptr, _ensure_bytes(old_name), _ensure_bytes(new_name))

    def set_main_project(self, solution_name: str, project_name: str) -> int:
        """设置主工程。"""
        return _call_int(self._controller_id, _lib.controller_set_main_project,
                         self._cptr, _ensure_bytes(solution_name), _ensure_bytes(project_name))

    def create_project(self, solution_name: str, project_name: str,
                        template_path: str = "") -> int:
        """创建工程。"""
        return _call_int(self._controller_id, _lib.controller_create_project,
                         self._cptr, _ensure_bytes(solution_name), _ensure_bytes(project_name),
                         _ensure_bytes(template_path))

    def delete_project(self, solution_name: str, project_name: str) -> int:
        """删除工程。"""
        return _call_int(self._controller_id, _lib.controller_delete_project,
                         self._cptr, _ensure_bytes(solution_name), _ensure_bytes(project_name))

    def rename_project(self, solution_name: str, old_name: str, new_name: str) -> int:
        """重命名工程。"""
        return _call_int(self._controller_id, _lib.controller_rename_project,
                         self._cptr, _ensure_bytes(solution_name),
                         _ensure_bytes(old_name), _ensure_bytes(new_name))

    def set_main_file(self, solution_name: str, project_name: str, file_name: str) -> int:
        """设置主文件。"""
        return _call_int(self._controller_id, _lib.controller_set_main_file,
                         self._cptr, _ensure_bytes(solution_name), _ensure_bytes(project_name),
                         _ensure_bytes(file_name))

    def create_qrl_file(self, solution_name: str, project_name: str,
                         file_name: str) -> int:
        """创建 QRL 文件。"""
        return _call_int(self._controller_id, _lib.controller_create_qrl_file,
                         self._cptr, _ensure_bytes(solution_name), _ensure_bytes(project_name),
                         _ensure_bytes(file_name))

    def delete_qrl_file(self, solution_name: str, project_name: str,
                          file_name: str) -> int:
        """删除 QRL 文件。"""
        return _call_int(self._controller_id, _lib.controller_delete_qrl_file,
                         self._cptr, _ensure_bytes(solution_name), _ensure_bytes(project_name),
                         _ensure_bytes(file_name))

    def rename_qrl_file(self, solution_name: str, project_name: str,
                          old_name: str, new_name: str) -> int:
        """重命名 QRL 文件。"""
        return _call_int(self._controller_id, _lib.controller_rename_qrl_file,
                         self._cptr, _ensure_bytes(solution_name), _ensure_bytes(project_name),
                         _ensure_bytes(old_name), _ensure_bytes(new_name))

    def get_pallas_solution_list(self) -> list:
        """
        获取 Pallas 解决方案列表。

        @return SolutionInfo 列表
        """
        result_ptr = ctypes.POINTER(SolutionInfo)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_pallas_solution_list,
                  self._cptr, ctypes.byref(result_ptr), ctypes.byref(result_count))
        count = result_count.value
        solutions = []
        for i in range(count):
            # 创建结构体副本，避免悬空指针
            sol = SolutionInfo()
            # 复制固定长度数组字段
            sol.name = bytes(result_ptr[i].name)
            sol.path = bytes(result_ptr[i].path)
            sol.is_qblock = result_ptr[i].is_qblock
            sol.main_project = bytes(result_ptr[i].main_project)
            sol.project_count = result_ptr[i].project_count
            # 复制 projectNames 指针
            sol.projectNames = result_ptr[i].projectNames
            solutions.append(sol)
        _lib.free_solution_info(result_ptr, count)
        return solutions

    def get_pallas_solution(self, solution_path: str) -> tuple:
        """
        获取 Pallas 解决方案详情。

        @param solution_path 解决方案路径
        @return (is_qblock: bool, solution_name: str, main_project_name: str, projects: list[ProjectInfo])
        """
        is_qblock = ctypes.c_bool()
        solution_name_ptr = ctypes.c_char_p()
        main_project_name_ptr = ctypes.c_char_p()
        project_list_ptr = ctypes.POINTER(ProjectInfo)()
        project_count = ctypes.c_int()

        _call_int(self._controller_id, _lib.controller_get_pallas_solution,
                  self._cptr, _ensure_bytes(solution_path),
                  ctypes.byref(is_qblock),
                  ctypes.byref(solution_name_ptr),
                  ctypes.byref(main_project_name_ptr),
                  ctypes.byref(project_list_ptr),
                  ctypes.byref(project_count))

        solution_name = _ensure_str(solution_name_ptr.value) if solution_name_ptr.value else ""
        main_project_name = _ensure_str(main_project_name_ptr.value) if main_project_name_ptr.value else ""

        projects = []
        count = project_count.value
        for i in range(count):
            projects.append(project_list_ptr[i])

        _free_string(solution_name_ptr)
        _free_string(main_project_name_ptr)
        _lib.free_project_info(project_list_ptr, count)

        return is_qblock.value, solution_name, main_project_name, projects

    def get_pallas_project(self, project_path: str) -> tuple:
        """
        获取 Pallas 项目详情。

        @param project_path 项目路径
        @return (project_name: str, main_file: str, qlv_files: list[str], ql_files: list[str])
        """
        project_name_ptr = ctypes.c_char_p()
        main_file_ptr = ctypes.c_char_p()
        qlv_files_ptr = ctypes.POINTER(ctypes.c_char_p)()
        qlv_files_count = ctypes.c_int()
        ql_files_ptr = ctypes.POINTER(ctypes.c_char_p)()
        ql_files_count = ctypes.c_int()

        _call_int(self._controller_id, _lib.controller_get_pallas_project,
                  self._cptr, _ensure_bytes(project_path),
                  ctypes.byref(project_name_ptr),
                  ctypes.byref(main_file_ptr),
                  ctypes.byref(qlv_files_ptr),
                  ctypes.byref(qlv_files_count),
                  ctypes.byref(ql_files_ptr),
                  ctypes.byref(ql_files_count))

        project_name = _ensure_str(project_name_ptr.value) if project_name_ptr.value else ""
        main_file = _ensure_str(main_file_ptr.value) if main_file_ptr.value else ""

        qlv_files = []
        qlv_count = qlv_files_count.value
        for i in range(qlv_count):
            qlv_files.append(_ensure_str(qlv_files_ptr[i]))

        ql_files = []
        ql_count = ql_files_count.value
        for i in range(ql_count):
            ql_files.append(_ensure_str(ql_files_ptr[i]))

        _free_string(project_name_ptr)
        _free_string(main_file_ptr)
        _free_string_array(qlv_files_ptr, qlv_count)
        _free_string_array(ql_files_ptr, ql_count)

        return project_name, main_file, qlv_files, ql_files

    def get_file_content(self, file_path: str) -> str:
        """获取文件内容。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_get_file_content, self._cptr, _ensure_bytes(file_path),
                  ctypes.byref(result_ptr))
        result = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return result

    def save_file_content(self, file_path: str, content: str) -> int:
        """保存文件内容。"""
        return _call_int(self._controller_id, _lib.controller_save_file_content,
                         self._cptr, _ensure_bytes(file_path), _ensure_bytes(content))

    def download_folder(self, folder_path: str, local_path: str,
                         node_id: int = 1, core_id: int = 1) -> int:
        """下载整个文件夹。"""
        return _call_int(self._controller_id, _lib.controller_download_folder,
                         self._cptr, _ensure_bytes(folder_path), _ensure_bytes(local_path),
                         node_id, core_id)

    def upload_folder(self, folder_path: str, remote_path: str,
                       node_id: int = 1, core_id: int = 1) -> int:
        """上传整个文件夹。"""
        return _call_int(self._controller_id, _lib.controller_upload_folder,
                         self._cptr, _ensure_bytes(folder_path), _ensure_bytes(remote_path),
                         node_id, core_id)

    # -------------------- 回调设置 --------------------

    def set_hidden_callback(self, callback: Callable) -> int:
        """
        设置隐藏数据主动上报回调。

        @param callback 回调函数，签名为:
                     callback(data_type: int, robot_id: int, values: List[str]) -> None
        """
        wrapper = _make_hidden_callback_wrapper(callback)
        _python_callbacks["hidden"][id(self)] = wrapper
        return _call_int(self._controller_id, _lib.controller_set_hidden_callback, self._cptr, wrapper)

    def set_qrl_callback(self, callback: Callable) -> int:
        """
        设置 QRL 指令回调。

        @param callback 回调函数，签名为: callback(qrl_data: str) -> None
        """
        wrapper = _make_qrl_callback_wrapper(callback)
        _python_callbacks["qrl"][id(self)] = wrapper
        return _call_int(self._controller_id, _lib.controller_set_qrl_callback, self._cptr, wrapper)

    def set_async_command_callback(self, callback: Callable) -> int:
        """
        设置指令异步执行结果回调。

        @param callback 回调函数，签名为:
                     callback(handle: int, error_id: int, result: str) -> None
        """
        wrapper = _make_async_command_callback_wrapper(callback)
        _python_callbacks["async_command"][id(self)] = wrapper
        return _call_int(self._controller_id, _lib.controller_set_async_command_callback, self._cptr, wrapper)

    def set_file_progress_callback(self, callback: Callable) -> int:
        """
        设置文件传输进度回调。

        @param callback 回调函数，签名为: callback(filename: str, progress: int) -> None
        """
        wrapper = _make_file_progress_callback_wrapper(callback)
        _python_callbacks["file_progress"][id(self)] = wrapper
        return _call_int(self._controller_id, _lib.controller_set_file_progress_callback, self._cptr, wrapper)

    # -------------------- 线程属性 --------------------

    def add_thread_property(self, thread_name: str, property_: int,
                            timeout: int = 0,
                            handle: int = MACRO_CMD_HANDLE) -> int:
        """
        添加线程属性。

        @param thread_name 线程名称
        @param property_ ThreadProperty 枚举值（None/Selected/Noemgstop/SelectedNoemgstop）
        @param timeout 超时时间
        @param handle 命令句柄
        """
        return _call_int(self._controller_id, _lib.controller_add_thread_property,
                         self._cptr, _ensure_bytes(thread_name), property_, timeout, handle)

    def remove_thread_property(self, thread_name: str, property_: int,
                               timeout: int = 0,
                               handle: int = MACRO_CMD_HANDLE) -> int:
        """
        删除线程属性。

        @param thread_name 线程名称
        @param property_ ThreadProperty 枚举值
        @param timeout 超时时间
        @param handle 命令句柄
        """
        return _call_int(self._controller_id, _lib.controller_remove_thread_property,
                         self._cptr, _ensure_bytes(thread_name), property_, timeout, handle)

    def get_thread_property(self, thread_name: str,
                            timeout: int = 0,
                            handle: int = MACRO_CMD_HANDLE) -> int:
        """
        获取线程属性。

        @param thread_name 线程名称
        @param timeout 超时时间
        @param handle 命令句柄
        @return ThreadProperty 枚举值
        """
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_thread_property,
                  self._cptr, _ensure_bytes(thread_name), ctypes.byref(result),
                  timeout, handle)
        return result.value

    # -------------------- QLV 文件 --------------------

    def get_qlv_file_content(self, file_path: str) -> QLVFile:
        """
        获取 QLV 文件内容。

        @param file_path QLV 文件路径
        @return QLVFile 结构体（包含 locations、location_js、profiles 及其数组）
        """
        qlv_file = QLVFile()
        _call_int(self._controller_id, _lib.controller_get_qlv_file_content,
                  self._cptr, _ensure_bytes(file_path), ctypes.byref(qlv_file))
        return qlv_file

    def save_qlv_file_content(self, file_path: str, qlv_file: QLVFile) -> int:
        """
        保存内容到 QLV 文件。

        @param file_path QLV 文件路径
        @param qlv_file QLVFile 结构体
        @return 错误码，0 表示成功
        """
        return _call_int(self._controller_id, _lib.controller_save_qlv_file_content,
                          self._cptr, _ensure_bytes(file_path), ctypes.byref(qlv_file))

    # -------------------- Latch 配置 --------------------

    def get_latch_config_number(self, timeout: int = MACRO_CMD_TIMEOUT,
                                handle: int = MACRO_CMD_HANDLE) -> int:
        """获取 Latch 配置数量。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_latch_config_number,
                  self._cptr, ctypes.byref(result), timeout, handle)
        return result.value

    def get_latch_count(self, latch_id: int,
                       timeout: int = MACRO_CMD_TIMEOUT,
                       handle: int = MACRO_CMD_HANDLE) -> int:
        """
        获取 Latch 捕获次数。

        @param latch_id Latch 编号
        @param timeout 超时时间
        @param handle 命令句柄
        @return 捕获次数
        """
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_latch_count,
                  self._cptr, latch_id, ctypes.byref(result), timeout, handle)
        return result.value

    def get_latch_config(self, latch_id: int,
                         timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> LatchConfig:
        """
        获取 Latch 配置。

        @param latch_id Latch 编号
        @param timeout 超时时间
        @param handle 命令句柄
        @return LatchConfig 结构体
        """
        config = LatchConfig()
        _call_int(self._controller_id, _lib.controller_get_latch_config,
                  self._cptr, latch_id, ctypes.byref(config), timeout, handle)
        return config

    def set_latch_config(self, config: LatchConfig,
                         timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> int:
        """
        设置 Latch 配置。

        @param config LatchConfig 结构体
        @param timeout 超时时间
        @param handle 命令句柄
        @return 0 表示成功，负值表示错误码
        """
        return _call_int(self._controller_id, _lib.controller_set_latch_config,
                         self._cptr, ctypes.byref(config), timeout, handle)

    def set_latch_flush(self, latch_id: int,
                         timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> int:
        """
        刷新 Latch（清除 Latch 数据缓冲区）。

        @param latch_id Latch 编号
        @param timeout 超时时间
        @param handle 命令句柄
        """
        return _call_int(self._controller_id, _lib.controller_set_latch_flush,
                         self._cptr, latch_id, timeout, handle)

    def set_latch_delay_time(self, latch_id: int, delay_time: float,
                              timeout: int = MACRO_CMD_TIMEOUT,
                              handle: int = MACRO_CMD_HANDLE) -> int:
        """
        设置 Latch 延迟时间。

        @param latch_id Latch 编号
        @param delay_time 延迟时间（秒）
        @param timeout 超时时间
        @param handle 命令句柄
        """
        return _call_int(self._controller_id, _lib.controller_set_latch_delay_time,
                         self._cptr, latch_id, delay_time, timeout, handle)

    def get_latch_result(self, latch_id: int, index: int, mode: int, is_del: bool = False,
                         timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> LatchResult:
        """
        获取 Latch 捕获结果。

        @param latch_id Latch 编号
        @param index 结果索引
        @param mode 模式
        @param is_del 是否删除
        @param timeout 超时时间
        @param handle 命令句柄
        @return LatchResult 结构体
        """
        result = LatchResult()
        _call_int(self._controller_id, _lib.controller_get_latch_result,
                  self._cptr, latch_id, index, mode, is_del,
                  ctypes.byref(result), timeout, handle)
        return result

    def get_latch_id(self, target: int, src: int, trigger_cfg: int,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> int:
        """
        获取 Latch ID。

        @param target 目标值
        @param src 源值
        @param trigger_cfg 触发配置
        @param timeout 超时时间
        @param handle 命令句柄
        @return Latch ID
        """
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_get_latch_id,
                  self._cptr, target, src, trigger_cfg,
                  ctypes.byref(result), timeout, handle)
        return result.value

    # -------------------- Zone 干涉区 --------------------

    def get_zone_config(self, zone_id: int,
                        timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> ZoneConfig:
        """
        获取干涉区配置。

        @param zone_id 干涉区编号
        @param timeout 超时时间
        @param handle 命令句柄
        @return ZoneConfig 结构体
        """
        config = ZoneConfig()
        _call_int(self._controller_id, _lib.controller_get_zone_config,
                  self._cptr, zone_id, ctypes.byref(config), timeout, handle)
        return config

    def set_zone_config(self, config: ZoneConfig,
                        timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> int:
        """
        设置干涉区配置。

        @param config ZoneConfig 结构体
        @param timeout 超时时间
        @param handle 命令句柄
        @return 错误码，0 表示成功
        """
        return _call_int(self._controller_id, _lib.controller_set_zone_config,
                         self._cptr, ctypes.byref(config), timeout, handle)

    # -------------------- 网关获取 --------------------

    def get_gateway(self, timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> str:
        """获取网关地址。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_get_gateway, self._cptr, ctypes.byref(result_ptr), timeout, handle)
        gateway = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return gateway

    # -------------------- I/O 相关 --------------------

    def set_io_pulse(self, index: int, pulse_time: int,
                   timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> int:
        """设置 I/O 脉冲。"""
        return _call_int(self._controller_id, _lib.controller_set_io_pulse, self._cptr,
                         ctypes.c_int(int(index)), ctypes.c_int(int(pulse_time)),
                         ctypes.c_int(int(timeout)), ctypes.c_uint32(int(handle)))

    def set_io_clear_force(self, index: int,
                         timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> int:
        """清除 I/O 强制状态。"""
        return _call_int(self._controller_id, _lib.controller_set_io_clear_force, self._cptr, index, timeout, handle)

    def reset_io(self, index: int,
               timeout: int = MACRO_CMD_TIMEOUT,
               handle: int = MACRO_CMD_HANDLE) -> int:
        """重置 I/O。"""
        return _call_int(self._controller_id, _lib.controller_reset_io, self._cptr, index, timeout, handle)

    # -------------------- 全局变量 --------------------

    def set_glocation(self, index: int, location: Location,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> int:
        """设置全局 Location 变量。"""
        return _call_int(self._controller_id, _lib.controller_set_glocation, self._cptr, index, ctypes.byref(location), timeout, handle)

    def get_glocation(self, index: int,
                     timeout: int = MACRO_CMD_TIMEOUT,
                     handle: int = MACRO_CMD_HANDLE) -> Location:
        """获取全局 Location 变量。"""
        location = Location()
        _call_int(self._controller_id, _lib.controller_get_glocation, self._cptr, index, ctypes.byref(location), timeout, handle)
        return location

    def set_glocationj(self, index: int, locationj: LocationJ,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> int:
        """设置全局 LocationJ 变量。"""
        return _call_int(self._controller_id, _lib.controller_set_glocationj, self._cptr, index, ctypes.byref(locationj), timeout, handle)

    def get_glocationj(self, index: int,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> LocationJ:
        """获取全局 LocationJ 变量。"""
        locationj = LocationJ()
        _call_int(self._controller_id, _lib.controller_get_glocationj, self._cptr, index, ctypes.byref(locationj), timeout, handle)
        return locationj

    def set_gprofile(self, index: int, profile: Profile,
                    timeout: int = MACRO_CMD_TIMEOUT,
                    handle: int = MACRO_CMD_HANDLE) -> int:
        """设置全局 Profile 变量。"""
        return _call_int(self._controller_id, _lib.controller_set_gprofile, self._cptr, index, ctypes.byref(profile), timeout, handle)

    def get_gprofile(self, index: int,
                    timeout: int = MACRO_CMD_TIMEOUT,
                    handle: int = MACRO_CMD_HANDLE) -> Profile:
        """获取全局 Profile 变量。"""
        profile = Profile()
        _call_int(self._controller_id, _lib.controller_get_gprofile, self._cptr, index, ctypes.byref(profile), timeout, handle)
        return profile

    def set_gtool(self, index: int, tool: Tool,
                 timeout: int = MACRO_CMD_TIMEOUT,
                 handle: int = MACRO_CMD_HANDLE) -> int:
        """设置全局 Tool 变量。"""
        return _call_int(self._controller_id, _lib.controller_set_gtool, self._cptr, index, ctypes.byref(tool), timeout, handle)

    def get_gtool(self, index: int,
                 timeout: int = MACRO_CMD_TIMEOUT,
                 handle: int = MACRO_CMD_HANDLE) -> Tool:
        """获取全局 Tool 变量。"""
        tool = Tool()
        _call_int(self._controller_id, _lib.controller_get_gtool, self._cptr, index, ctypes.byref(tool), timeout, handle)
        tool.index = index
        return tool

    def set_guserframe(self, index: int, userframe: UserFrame,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> int:
        """设置全局 UserFrame 变量。"""
        return _call_int(self._controller_id, _lib.controller_set_guserframe, self._cptr, index, ctypes.byref(userframe), timeout, handle)

    def get_guserframe(self, index: int,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> UserFrame:
        """获取全局 UserFrame 变量。"""
        userframe = UserFrame()
        _call_int(self._controller_id, _lib.controller_get_guserframe, self._cptr, index, ctypes.byref(userframe), timeout, handle)
        userframe.index = index
        return userframe

    # -------------------- 网络服务 --------------------

    def net_server_start(self, server_name: str, port: int, as_hex: bool = False,
                        timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> int:
        """启动网络服务器。"""
        return _call_int(self._controller_id, _lib.controller_net_server_start, self._cptr, _ensure_bytes(server_name), port, as_hex, timeout, handle)

    def net_server_stop(self, server_name: str,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> int:
        """停止网络服务器。"""
        return _call_int(self._controller_id, _lib.controller_net_server_stop, self._cptr, _ensure_bytes(server_name), timeout, handle)

    def net_server_add_client(self, server_name: str, client_name: str, client_ip: str,
                            timeout: int = MACRO_CMD_TIMEOUT,
                            handle: int = MACRO_CMD_HANDLE) -> int:
        """添加网络服务客户端。"""
        return _call_int(self._controller_id, _lib.controller_net_server_add_client, self._cptr, _ensure_bytes(server_name), _ensure_bytes(client_name), _ensure_bytes(client_ip), timeout, handle)

    def net_server_send(self, server_name: str, client_name: str, data: str,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> int:
        """发送数据给网络服务。"""
        return _call_int(self._controller_id, _lib.controller_net_server_send, self._cptr, _ensure_bytes(server_name), _ensure_bytes(client_name), _ensure_bytes(data), timeout, handle)

    def net_server_receive(self, server_name: str, client_name: str, receive_timeout: int = 1000,
                         timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> str:
        """接收网络服务数据。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_net_server_receive, self._cptr, _ensure_bytes(server_name), _ensure_bytes(client_name), receive_timeout, ctypes.byref(result_ptr), timeout, handle)
        data = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return data

    def net_server_get_status(self, server_name: str, client_name: str,
                            timeout: int = MACRO_CMD_TIMEOUT,
                            handle: int = MACRO_CMD_HANDLE) -> int:
        """获取网络服务状态。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_net_server_get_status, self._cptr, _ensure_bytes(server_name), _ensure_bytes(client_name), ctypes.byref(result), timeout, handle)
        return result.value

    def net_client_connect(self, client_name: str, server_ip: str, port: int, as_hex: bool = False,
                         timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> int:
        """启动网络客户端。"""
        return _call_int(self._controller_id, _lib.controller_net_client_connect, self._cptr, _ensure_bytes(client_name), _ensure_bytes(server_ip), port, as_hex, timeout, handle)

    def net_client_disconnect(self, client_name: str,
                            timeout: int = MACRO_CMD_TIMEOUT,
                            handle: int = MACRO_CMD_HANDLE) -> int:
        """停止网络客户端。"""
        return _call_int(self._controller_id, _lib.controller_net_client_disconnect, self._cptr, _ensure_bytes(client_name), timeout, handle)

    def net_client_send(self, client_name: str, data: str,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> int:
        """发送数据给网络客户端。"""
        return _call_int(self._controller_id, _lib.controller_net_client_send, self._cptr, _ensure_bytes(client_name), _ensure_bytes(data), timeout, handle)

    def net_client_receive(self, client_name: str, receive_timeout: int = 1000,
                         timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> str:
        """接收网络客户端数据。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_net_client_receive, self._cptr, _ensure_bytes(client_name), receive_timeout, ctypes.byref(result_ptr), timeout, handle)
        data = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return data

    def net_udp_start(self, server_name: str, remote_ip: str, remote_port: int, local_port: int, as_hex: bool = False,
                    timeout: int = MACRO_CMD_TIMEOUT,
                    handle: int = MACRO_CMD_HANDLE) -> int:
        """启动 UDP 服务。"""
        return _call_int(self._controller_id, _lib.controller_net_udp_start, self._cptr, _ensure_bytes(server_name), _ensure_bytes(remote_ip), remote_port, local_port, as_hex, timeout, handle)

    def net_udp_stop(self, server_name: str,
                   timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> int:
        """停止 UDP 服务。"""
        return _call_int(self._controller_id, _lib.controller_net_udp_stop, self._cptr, _ensure_bytes(server_name), timeout, handle)

    def net_udp_receive(self, server_name: str, receive_timeout: int = 1000,
                      timeout: int = MACRO_CMD_TIMEOUT,
                      handle: int = MACRO_CMD_HANDLE) -> str:
        """接收 UDP 数据。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_net_udp_receive, self._cptr, _ensure_bytes(server_name), receive_timeout, ctypes.byref(result_ptr), timeout, handle)
        data = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return data

    def net_udp_send(self, server_name: str, data: str,
                   timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> int:
        """发送 UDP 数据。"""
        return _call_int(self._controller_id, _lib.controller_net_udp_send, self._cptr, _ensure_bytes(server_name), _ensure_bytes(data), timeout, handle)

    # -------------------- 串口服务 --------------------

    def com_open(self, port_name: str, port: int = 232, baud_rate: int = 115200, data_bits: int = 8, stop_bits: int = 1, parity: int = 0, as_hex: bool = False,
               timeout: int = MACRO_CMD_TIMEOUT,
               handle: int = MACRO_CMD_HANDLE) -> int:
        """打开串口。"""
        return _call_int(self._controller_id, _lib.controller_com_open, self._cptr, _ensure_bytes(port_name), port, baud_rate, data_bits, stop_bits, parity, as_hex, timeout, handle)

    def com_close(self, port_name: str,
                timeout: int = MACRO_CMD_TIMEOUT,
                handle: int = MACRO_CMD_HANDLE) -> int:
        """关闭串口。"""
        return _call_int(self._controller_id, _lib.controller_com_close, self._cptr, _ensure_bytes(port_name), timeout, handle)

    def com_send(self, port_name: str, data: str,
               timeout: int = MACRO_CMD_TIMEOUT,
               handle: int = MACRO_CMD_HANDLE) -> int:
        """发送串口数据。"""
        return _call_int(self._controller_id, _lib.controller_com_send, self._cptr, _ensure_bytes(port_name), _ensure_bytes(data), timeout, handle)

    def com_receive(self, port_name: str, receive_timeout: int = 1000,
                  timeout: int = MACRO_CMD_TIMEOUT,
                  handle: int = MACRO_CMD_HANDLE) -> str:
        """接收串口数据。"""
        result_ptr = ctypes.c_char_p()
        _call_int(self._controller_id, _lib.controller_com_receive, self._cptr, _ensure_bytes(port_name), receive_timeout, ctypes.byref(result_ptr), timeout, handle)
        data = _ensure_str(result_ptr.value)
        _free_string(result_ptr)
        return data

    def com_flush(self, port_name: str,
                 timeout: int = MACRO_CMD_TIMEOUT,
                 handle: int = MACRO_CMD_HANDLE) -> int:
        """清空串口缓冲区。"""
        return _call_int(self._controller_id, _lib.controller_com_flush, self._cptr, _ensure_bytes(port_name), timeout, handle)

    # -------------------- Modbus 服务 --------------------

    def modbus_open(self, modbus_name: str, ip: str,
                   timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> int:
        """启动 Modbus 主站。"""
        return _call_int(self._controller_id, _lib.controller_modbus_open, self._cptr, _ensure_bytes(modbus_name), _ensure_bytes(ip), timeout, handle)

    def modbus_close(self, modbus_name: str,
                   timeout: int = MACRO_CMD_TIMEOUT,
                   handle: int = MACRO_CMD_HANDLE) -> int:
        """停止 Modbus 主站。"""
        return _call_int(self._controller_id, _lib.controller_modbus_close, self._cptr, _ensure_bytes(modbus_name), timeout, handle)

    def modbus_read_coils(self, modbus_name: str, start: int, number: int,
                        timeout: int = MACRO_CMD_TIMEOUT,
                        handle: int = MACRO_CMD_HANDLE) -> list[bool]:
        """读取 Modbus 线圈。"""
        result_ptr = ctypes.POINTER(ctypes.c_bool)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_modbus_read_coils, self._cptr, _ensure_bytes(modbus_name), start, number,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_bool_array(result_ptr)
        return result

    def modbus_read_discrete_inputs(self, modbus_name: str, start: int, number: int,
                                  timeout: int = MACRO_CMD_TIMEOUT,
                                  handle: int = MACRO_CMD_HANDLE) -> list[bool]:
        """读取 Modbus 离散输入。"""
        result_ptr = ctypes.POINTER(ctypes.c_bool)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_modbus_read_discrete_inputs, self._cptr, _ensure_bytes(modbus_name), start, number,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _free_bool_array(result_ptr)
        return result

    def modbus_read_holding_registers(self, modbus_name: str, start: int, number: int, dword: int = 0,
                                    timeout: int = MACRO_CMD_TIMEOUT,
                                    handle: int = MACRO_CMD_HANDLE) -> list[int]:
        """读取 Modbus 保持寄存器。"""
        result_ptr = ctypes.POINTER(ctypes.c_int)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_modbus_read_holding_registers, self._cptr, _ensure_bytes(modbus_name), start, number, dword,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _lib.free_int_array(result_ptr)
        return result

    def modbus_read_input_registers(self, modbus_name: str, start: int, number: int, dword: int = 0,
                                   timeout: int = MACRO_CMD_TIMEOUT,
                                   handle: int = MACRO_CMD_HANDLE) -> list[int]:
        """读取 Modbus 输入寄存器。"""
        result_ptr = ctypes.POINTER(ctypes.c_int)()
        result_count = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_modbus_read_input_registers, self._cptr, _ensure_bytes(modbus_name), start, number, dword,
                  ctypes.byref(result_ptr), ctypes.byref(result_count), timeout, handle)
        count = result_count.value
        result = [result_ptr[i] for i in range(count)]
        _lib.free_int_array(result_ptr)
        return result

    def modbus_write_multiple_coils(self, modbus_name: str, start: int, values: list[bool],
                                   timeout: int = MACRO_CMD_TIMEOUT,
                                   handle: int = MACRO_CMD_HANDLE) -> int:
        """写入多个 Modbus 线圈。"""
        values_array = (ctypes.c_bool * len(values))(*values)
        return _call_int(self._controller_id, _lib.controller_modbus_write_multiple_coils, self._cptr, _ensure_bytes(modbus_name), start, values_array, len(values), timeout, handle)

    def modbus_write_multiple_registers(self, modbus_name: str, start: int, values: list[int], dword: int = 0,
                                      timeout: int = MACRO_CMD_TIMEOUT,
                                      handle: int = MACRO_CMD_HANDLE) -> int:
        """写入多个 Modbus 寄存器。"""
        values_array = (ctypes.c_int * len(values))(*values)
        return _call_int(self._controller_id, _lib.controller_modbus_write_multiple_registers, self._cptr, _ensure_bytes(modbus_name), start, values_array, len(values), dword, timeout, handle)

    def modbus_write_single_coil(self, modbus_name: str, address: int, value: bool,
                               timeout: int = MACRO_CMD_TIMEOUT,
                               handle: int = MACRO_CMD_HANDLE) -> int:
        """写入单个 Modbus 线圈。"""
        return _call_int(self._controller_id, _lib.controller_modbus_write_single_coil, self._cptr, _ensure_bytes(modbus_name), address, value, timeout, handle)

    def modbus_write_single_register(self, modbus_name: str, address: int, value: int, dword: int = 0,
                                   timeout: int = MACRO_CMD_TIMEOUT,
                                   handle: int = MACRO_CMD_HANDLE) -> int:
        """写入单个 Modbus 寄存器。"""
        return _call_int(self._controller_id, _lib.controller_modbus_write_single_register, self._cptr, _ensure_bytes(modbus_name), address, value, dword, timeout, handle)

    def modbus_set_timeout(self, modbus_name: str, modbus_timeout: int,
                         timeout: int = MACRO_CMD_TIMEOUT,
                         handle: int = MACRO_CMD_HANDLE) -> int:
        """设置 Modbus 超时时间。"""
        return _call_int(self._controller_id, _lib.controller_modbus_set_timeout, self._cptr, _ensure_bytes(modbus_name), modbus_timeout, timeout, handle)

    def modbus_get_timeout(self, modbus_name: str,
                          timeout: int = MACRO_CMD_TIMEOUT,
                          handle: int = MACRO_CMD_HANDLE) -> int:
        """获取 Modbus 超时时间。"""
        result = ctypes.c_int()
        _call_int(self._controller_id, _lib.controller_modbus_get_timeout, self._cptr, _ensure_bytes(modbus_name), ctypes.byref(result), timeout, handle)
        return result.value
