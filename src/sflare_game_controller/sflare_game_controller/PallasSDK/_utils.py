"""
_utils.py

Created on: 2026-4-16
Author: 0590

通用辅助函数，供其他模块使用。
"""

import ctypes

# ---------------------------------------------------------------------------
# 默认参数
# ---------------------------------------------------------------------------
MACRO_CMD_TIMEOUT = 5000
MACRO_MOVE_CMD_TIMEOUT = 60000
MACRO_CMD_HANDLE = 0

# ---------------------------------------------------------------------------
# 错误码定义
# ---------------------------------------------------------------------------
ERROR_COMMAND_PARAMETER_INVALID = -20305
ERROR_COMMAND_VALUE_INVALID = -20306

# ---------------------------------------------------------------------------
# 内部异常类
# ---------------------------------------------------------------------------

class PallasException(Exception):
    """RobotSDK 异常，对应 C++ PallasException"""
    def __init__(self, controller_id: int, code: int, message: str = "",
                 file: str = "", line: int = 0, function: str = ""):
        from ._enums import LogLevel
        self.controller_id = controller_id
        self.level = LogLevel.Error
        self.code = code
        self.message = message
        self.file = file
        self.line = line
        self.function = function
        self.code_message = f"{code} {message}"
        self.full_message = f"[Error] {self.code_message}"
        super().__init__(self.full_message)
        # 写入日志（调用 C++ Logger）
        self._log_exception()

    def _log_exception(self):
        """通过 C ABI 接口调用 C++ Logger 记录异常日志"""
        try:
            from ._lib import _lib
            # 将字符串转换为 bytes
            file_bytes = self.file.encode('utf-8') if self.file else b""
            func_bytes = self.function.encode('utf-8') if self.function else b""
            msg_bytes = self.code_message.encode('utf-8')
            # 调用 C++ Logger
            _lib.logger_log(self.controller_id, self.level, file_bytes, self.line, func_bytes, msg_bytes)
        except Exception:
            # 如果日志写入失败，不影响异常抛出
            print(f"Failed to write log: {self.full_message}")
            pass

    def __repr__(self):
        return f"PallasException(code={self.code}, message='{self.message}', file='{self.file}:{self.line}')"

    def get_id(self) -> int:
        """获取错误码（对应 C++ get_id()）"""
        return self.code

    def get_message(self) -> str:
        """获取错误消息（对应 C++ get_message()）"""
        return self.message

    def get_file(self) -> str:
        """获取异常发生的文件（对应 C++ get_file()）"""
        return self.file

    def get_line(self) -> int:
        """获取异常发生的行号（对应 C++ get_line()）"""
        return self.line

    def get_function(self) -> str:
        """获取异常发生的函数名（对应 C++ get_function()）"""
        return self.function

    def what(self) -> str:
        """获取完整异常消息（对应 C++ what()）"""
        return self.full_message


# ---------------------------------------------------------------------------
# 辅助函数
# ---------------------------------------------------------------------------

def _call_int(controller_id: int, lib_func, *args) -> int:
    """调用返回 int 的 C 函数，检查错误码。"""
    result = lib_func(*args)
    if result < 0:
        import inspect
        frame = inspect.currentframe()
        caller_frame = frame.f_back.f_back if frame and frame.f_back else None
        file = caller_frame.f_code.co_filename if caller_frame else ""
        line = caller_frame.f_lineno if caller_frame else 0
        function = caller_frame.f_code.co_name if caller_frame else ""
        raise PallasException(controller_id, result, "", file, line, function)
    return result

def _call_int_no_ctrl(lib_func, *args) -> int:
    """调用返回 int 的 C 函数（不需要 controller），检查错误码。"""
    result = lib_func(*args)
    if result < 0:
        import inspect
        frame = inspect.currentframe()
        caller_frame = frame.f_back.f_back if frame and frame.f_back else None
        file = caller_frame.f_code.co_filename if caller_frame else ""
        line = caller_frame.f_lineno if caller_frame else 0
        function = caller_frame.f_code.co_name if caller_frame else ""
        raise PallasException(0, result, "", file, line, function)
    return result

def _check_parameter(controller_id: int, message):
    """统一错误检查参数是否有效。"""
    import inspect
    frame = inspect.currentframe()
    caller_frame = frame.f_back.f_back if frame and frame.f_back else None
    file = caller_frame.f_code.co_filename if caller_frame else ""
    line = caller_frame.f_lineno if caller_frame else 0
    function = caller_frame.f_code.co_name if caller_frame else ""
    raise PallasException(controller_id, ERROR_COMMAND_PARAMETER_INVALID, message, file, line, function)

def _ensure_bytes(s) -> bytes:
    """确保返回 bytes 类型。"""
    if isinstance(s, bytes):
        return s
    if isinstance(s, str):
        return s.encode("utf-8")
    return s

def _ensure_str(b) -> str:
    """确保返回 str 类型。"""
    if isinstance(b, str):
        return b
    if isinstance(b, bytes):
        return b.decode("utf-8", errors="ignore")
    return str(b)

def _free_double_array(ptr):
    """释放 double 数组内存。"""
    from ._lib import _lib  # lazy import，避免循环
    if hasattr(_lib, 'free_double_array') and ptr:
        _lib.free_double_array(ptr)

def _free_int_array(ptr):
    """释放 int 数组内存。"""
    from ._lib import _lib  # lazy import，避免循环
    if hasattr(_lib, 'free_int_array') and ptr:
        _lib.free_int_array(ptr)

def _free_bool_array(ptr):
    """释放 bool 数组内存。"""
    from ._lib import _lib  # lazy import，避免循环
    if hasattr(_lib, 'free_bool_array') and ptr:
        _lib.free_bool_array(ptr)

def _free_string(ptr):
    """释放字符串内存。"""
    from ._lib import _lib  # lazy import，避免循环
    if hasattr(_lib, 'free_string') and ptr:
        _lib.free_string(ptr)

def _free_string_array(ptr, count):
    """释放字符串数组内存。"""
    from ._lib import _lib  # lazy import，避免循环
    if hasattr(_lib, 'free_string_array') and ptr:
        _lib.free_string_array(ptr, count)

def _copy_list_to_ctypes_array(ctypes_array, source_list):
    """
    将 source_list 中的元素复制到 ctypes 数组 ctypes_array 中，
    只复制 min(len(source_list), 数组长度) 个元素，不修改数组剩余元素。

    :param ctypes_array: ctypes 数组对象（如 ctypes.c_double * 6）
    :param source_list: 可迭代对象，元素类型需与数组兼容
    """
    n = min(len(source_list), ctypes_array._length_)
    for i in range(n):
        ctypes_array[i] = source_list[i]