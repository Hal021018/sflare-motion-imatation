"""
_types.py

Created on: 2026-4-13
Author: 0590

与 RobotTypesExportC.h / RobotProtocol.h 对应的 ctypes 结构体定义。
供 Python ctypes 封装层使用，不依赖任何特定 Python 版本。

字段名与 bind_UserData.h（pybind11 接口）保持一致，
字段顺序与 RobotTypesExportC.h 中 C struct 内存布局严格对应。
"""

import ctypes
from typing import TYPE_CHECKING
from ._utils import _call_int, _check_parameter, _copy_list_to_ctypes_array, _ensure_bytes, _ensure_str

if TYPE_CHECKING:
    # 仅用于类型提示，运行时不导入（避免循环依赖）
    from ._lib import _lib as _robot_lib

# ---------------------------------------------------------------------------
# 可变数组包装类
# ---------------------------------------------------------------------------

class MutableArrayWrapper:
    """可变数组包装类，支持索引访问和修改。"""
    def __init__(self, ctypes_array):
        self._ctypes_array = ctypes_array
        self._length = ctypes_array._length_

    def __getitem__(self, idx):
        if not (0 <= idx < self._length):
            _check_parameter(0, f"index {idx} out of range [0, {self._length - 1}]")
        return self._ctypes_array[idx]

    def __setitem__(self, idx, val):
        if not (0 <= idx < self._length):
            _check_parameter(0, f"index {idx} out of range [0, {self._length - 1}]")
        self._ctypes_array[idx] = val

    def __len__(self):
        return self._length

    def __iter__(self):
        return iter(self._ctypes_array[i] for i in range(self._length))

    def __repr__(self):
        return f"[{', '.join(str(self[i]) for i in range(self._length))}]"

    def to_list(self):
        return [self[i] for i in range(self._length)]

# ---------------------------------------------------------------------------
# 模块级辅助函数：将任意坐标系对象转为 Frame
# 支持 Frame / Tool / UserFrame（直接 cast）以及 Location（字段拷贝）
# ---------------------------------------------------------------------------

def _to_frame(obj):
    """
    将任意坐标系对象转为 Frame。
    - Frame / Tool / UserFrame：直接拷贝前6字段
    - Location：调用其 _to_frame() 拷贝前6字段
    """
    if isinstance(obj, Frame):
        return Frame(obj.x, obj.y, obj.z, obj.yaw, obj.pitch, obj.roll)
    elif isinstance(obj, Tool):
        return Frame(obj.x, obj.y, obj.z, obj.yaw, obj.pitch, obj.roll)
    elif isinstance(obj, UserFrame):
        return Frame(obj.x, obj.y, obj.z, obj.yaw, obj.pitch, obj.roll)
    elif isinstance(obj, Location):
        return obj._to_frame()
    else:
        _check_parameter(0, f"Must be Frame, Tool, UserFrame or Location, got {type(obj).__name__}")

# ---------------------------------------------------------------------------
# 欧拉角类型枚举
# ---------------------------------------------------------------------------

class EulerType:
    """欧拉角类型枚举。"""
    ZYZ = 0    # Z-Y-Z 欧拉角
    XYZ = 1    # X-Y-Z 欧拉角
    ZYX = 2    # Z-Y-X 欧拉角
    FixAngle = 3    # 固定角

# ---------------------------------------------------------------------------
# 基础坐标结构体
# ---------------------------------------------------------------------------

class Frame(ctypes.Structure):
    """笛卡尔坐标系。"""
    _fields_ = [
        ("x",     ctypes.c_double),
        ("y",     ctypes.c_double),
        ("z",     ctypes.c_double),
        ("yaw",   ctypes.c_double),
        ("pitch", ctypes.c_double),
        ("roll",  ctypes.c_double),
    ]

    def __repr__(self):
        return (f"{self.x:.3f},{self.y:.3f},{self.z:.3f},"
                f"{self.yaw:.3f},{self.pitch:.3f},{self.roll:.3f}")

    def toString(self):
        return self.__repr__()

    def toList(self):
        return [self.x, self.y, self.z, self.yaw, self.pitch, self.roll]

    @classmethod
    def from_list(cls, lst):
        vals = list(lst) + [0.0] * (6 - len(lst))
        return cls(*vals[:6])

    def _get_lib(self):
        """延迟获取共享库实例，避免循环导入。"""
        from . import _lib as _lib_module
        return _lib_module._lib

    def inv(self) -> "Frame":
        """
        计算坐标系的逆变换。
        @return 逆变换后的 Frame
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        result = Frame()
        _call_int(lib.utils_frame_inv, ctypes.byref(self), ctypes.byref(result))
        return result

    def mul(self, other) -> "Frame":
        """
        计算两个坐标系的乘积（复合变换）。
        @param other 右侧坐标系（Frame / Tool / UserFrame / Location）
        @return 复合变换后的 Frame
        @raise TypeError  若 other 类型不兼容
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        result = Frame()
        tmp_other = _to_frame(other)  # 保存临时对象引用
        _call_int(lib.utils_frame_mul,
                    ctypes.cast(ctypes.byref(self), ctypes.POINTER(Frame)),
                    ctypes.cast(ctypes.byref(tmp_other), ctypes.POINTER(Frame)),
                    ctypes.byref(result))
        return result

    def dist(self, other) -> float:
        """
        计算两个坐标系之间的距离。
        @param other 目标坐标系（Frame / Tool / UserFrame / Location）
        @return 两坐标系之间的距离（浮点数）
        @raise TypeError  若 other 类型不兼容
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        result = ctypes.c_double(0.0)
        tmp_other = _to_frame(other)  # 保存临时对象引用
        _call_int(lib.utils_frame_dist,
                    ctypes.cast(ctypes.byref(self), ctypes.POINTER(Frame)),
                    ctypes.cast(ctypes.byref(tmp_other), ctypes.POINTER(Frame)),
                    ctypes.byref(result))
        return result.value

    def euler_transfer(self, euler_in: int, euler_out: int) -> "Frame":
        """
        欧拉角类型转换。
        @param euler_in  输入欧拉角类型（EulerType 枚举值）
        @param euler_out 输出欧拉角类型（EulerType 枚举值）
        @return 转换后的 Frame
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        result = Frame()
        _call_int(lib.utils_frame_euler_transfer,
                  ctypes.byref(self), ctypes.c_int(euler_in),
                  ctypes.c_int(euler_out), ctypes.byref(result))
        return result


class Tool(ctypes.Structure):
    """工具坐标系。"""
    _fields_ = [
        ("x",     ctypes.c_double),
        ("y",     ctypes.c_double),
        ("z",     ctypes.c_double),
        ("yaw",   ctypes.c_double),
        ("pitch", ctypes.c_double),
        ("roll",  ctypes.c_double),
    ]

    # Python-only attribute (not in C struct layout) — populated by Robot.get_tool()
    index: int = 0

    def __repr__(self):
        return (f"{self.x:.3f},{self.y:.3f},{self.z:.3f},"
                f"{self.yaw:.3f},{self.pitch:.3f},{self.roll:.3f}")

    def toString(self):
        return self.__repr__()

    def toList(self):
        return [self.x, self.y, self.z, self.yaw, self.pitch, self.roll]

    @classmethod
    def from_list(cls, lst):
        vals = list(lst) + [0.0] * (6 - len(lst))
        return cls(*vals[:6])

    def _get_lib(self):
        """延迟获取共享库实例，避免循环导入。"""
        from . import _lib as _lib_module
        return _lib_module._lib

    def _as_frame_ptr(self):
        """将自身内存地址强制转换为 Frame 指针（内存布局相同）。"""
        return ctypes.cast(ctypes.byref(self), ctypes.POINTER(Frame))

    def inv(self) -> "Frame":
        """
        计算坐标系的逆变换。
        @return 逆变换后的 Frame
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        result = Frame()
        _call_int(lib.utils_frame_inv, self._as_frame_ptr(), ctypes.byref(result))
        return result

    def mul(self, other) -> "Frame":
        """
        计算两个坐标系的乘积（复合变换）。
        @param other 右侧坐标系（Frame / Tool / UserFrame / Location）
        @return 复合变换后的 Frame
        @raise TypeError  若 other 类型不兼容
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        tmp_other = _to_frame(other)  # 保存临时对象引用
        result = Frame()
        _call_int(lib.utils_frame_mul, self._as_frame_ptr(),
                  ctypes.cast(ctypes.byref(tmp_other), ctypes.POINTER(Frame)),
                  ctypes.byref(result))
        return result

    def dist(self, other) -> float:
        """
        计算两个坐标系之间的距离。
        @param other 目标坐标系（Frame / Tool / UserFrame / Location）
        @return 距离（浮点数）
        @raise TypeError  若 other 类型不兼容
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        tmp_other = _to_frame(other)  # 保存临时对象引用
        result = ctypes.c_double(0.0)
        _call_int(lib.utils_frame_dist, self._as_frame_ptr(),
                  ctypes.cast(ctypes.byref(tmp_other), ctypes.POINTER(Frame)),
                  ctypes.byref(result))
        return result.value

    def euler_transfer(self, euler_in: int, euler_out: int) -> "Frame":
        """
        欧拉角类型转换。
        @param euler_in  输入欧拉角类型（EulerType 枚举值）
        @param euler_out 输出欧拉角类型（EulerType 枚举值）
        @return 转换后的 Frame
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        result = Frame()
        _call_int(lib.utils_frame_euler_transfer,
                  self._as_frame_ptr(), ctypes.c_int(euler_in),
                  ctypes.c_int(euler_out), ctypes.byref(result))
        return result

    def calib(self, points) -> tuple:
        """
        通过多个示教点标定工具坐标系。
        @param points  示教点列表，元素为 Frame（或内存布局兼容类型）
        @return (result: int, calibError: float)
                result 为 0 表示成功；calibError 为标定误差
        @raise ValueError      若 points 为空
        @raise TypeError       若 points 中元素类型不兼容
        @raise PallasException  若 C 接口调用失败
        """
        if not points:
            _check_parameter("calib requires at least one point")
        for i, p in enumerate(points):
            if not isinstance(p, (Frame, Tool, UserFrame, Location)):
                _check_parameter(f"calib requires points[{i}] must be Frame or Tool or UserFrame or Location")
        lib = self._get_lib()
        n = len(points)
        FrameArray = Frame * n
        arr = FrameArray()
        for i, p in enumerate(points):
            arr[i].x     = p.x
            arr[i].y     = p.y
            arr[i].z     = p.z
            arr[i].yaw   = p.yaw
            arr[i].pitch = p.pitch
            arr[i].roll  = p.roll
        calib_error = ctypes.c_double(0.0)
        result = _call_int(lib.utils_tool_calib, arr, ctypes.c_int(n), self._as_frame_ptr(), ctypes.byref(calib_error))
        return result, calib_error.value


class UserFrame(ctypes.Structure):
    """用户坐标系。"""
    _fields_ = [
        ("x",     ctypes.c_double),
        ("y",     ctypes.c_double),
        ("z",     ctypes.c_double),
        ("yaw",   ctypes.c_double),
        ("pitch", ctypes.c_double),
        ("roll",  ctypes.c_double),
    ]

    # Python-only attribute (not in C struct layout) — populated by Robot.get_userframe()
    index: int = 0

    def __repr__(self):
        return (f"{self.x:.3f},{self.y:.3f},{self.z:.3f},"
                f"{self.yaw:.3f},{self.pitch:.3f},{self.roll:.3f}")

    def toString(self):
        return self.__repr__()

    def toList(self):
        return [self.x, self.y, self.z, self.yaw, self.pitch, self.roll]

    @classmethod
    def from_list(cls, lst):
        vals = list(lst) + [0.0] * (6 - len(lst))
        return cls(*vals[:6])

    def _get_lib(self):
        """延迟获取共享库实例，避免循环导入。"""
        from . import _lib as _lib_module
        return _lib_module._lib

    def _as_frame_ptr(self):
        """将自身内存地址强制转换为 Frame 指针（内存布局相同）。"""
        return ctypes.cast(ctypes.byref(self), ctypes.POINTER(Frame))

    def inv(self) -> "Frame":
        """
        计算坐标系的逆变换。
        @return 逆变换后的 Frame
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        result = Frame()
        _call_int(lib.utils_frame_inv, self._as_frame_ptr(), ctypes.byref(result))
        return result

    def mul(self, other) -> "Frame":
        """
        计算两个坐标系的乘积（复合变换）。
        @param other 右侧坐标系（Frame / Tool / UserFrame / Location）
        @return 复合变换后的 Frame
        @raise TypeError  若 other 类型不兼容
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        tmp_other = _to_frame(other)  # 保存临时对象引用
        result = Frame()
        _call_int(lib.utils_frame_mul, self._as_frame_ptr(),
                  ctypes.cast(ctypes.byref(tmp_other), ctypes.POINTER(Frame)),
                  ctypes.byref(result))
        return result

    def dist(self, other) -> float:
        """
        计算两个坐标系之间的距离。
        @param other 目标坐标系（Frame / Tool / UserFrame / Location）
        @return 距离（浮点数）
        @raise TypeError  若 other 类型不兼容
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        tmp_other = _to_frame(other)  # 保存临时对象引用
        result = ctypes.c_double(0.0)
        _call_int(lib.utils_frame_dist, self._as_frame_ptr(),
                  ctypes.cast(ctypes.byref(tmp_other), ctypes.POINTER(Frame)),
                  ctypes.byref(result))
        return result.value

    def euler_transfer(self, euler_in: int, euler_out: int) -> "Frame":
        """
        欧拉角类型转换。
        @param euler_in  输入欧拉角类型（EulerType 枚举值）
        @param euler_out 输出欧拉角类型（EulerType 枚举值）
        @return 转换后的 Frame
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        result = Frame()
        _call_int(lib.utils_frame_euler_transfer, self._as_frame_ptr(), ctypes.c_int(euler_in),
                  ctypes.c_int(euler_out), ctypes.byref(result))
        return result

    def calib(self, p0, px, py) -> int:
        """
        通过三点法标定用户坐标系。
        @param p0  原点示教点（Frame 或内存布局兼容类型）
        @param px  X 轴方向示教点
        @param py  Y 轴方向示教点
        @return 整数错误码，0 表示成功
        @raise TypeError       若参数类型不兼容
        @raise PallasException  若 C 接口调用失败
        """
        for name, val in [("p0", p0), ("px", px), ("py", py)]:
            if not isinstance(val, (Frame, Tool, UserFrame, Location)):
                _check_parameter(f"calib requires {name} must be Frame or Tool or UserFrame or Location")
        lib = self._get_lib()
        p0_ptr = ctypes.cast(ctypes.byref(p0), ctypes.POINTER(Frame))
        px_ptr = ctypes.cast(ctypes.byref(px), ctypes.POINTER(Frame))
        py_ptr = ctypes.cast(ctypes.byref(py), ctypes.POINTER(Frame))
        result = _call_int(lib.utils_userframe_calib, p0_ptr, px_ptr, py_ptr, self._as_frame_ptr())
        return result


class Location(ctypes.Structure):
    """笛卡尔位置（含配置位和外部轴）。"""
    _fields_ = [
        ("x",      ctypes.c_double),
        ("y",      ctypes.c_double),
        ("z",      ctypes.c_double),
        ("yaw",    ctypes.c_double),
        ("pitch",  ctypes.c_double),
        ("roll",   ctypes.c_double),
        ("config", ctypes.c_int),
        ("ext1",   ctypes.c_double),
        ("ext2",   ctypes.c_double),
        ("ext3",   ctypes.c_double),
    ]

    def __repr__(self):
        return (f"{self.x:.3f},{self.y:.3f},{self.z:.3f},"
                f"{self.yaw:.3f},{self.pitch:.3f},{self.roll:.3f},"
                f"{self.config},{self.ext1:.3f},{self.ext2:.3f},{self.ext3:.3f}")

    def toString(self):
        return self.__repr__()

    def toList(self):
        return [self.x, self.y, self.z, self.yaw, self.pitch, self.roll,
                self.config, self.ext1, self.ext2, self.ext3]

    @classmethod
    def from_list(cls, lst):
        obj = cls()
        fields = ["x", "y", "z", "yaw", "pitch", "roll", "config", "ext1", "ext2", "ext3"]
        for i, f in enumerate(fields):
            if i < len(lst):
                setattr(obj, f, lst[i])
        return obj

    # ------------------------------------------------------------------
    # 以下方法将前 6 个 double 字段拷贝到临时 Frame，再调用 C 接口。
    # Location 的内存布局（config/ext1/ext2/ext3 在后）与 Frame 不完全
    # 一致，不能直接 cast，需显式拷贝。
    # ------------------------------------------------------------------

    def _to_frame(self):
        """将 Location 前6字段拷贝为临时 Frame（用于 C 接口调用）。"""
        return Frame(self.x, self.y, self.z, self.yaw, self.pitch, self.roll)

    def _get_lib(self):
        """延迟获取共享库实例，避免循环导入。"""
        from . import _lib as _lib_module
        return _lib_module._lib

    def inv(self) -> "Frame":
        """
        计算坐标系的逆变换。
        @return 逆变换后的 Frame
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        tmp = self._to_frame()
        result = Frame()
        _call_int(lib.utils_frame_inv, ctypes.byref(tmp), ctypes.byref(result))
        return result

    def mul(self, other) -> "Frame":
        """
        计算两个坐标系的乘积（复合变换）。
        @param other 右侧坐标系（Frame / Tool / UserFrame / Location）
        @return 复合变换后的 Frame
        @raise TypeError  若 other 类型不兼容
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        tmp_self = self._to_frame()
        tmp_other = _to_frame(other)
        result = Frame()
        _call_int(lib.utils_frame_mul,
                    ctypes.cast(ctypes.byref(tmp_self), ctypes.POINTER(Frame)),
                    ctypes.cast(ctypes.byref(tmp_other), ctypes.POINTER(Frame)),
                    ctypes.byref(result))
        return result

    def dist(self, other) -> float:
        """
        计算两个坐标系之间的距离。
        @param other 目标坐标系（Frame / Tool / UserFrame / Location）
        @return 两坐标系之间的距离（浮点数）
        @raise TypeError  若 other 类型不兼容
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        tmp_self = self._to_frame()
        tmp_other = _to_frame(other)
        result = ctypes.c_double(0.0)
        _call_int(lib.utils_frame_dist,
                    ctypes.cast(ctypes.byref(tmp_self), ctypes.POINTER(Frame)),
                    ctypes.cast(ctypes.byref(tmp_other), ctypes.POINTER(Frame)),
                    ctypes.byref(result))
        return result.value

    def euler_transfer(self, euler_in: int, euler_out: int) -> "Frame":
        """
        欧拉角类型转换。
        @param euler_in  输入欧拉角类型（EulerType 枚举值）
        @param euler_out 输出欧拉角类型（EulerType 枚举值）
        @return 转换后的 Frame
        @raise PallasException 若 C 接口调用失败
        """
        lib = self._get_lib()
        tmp = self._to_frame()
        result = Frame()
        _call_int(lib.utils_frame_euler_transfer,
                ctypes.cast(ctypes.byref(tmp), ctypes.POINTER(Frame)),
                ctypes.c_int(euler_in), ctypes.c_int(euler_out),
                ctypes.byref(result))
        return result


class LocationJ(ctypes.Structure):
    """轴空间坐标（最多12轴）。对应 LocationJ_C。"""
    _fields_ = [
        ("j1",  ctypes.c_double),
        ("j2",  ctypes.c_double),
        ("j3",  ctypes.c_double),
        ("j4",  ctypes.c_double),
        ("j5",  ctypes.c_double),
        ("j6",  ctypes.c_double),
        ("j7",  ctypes.c_double),
        ("j8",  ctypes.c_double),
        ("j9",  ctypes.c_double),
        ("j10", ctypes.c_double),
        ("j11", ctypes.c_double),
        ("j12", ctypes.c_double),
    ]

    def __repr__(self):
        vals = [str(getattr(self, f"j{i+1}")) for i in range(12)]
        return f"{','.join(vals)}"

    def toString(self):
        return self.__repr__()

    def __getitem__(self, idx):
        if not (0 <= idx < 12):
            raise IndexError(f"LocationJ index {idx} out of range [0, 11]")
        return getattr(self, f"j{idx+1}")

    def __setitem__(self, idx, val):
        if not (0 <= idx < 12):
            raise IndexError(f"LocationJ index {idx} out of range [0, 11]")
        setattr(self, f"j{idx+1}", val)

    def toList(self):
        return [getattr(self, f"j{i+1}") for i in range(12)]

    @classmethod
    def from_list(cls, lst):
        obj = cls()
        for i, v in enumerate(lst):
            if i >= 12:
                break
            setattr(obj, f"j{i+1}", v)
        return obj


# ---------------------------------------------------------------------------
# 运动参数结构体
# ---------------------------------------------------------------------------

class Profile(ctypes.Structure):
    """运动轨迹参数。"""
    _fields_ = [
        ("speed",       ctypes.c_double),  # 规划最大速度
        ("speed2",      ctypes.c_double),  # 特殊轴峰值运动速度（暂不支持）
        ("accel",       ctypes.c_double),  # 加速度
        ("decel",       ctypes.c_double),  # 减速度
        ("accelRamp",   ctypes.c_double),  # 加加速时间
        ("decelRamp",   ctypes.c_double),  # 减减速时间
        ("inRange",     ctypes.c_double),  # 到位逼近阈值（%）
        ("type",        ctypes.c_uint8),   # 速度类型（0: 相对, 1: 绝对）
        ("time",        ctypes.c_double),  # PTP/直线运动时间
        ("blend_type",  ctypes.c_uint8),   # 运动间拟合类型
        ("blend_value", ctypes.c_double),  # 拟合参数值
    ]

    def __repr__(self):
        return (f"{self.speed:.3f},{self.speed2:.3f},{self.accel:.3f},{self.decel:.3f},{self.accelRamp:.3f},{self.decelRamp:.3f},"
                f"{self.inRange:.3f},{self.type},{self.time:.3f},{self.blend_type},{self.blend_value:.3f}")

    def toString(self):
        return self.__repr__()

    def toList(self):
        return [self.speed, self.speed2, self.accel, self.decel, self.accelRamp,
                self.decelRamp, self.inRange, self.type, self.time,
                self.blend_type, self.blend_value]


class ForceModeParam(ctypes.Structure):
    """力控参数。"""
    _fields_ = [
        ("frame",     ctypes.c_int16),
        ("_selector", ctypes.c_char * 6),
        ("_wrench",   ctypes.c_double * 6),
        ("_limit",    ctypes.c_double * 6),
        ("source",    ctypes.c_int32),
        ("time",      ctypes.c_uint32),
        ("method",    ctypes.c_int16),
        ("distance",  ctypes.c_double),
    ]

    @property
    def selector(self):
        import ctypes
        addr = ctypes.addressof(self) + ctypes.sizeof(ctypes.c_int16)
        raw_bytes = ctypes.string_at(addr, 6)
        return [raw_bytes[i] != 0 for i in range(6)]

    @selector.setter
    def selector(self, value):
        result = bytearray(6)
        n = min(len(value), 6)
        for i in range(n):
            v = value[i]
            if isinstance(v, bytes):
                result[i] = 1 if v != b'\x00' else 0
            else:
                result[i] = 1 if v else 0
        self._selector = bytes(result)

    @property
    def wrench(self):
        return MutableArrayWrapper(self._wrench)

    @wrench.setter
    def wrench(self, value):
        _copy_list_to_ctypes_array(self._wrench, value)

    @property
    def limit(self):
        return MutableArrayWrapper(self._limit)

    @limit.setter
    def limit(self, value):
        _copy_list_to_ctypes_array(self._limit, value)


# ---------------------------------------------------------------------------
# NodeItem / BrowseRespItem 结构体
# ---------------------------------------------------------------------------

class NodeItem(ctypes.Structure):
    """网络节点信息。"""
    _fields_ = [
        ("node_id",    ctypes.c_uint16),
        ("node_type",  ctypes.c_uint16),
        ("group_id",   ctypes.c_uint16),
        ("version",    ctypes.c_uint16 * 3),
        ("_node_desc",  ctypes.c_char * 64),  # 新增字段
    ]

    def __repr__(self):
        ver = list(self.version)
        desc = self.node_desc.decode('utf-8').strip('\x00')
        return f"{self.node_id},{desc}({self.node_type}),{self.group_id},{','.join(ver)}"

    @property
    def node_desc(self):
        return _ensure_str(self._node_desc)

    @node_desc.setter
    def node_desc(self, value):
        self._node_desc = _ensure_bytes(value)


class BrowseRespItem(ctypes.Structure):
    """扫描结果——机器人控制器信息。"""
    _fields_ = [
        ("_identifier",  ctypes.c_char_p),
        ("_macAddress",  ctypes.c_char_p),
        ("_ipAddress",   ctypes.c_char_p),
        ("_subnet",      ctypes.c_char_p),
        ("_gateway",     ctypes.c_char_p),
        ("_modelName",   ctypes.c_char_p),
        ("nodeCount",    ctypes.c_int),
        ("_nodes",       ctypes.POINTER(NodeItem)),
    ]

    @property
    def identifier(self):
        return _ensure_str(self._identifier)

    @identifier.setter
    def identifier(self, value):
        self._identifier = _ensure_bytes(value)

    @property
    def macAddress(self):
        return _ensure_str(self._macAddress)

    @macAddress.setter
    def macAddress(self, value):
        self._macAddress = _ensure_bytes(value)

    @property
    def ipAddress(self):
        return _ensure_str(self._ipAddress)

    @ipAddress.setter
    def ipAddress(self, value):
        self._ipAddress = _ensure_bytes(value)

    @property
    def subnet(self):
        return _ensure_str(self._subnet)

    @subnet.setter
    def subnet(self, value):
        self._subnet = _ensure_bytes(value)

    @property
    def gateway(self):
        return _ensure_str(self._gateway)

    @gateway.setter
    def gateway(self, value):
        self._gateway = _ensure_bytes(value)

    @property
    def modelName(self):
        return _ensure_str(self._modelName)

    @modelName.setter
    def modelName(self, value):
        self._modelName = _ensure_bytes(value)

    @property
    def nodes(self):
        result = []
        nodes_ptr = self._nodes
        for i in range(self.nodeCount):
            result.append(nodes_ptr[i])
        return result

    @nodes.setter
    def nodes(self, value):
        self._nodes = value

    def __repr__(self):
        return (f"{self.identifier},{self.macAddress},{self.ipAddress},"
                f"{self.subnet},{self.gateway},{self.modelName},{self.nodeCount}")


# ---------------------------------------------------------------------------
# 运动触发器结构体
# ---------------------------------------------------------------------------

class Trigger(ctypes.Structure):
    """运动触发器。"""
    _fields_ = [
        ("type",      ctypes.c_int),       # TriggerType
        ("refPoint",  ctypes.c_int),       # TriggerRefPoint
        ("condition", ctypes.c_double),
        ("outType",   ctypes.c_int),       # TriggerOutType
        ("index",     ctypes.c_int),
        ("value",     ctypes.c_int),
        ("num",       ctypes.c_int),
        ("delayTime", ctypes.c_double),
    ]

    def __repr__(self):
        return (f"{self.type}, {self.refPoint},"
                f"{self.condition:.3f}, {self.outType},"
                f"{self.index}, {self.value}, {self.num},"
                f"{self.delayTime:.3f}")


# ---------------------------------------------------------------------------
# Latch 相关结构体
# ---------------------------------------------------------------------------

class LatchConfig(ctypes.Structure):
    """Latch 配置。"""
    _fields_ = [
        ("latchID",          ctypes.c_int),
        ("enable",           ctypes.c_bool),
        ("target",           ctypes.c_int),
        ("targetType",       ctypes.c_int),   # LatchTargetType
        ("targetRobotId",    ctypes.c_int),
        ("targetEncoderId",  ctypes.c_int),
        ("targetSiteId",     ctypes.c_int),
        ("src",              ctypes.c_int),
        ("triggerCfg",       ctypes.c_int),   # LatchTriggerCfgType
        ("delayTime",        ctypes.c_double),
        ("queueSize",        ctypes.c_int),
        ("alarmThreshold",   ctypes.c_int),
        ("conveyorID",       ctypes.c_int),
    ]


class LatchResult(ctypes.Structure):
    """Latch 结果。"""
    _fields_ = [
        ("timeStamp", ctypes.c_int),
        ("hitFlag",   ctypes.c_int),
        ("_position",  ctypes.c_double * 12),
    ]

    @property
    def position(self):
        return MutableArrayWrapper(self._position)

    @position.setter
    def position(self, value):
        _copy_list_to_ctypes_array(self._position, value)


# ---------------------------------------------------------------------------
# Zone 相关结构体
# ---------------------------------------------------------------------------

class ZoneConfig(ctypes.Structure):
    """干涉区配置。"""
    _fields_ = [
        ("zoneID",          ctypes.c_int),
        ("robotID",         ctypes.c_int),
        ("enable",          ctypes.c_int),    # ZoneEnable
        ("frameID",         ctypes.c_int),
        ("frame",           UserFrame),
        ("zoneType",        ctypes.c_int),    # ZoneType
        ("validAreaType",   ctypes.c_int),    # ValidAreaType
        ("checkPart",       ctypes.c_int),    # CheckPart
        ("enableMethod",    ctypes.c_int),    # EnableMethod
        ("DINIndex",        ctypes.c_int),
        ("outputSignal",    ctypes.c_int),
        ("DOUTIndex",       ctypes.c_int),
        ("_zoneParameter",  ctypes.c_double * 6),
        ("isEnabled",       ctypes.c_bool),
        ("robotInArea",     ctypes.c_bool),
        ("TCPInArea",       ctypes.c_bool),
        ("ArmInArea",       ctypes.c_bool),
        ("ElbowInArea",     ctypes.c_bool),
        ("diType",          ctypes.c_int),
        ("doType",          ctypes.c_int),
        ("controlMode",     ctypes.c_int),
    ]

    @property
    def zoneParameter(self):
        return MutableArrayWrapper(self._zoneParameter)

    @zoneParameter.setter
    def zoneParameter(self, value):
        _copy_list_to_ctypes_array(self._zoneParameter, value)


# ---------------------------------------------------------------------------
# EMI 反馈数据（对应 RobotProtocol.h::EMIFeedbackData）
# ---------------------------------------------------------------------------

class EMIFeedbackData(ctypes.Structure):
    """EMI 反馈数据包。"""
    _pack_ = 4  # 对应 C++ #pragma pack(push, 4)
    _fields_ = [
        ("robot_id",               ctypes.c_uint8),
        ("robot_speed",            ctypes.c_double),
        ("_robot_joint_pos",       ctypes.c_double * 12),
        ("_robot_joint_speed",     ctypes.c_double * 12),
        ("_robot_cartesian_speed", ctypes.c_double * 6),
        ("_robot_tool",            ctypes.c_double * 6),
        ("_robot_cartesian_pos",   ctypes.c_double * 6),
        ("robot_cartesian_config", ctypes.c_uint16),
        ("robot_mode",             ctypes.c_int32),
        ("error_code",             ctypes.c_int32),
        ("robot_state",            ctypes.c_uint32),
        ("reserved",               ctypes.c_uint16 * 32),
    ]

    @property
    def robot_joint_pos(self):
        return MutableArrayWrapper(self._robot_joint_pos)

    @robot_joint_pos.setter
    def robot_joint_pos(self, value):
        _copy_list_to_ctypes_array(self._robot_joint_pos, value)

    @property
    def robot_joint_speed(self):
        return MutableArrayWrapper(self._robot_joint_speed)

    @robot_joint_speed.setter
    def robot_joint_speed(self, value):
        _copy_list_to_ctypes_array(self._robot_joint_speed, value)

    @property
    def robot_cartesian_speed(self):
        return MutableArrayWrapper(self._robot_cartesian_speed)

    @robot_cartesian_speed.setter
    def robot_cartesian_speed(self, value):
        _copy_list_to_ctypes_array(self._robot_cartesian_speed, value)

    @property
    def robot_tool(self):
        return MutableArrayWrapper(self._robot_tool)

    @robot_tool.setter
    def robot_tool(self, value):
        _copy_list_to_ctypes_array(self._robot_tool, value)

    @property
    def robot_cartesian_pos(self):
        return MutableArrayWrapper(self._robot_cartesian_pos)

    @robot_cartesian_pos.setter
    def robot_cartesian_pos(self, value):
        _copy_list_to_ctypes_array(self._robot_cartesian_pos, value)

# ---------------------------------------------------------------------------
# IPC 数据包（对应 RobotTypesExport.h::IPCCommandPacket_C）
# ---------------------------------------------------------------------------

class PacketHeader(ctypes.Structure):
    """IPC 数据包头部结构 (24字节)。对应 PacketHeader_C。"""
    _fields_ = [
        ("protocol_version", ctypes.c_uint32),  # 通信协议版本号
        ("sequence_number",  ctypes.c_uint32),  # 报文序号
        ("data_length",      ctypes.c_uint16),  # 数据包字节长度
        ("packet_type",      ctypes.c_uint8),   # 0: EMI, 1: human
        ("reserved",         ctypes.c_uint8 * 10),  # 预留
    ]

class IPCCommandPacket(ctypes.Structure):
    """IPC下发数据包。对应 IPCCommandPacket_C。"""
    _fields_ = [
        ("header",              PacketHeader),
        ("ctrl_word",           ctypes.c_uint32),

        ("arm1_id",             ctypes.c_uint8),
        ("_arm1_cmd",           ctypes.c_float * 7),
        ("_arm1_force_cmd",     ctypes.c_float * 7),

        ("_hand1_cmd",          ctypes.c_float * 6),
        ("_hand1_speed_limit",  ctypes.c_float * 6),
        ("_hand1_current_limit",ctypes.c_float * 6),

        ("arm2_id",             ctypes.c_uint8),
        ("_arm2_cmd",           ctypes.c_float * 7),
        ("_arm2_force_cmd",     ctypes.c_float * 7),

        ("_hand2_cmd",          ctypes.c_float * 6),
        ("_hand2_speed_limit",  ctypes.c_float * 6),
        ("_hand2_current_limit",ctypes.c_float * 6),

        ("arm0_id",             ctypes.c_uint8),
        ("_head_waist_cmd",     ctypes.c_float * 3),

        ("_arm1_param",         ctypes.c_float * 5),
        ("_arm2_param",         ctypes.c_float * 5),

        ("checksum",            ctypes.c_uint32),
    ]

    @property
    def arm1_cmd(self):
        return MutableArrayWrapper(self._arm1_cmd)

    @arm1_cmd.setter
    def arm1_cmd(self, value):
        _copy_list_to_ctypes_array(self._arm1_cmd, value)

    @property
    def arm1_force_cmd(self):
        return MutableArrayWrapper(self._arm1_force_cmd)

    @arm1_force_cmd.setter
    def arm1_force_cmd(self, value):
        _copy_list_to_ctypes_array(self._arm1_force_cmd, value)

    @property
    def hand1_cmd(self):
        return MutableArrayWrapper(self._hand1_cmd)

    @hand1_cmd.setter
    def hand1_cmd(self, value):
        _copy_list_to_ctypes_array(self._hand1_cmd, value)

    @property
    def hand1_speed_limit(self):
        return MutableArrayWrapper(self._hand1_speed_limit)

    @hand1_speed_limit.setter
    def hand1_speed_limit(self, value):
        _copy_list_to_ctypes_array(self._hand1_speed_limit, value)

    @property
    def hand1_current_limit(self):
        return MutableArrayWrapper(self._hand1_current_limit)

    @hand1_current_limit.setter
    def hand1_current_limit(self, value):
        _copy_list_to_ctypes_array(self._hand1_current_limit, value)

    @property
    def arm2_cmd(self):
        return MutableArrayWrapper(self._arm2_cmd)

    @arm2_cmd.setter
    def arm2_cmd(self, value):
        _copy_list_to_ctypes_array(self._arm2_cmd, value)

    @property
    def arm2_force_cmd(self):
        return MutableArrayWrapper(self._arm2_force_cmd)

    @arm2_force_cmd.setter
    def arm2_force_cmd(self, value):
        _copy_list_to_ctypes_array(self._arm2_force_cmd, value)

    @property
    def hand2_cmd(self):
        return MutableArrayWrapper(self._hand2_cmd)

    @hand2_cmd.setter
    def hand2_cmd(self, value):
        _copy_list_to_ctypes_array(self._hand2_cmd, value)

    @property
    def hand2_speed_limit(self):
        return MutableArrayWrapper(self._hand2_speed_limit)

    @hand2_speed_limit.setter
    def hand2_speed_limit(self, value):
        _copy_list_to_ctypes_array(self._hand2_speed_limit, value)

    @property
    def hand2_current_limit(self):
        return MutableArrayWrapper(self._hand2_current_limit)

    @hand2_current_limit.setter
    def hand2_current_limit(self, value):
        _copy_list_to_ctypes_array(self._hand2_current_limit, value)

    @property
    def head_waist_cmd(self):
        return MutableArrayWrapper(self._head_waist_cmd)

    @head_waist_cmd.setter
    def head_waist_cmd(self, value):
        _copy_list_to_ctypes_array(self._head_waist_cmd, value)

    @property
    def arm1_param(self):
        return MutableArrayWrapper(self._arm1_param)

    @arm1_param.setter
    def arm1_param(self, value):
        _copy_list_to_ctypes_array(self._arm1_param, value)

    @property
    def arm2_param(self):
        return MutableArrayWrapper(self._arm2_param)

    @arm2_param.setter
    def arm2_param(self, value):
        _copy_list_to_ctypes_array(self._arm2_param, value)

# ---------------------------------------------------------------------------
# QLV 文件相关结构体
# ---------------------------------------------------------------------------
# 对应 RobotTypesExportC.h 中的 QLV*_C 类型

FILE_NAME_LENGTH = 256
FILE_PATH_LENGTH = 1024


class QLVLocationItem(ctypes.Structure):
    """QLV 笛卡尔位置项（位置 + 名称 + 注释）。对应 QLVLocationItem_C。"""
    _fields_ = [
        ("location",   Location),
        ("name",       ctypes.c_char * FILE_NAME_LENGTH),
        ("annotation", ctypes.c_char * FILE_PATH_LENGTH),
    ]

    def get_name(self):
        if not self.name:
            return ""
        try:
            return self.name.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.name.decode('utf-8', errors='ignore').rstrip('\x00')

    def get_annotation(self):
        if not self.annotation:
            return ""
        try:
            return self.annotation.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.annotation.decode('utf-8', errors='ignore').rstrip('\x00')


class QLVLocationJItem(ctypes.Structure):
    """QLV 关节位置项（关节位置 + 名称 + 注释）。对应 QLVLocationJItem_C。"""
    _fields_ = [
        ("location_j", LocationJ),
        ("name",       ctypes.c_char * FILE_NAME_LENGTH),
        ("annotation", ctypes.c_char * FILE_PATH_LENGTH),
    ]

    def get_name(self):
        if not self.name:
            return ""
        try:
            return self.name.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.name.decode('utf-8', errors='ignore').rstrip('\x00')

    def get_annotation(self):
        if not self.annotation:
            return ""
        try:
            return self.annotation.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.annotation.decode('utf-8', errors='ignore').rstrip('\x00')


class QLVProfileItem(ctypes.Structure):
    """QLV 运动参数项（Profile + 名称 + 注释）。对应 QLVProfileItem_C。"""
    _fields_ = [
        ("profile",    Profile),
        ("name",       ctypes.c_char * FILE_NAME_LENGTH),
        ("annotation", ctypes.c_char * FILE_PATH_LENGTH),
    ]

    def get_name(self):
        if not self.name:
            return ""
        try:
            return self.name.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.name.decode('utf-8', errors='ignore').rstrip('\x00')

    def get_annotation(self):
        if not self.annotation:
            return ""
        try:
            return self.annotation.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.annotation.decode('utf-8', errors='ignore').rstrip('\x00')


class QLVLocationArray(ctypes.Structure):
    """QLV 笛卡尔位置数组。对应 QLVLocationArray_C。"""
    _fields_ = [
        ("count",      ctypes.c_int),
        ("items",      ctypes.POINTER(QLVLocationItem)),
        ("name",       ctypes.c_char * FILE_NAME_LENGTH),
        ("annotation", ctypes.c_char * FILE_PATH_LENGTH),
    ]

    def get_name(self):
        if not self.name:
            return ""
        try:
            return self.name.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.name.decode('utf-8', errors='ignore').rstrip('\x00')

    def get_annotation(self):
        if not self.annotation:
            return ""
        try:
            return self.annotation.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.annotation.decode('utf-8', errors='ignore').rstrip('\x00')


class QLVLocationJArray(ctypes.Structure):
    """QLV 关节位置数组。对应 QLVLocationJArray_C。"""
    _fields_ = [
        ("count",      ctypes.c_int),
        ("items",      ctypes.POINTER(QLVLocationJItem)),
        ("name",       ctypes.c_char * FILE_NAME_LENGTH),
        ("annotation", ctypes.c_char * FILE_PATH_LENGTH),
    ]

    def get_name(self):
        if not self.name:
            return ""
        try:
            return self.name.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.name.decode('utf-8', errors='ignore').rstrip('\x00')

    def get_annotation(self):
        if not self.annotation:
            return ""
        try:
            return self.annotation.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.annotation.decode('utf-8', errors='ignore').rstrip('\x00')


class QLVProfileArray(ctypes.Structure):
    """QLV 运动参数数组。对应 QLVProfileArray_C。"""
    _fields_ = [
        ("count",      ctypes.c_int),
        ("items",      ctypes.POINTER(QLVProfileItem)),
        ("name",       ctypes.c_char * FILE_NAME_LENGTH),
        ("annotation", ctypes.c_char * FILE_PATH_LENGTH),
    ]

    def get_name(self):
        if not self.name:
            return ""
        try:
            return self.name.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.name.decode('utf-8', errors='ignore').rstrip('\x00')

    def get_annotation(self):
        if not self.annotation:
            return ""
        try:
            return self.annotation.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.annotation.decode('utf-8', errors='ignore').rstrip('\x00')


class QLVFile(ctypes.Structure):
    """QLV 文件结构体。对应 QLVFile_C。"""
    _fields_ = [
        ("locations_count",         ctypes.c_int),
        ("locations",               ctypes.POINTER(QLVLocationItem)),
        ("location_js_count",       ctypes.c_int),
        ("location_js",             ctypes.POINTER(QLVLocationJItem)),
        ("profiles_count",          ctypes.c_int),
        ("profiles",                ctypes.POINTER(QLVProfileItem)),
        ("location_arrays_count",   ctypes.c_int),
        ("location_arrays",         ctypes.POINTER(QLVLocationArray)),
        ("location_j_arrays_count", ctypes.c_int),
        ("location_j_arrays",       ctypes.POINTER(QLVLocationJArray)),
        ("profile_arrays_count",    ctypes.c_int),
        ("profile_arrays",          ctypes.POINTER(QLVProfileArray)),
    ]


# ---------------------------------------------------------------------------
# Pallas 解决方案和项目相关结构体
# ---------------------------------------------------------------------------

class SolutionInfo(ctypes.Structure):
    """Pallas 解决方案信息。对应 SolutionInfo_C。"""
    _fields_ = [
        ("name",            ctypes.c_char * FILE_NAME_LENGTH),
        ("path",            ctypes.c_char * FILE_PATH_LENGTH),
        ("is_qblock",       ctypes.c_int),
        ("main_project",    ctypes.c_char * FILE_NAME_LENGTH),
        ("project_count",   ctypes.c_int),
        ("projectNames",    ctypes.POINTER(ctypes.c_char_p)),
    ]

    def get_name(self):
        if not self.name:
            return ""
        try:
            return self.name.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.name.decode('utf-8', errors='ignore').rstrip('\x00')

    def get_path(self):
        if not self.path:
            return ""
        try:
            return self.path.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.path.decode('utf-8', errors='ignore').rstrip('\x00')

    def get_main_project(self):
        if not self.main_project:
            return ""
        try:
            return self.main_project.decode('utf-8').rstrip('\x00')
        except UnicodeDecodeError:
            return self.main_project.decode('utf-8', errors='ignore').rstrip('\x00')

    def get_projects(self):
        if not self.projectNames or self.project_count <= 0:
            return []
        result = []
        for i in range(self.project_count):
            try:
                result.append(self.projectNames[i].decode('utf-8'))
            except (UnicodeDecodeError, AttributeError):
                result.append(self.projectNames[i].decode('utf-8', errors='ignore') if self.projectNames[i] else "")
        return result


class ProjectInfo(ctypes.Structure):
    """Pallas 项目信息。对应 ProjectInfo_C。"""
    _fields_ = [
        ("name",            ctypes.c_char * FILE_NAME_LENGTH),
        ("path",            ctypes.c_char * FILE_PATH_LENGTH),
        ("main_file",       ctypes.c_char * FILE_NAME_LENGTH),
        ("ql_files_count",  ctypes.c_int),
        ("qlv_files_count", ctypes.c_int),
        ("ql_files",        ctypes.POINTER(ctypes.c_char_p)),
        ("qlv_files",       ctypes.POINTER(ctypes.c_char_p)),
    ]

    def get_name(self):
        return self.name.decode('utf-8') if self.name else ""

    def get_path(self):
        return self.path.decode('utf-8') if self.path else ""

    def get_main_file(self):
        return self.main_file.decode('utf-8') if self.main_file else ""

    def get_ql_files(self):
        if not self.ql_files or self.ql_files_count <= 0:
            return []
        return [self.ql_files[i].decode('utf-8') for i in range(self.ql_files_count)]

    def get_qlv_files(self):
        if not self.qlv_files or self.qlv_files_count <= 0:
            return []
        return [self.qlv_files[i].decode('utf-8') for i in range(self.qlv_files_count)]
