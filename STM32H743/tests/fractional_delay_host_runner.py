"""运行直接链接 FilterEx.c 的分数延时器主机回归测试。"""
import ctypes
import math
import pathlib
import sys


class FractionalDelayHostResult(ctypes.Structure):
    """与 C 测试导出结构体一一对应的结果结构体。"""

    _fields_ = [
        ("failed_checks", ctypes.c_uint32),
        ("configured_delay", ctypes.c_float),
        ("sine_component", ctypes.c_float),
        ("cosine_component", ctypes.c_float),
        ("max_integer_error", ctypes.c_float),
        ("max_block_error", ctypes.c_float),
    ]


def main() -> int:
    """加载 DLL，打印 C 代码真实运行后的相移测试结果。"""
    dll_path = pathlib.Path(__file__).with_name("build") / "fractional_delay_host_test.dll"
    library = ctypes.CDLL(str(dll_path))
    run_test = library.FractionalDelay_HostTest
    run_test.argtypes = [ctypes.POINTER(FractionalDelayHostResult)]
    run_test.restype = ctypes.c_uint32

    result = FractionalDelayHostResult()
    failed_checks = run_test(ctypes.byref(result))
    amplitude = math.hypot(result.sine_component, result.cosine_component)
    phase_deg = math.degrees(math.atan2(result.cosine_component, result.sine_component))

    print("实际 FilterEx_FractionalDelay C 代码测试")
    print(f"SetPhase(1 MSPS, 100 kHz, 90°) 延时: {result.configured_delay:.6f} 点")
    print(f"输出幅值比: {amplitude:.6f}")
    print(f"输出相位: {phase_deg:.3f}°（延时方向，目标为 -90°）")
    print(f"50 kHz、90°整数延时最大误差: {result.max_integer_error:.7f}")
    print(f"原地块处理最大误差: {result.max_block_error:.7f}")
    print(f"参数及功能检查失败位图: 0x{failed_checks:08X}")
    return 0 if failed_checks == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
