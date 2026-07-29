"""E题自适应滤波数值回归。

本脚本逐行保持与DSP/Adaptive.c中Contest_LMSProcess()一致的：
状态移位、归一化功率、泄漏和权重更新公式。它验证软件移相后的
D=A'+B'与参考B输入LMS后，输出E能保留A并消除B。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from phase_shift_e_validation import FractionalDelay


TWO_PI = 2.0 * math.pi  # 两倍圆周率。
SAMPLE_RATE_HZ = 1_000_000.0  # 验证使用的采样率。
LMS_TAPS = 2  # 两个相邻采样可表示单频信号的任意相位。
LMS_MU = 0.0002  # 兼顾100 Hz频差保真和收敛速度的步长。


@dataclass
class ContestLmsModel:
    """与Contest_LMSProcess()同公式的主机数值模型。"""

    taps: int
    mu: float
    leak: float = 0.0

    def __post_init__(self) -> None:
        self.weights = [0.0] * self.taps  # LMS权重数组。
        self.state = [0.0] * self.taps  # 输入历史数组。

    def process(self, x: float, desired: float) -> tuple[float, float]:
        """完全按Contest_LMSProcess()的顺序处理一个样本。"""
        for index in range(self.taps - 1, 0, -1):
            self.state[index] = self.state[index - 1]
        self.state[0] = x

        output = 0.0
        power = 1.0e-9
        for index in range(self.taps):
            output += self.weights[index] * self.state[index]
            power += self.state[index] * self.state[index]

        error = desired - output
        step = self.mu * error / power
        keep = 1.0 - self.leak
        for index in range(self.taps):
            self.weights[index] = (
                keep * self.weights[index] + step * self.state[index]
            )
        return output, error


@dataclass
class SineFit:
    """指定频率正弦分量的流式最小二乘估计器。"""

    ss: float = 0.0
    cc: float = 0.0
    sc: float = 0.0
    sy: float = 0.0
    cy: float = 0.0

    def push(self, sample_index: int, frequency_hz: float, value: float) -> None:
        """加入一个样本。"""
        phase = TWO_PI * frequency_hz * sample_index / SAMPLE_RATE_HZ
        sine = math.sin(phase)
        cosine = math.cos(phase)
        self.ss += sine * sine
        self.cc += cosine * cosine
        self.sc += sine * cosine
        self.sy += value * sine
        self.cy += value * cosine

    def amplitude(self) -> float:
        """返回估计到的正弦幅值。"""
        determinant = self.ss * self.cc - self.sc * self.sc
        if abs(determinant) < 1.0e-12:
            raise RuntimeError("正弦拟合矩阵退化")
        sine_gain = (self.sy * self.cc - self.cy * self.sc) / determinant
        cosine_gain = (self.cy * self.ss - self.sy * self.sc) / determinant
        return math.hypot(sine_gain, cosine_gain)


def run_recovery_case(freq_a: float, freq_b: float, phase_deg: float) -> tuple[float, float]:
    """验证D=A'+B'时，LMS输出E对A和B的处理结果。"""
    lms = ContestLmsModel(LMS_TAPS, LMS_MU)
    delay = FractionalDelay(capacity=64)
    delay.set_delay_samples(
        SAMPLE_RATE_HZ * phase_deg / (360.0 * freq_b)
    )
    total_samples = 1_200_000
    measure_start = 700_000
    e_fit_a = SineFit()
    d_fit_a = SineFit()
    e_fit_b = SineFit()
    d_fit_b = SineFit()

    for index in range(total_samples):
        time_s = index / SAMPLE_RATE_HZ
        signal_a = 0.75 * math.sin(TWO_PI * freq_a * time_s + 0.31)
        signal_b = 0.65 * math.sin(TWO_PI * freq_b * time_s - 0.27)
        signal_d = delay.process(signal_a + signal_b)
        _, signal_e = lms.process(signal_b, signal_d)

        if index >= measure_start:
            e_fit_a.push(index, freq_a, signal_e)
            d_fit_a.push(index, freq_a, signal_d)
            e_fit_b.push(index, freq_b, signal_e)
            d_fit_b.push(index, freq_b, signal_d)

    a_gain = e_fit_a.amplitude() / d_fit_a.amplitude()
    b_residual = e_fit_b.amplitude() / d_fit_b.amplitude()
    print(
        f"恢复测试: A={freq_a:.1f}Hz, B={freq_b:.1f}Hz, "
        f"相移={phase_deg:.1f}°, A幅值比={a_gain:.5f}, "
        f"B残余={b_residual * 100.0:.5f}%"
    )
    if not 0.90 <= a_gain <= 1.10:
        raise AssertionError("A恢复幅值误差超过10%")
    if b_residual > 0.01:
        raise AssertionError("B残余超过1%")
    return a_gain, b_residual


def run_response_case(freq_b: float, phase_deg: float) -> float:
    """模拟重新加入B后达到1%残余的时间。"""
    lms = ContestLmsModel(LMS_TAPS, LMS_MU)
    delay = FractionalDelay(capacity=64)
    delay.set_delay_samples(
        SAMPLE_RATE_HZ * phase_deg / (360.0 * freq_b)
    )
    block_samples = max(1_000, int(20.0 * SAMPLE_RATE_HZ / freq_b))
    fit = SineFit()

    for index in range(1_000_000):
        time_s = index / SAMPLE_RATE_HZ
        signal_b = 0.65 * math.sin(TWO_PI * freq_b * time_s - 0.27)
        signal_d = delay.process(signal_b)
        _, signal_e = lms.process(signal_b, signal_d)
        fit.push(index, freq_b, signal_e)

        if (index + 1) % block_samples == 0:
            residual = fit.amplitude() / 0.65
            if residual < 0.01:
                response_s = (index + 1) / SAMPLE_RATE_HZ
                print(
                    f"响应测试: B={freq_b:.1f}Hz, 相移={phase_deg:.1f}°, "
                    f"达到1%残余时间={response_s:.3f}s"
                )
                return response_s
            fit = SineFit()

    raise AssertionError("1秒内未达到1%残余")


def main() -> None:
    """运行E题恢复和响应时间回归。"""
    # 三组频差均为100 Hz，覆盖10~100 kHz工作范围。
    run_recovery_case(10_100.0, 10_000.0, 180.0)
    run_recovery_case(50_100.0, 50_000.0, 137.0)
    run_recovery_case(99_900.0, 100_000.0, 90.0)

    # 对应E题发挥部分的响应时间测试方法：仅重新加入B。
    for frequency_hz in (10_000.0, 50_000.0, 100_000.0):
        response_s = run_response_case(frequency_hz, 137.0)
        if response_s > 1.0:
            raise AssertionError("响应时间超过1秒")

    print("结论: 现有Contest_LMSProcess公式在本E题测试条件下通过。")


if __name__ == "__main__":
    main()
