"""E题软件移相预验证。

本脚本只验证算法，不修改正式DSP层：
1. 用环形缓冲和线性插值实现分数采样延时；
2. 检查10~100 kHz、0~180°范围内的相位误差和幅值误差；
3. 按E题关系构造C=A+B，并验证D为C的延时版本。
"""

from __future__ import annotations

import math
from dataclasses import dataclass


TWO_PI = 2.0 * math.pi  # 两倍圆周率。


@dataclass
class FractionalDelay:
    """无动态内存的线性分数延时器模型。"""

    capacity: int  # 环形缓冲区容量。
    delay_samples: float = 0.0  # 当前延时，单位为采样点。

    def __post_init__(self) -> None:
        self.buffer = [0.0] * self.capacity  # 延时历史数据。
        self.write_index = 0  # 当前写入位置。
        self.set_delay_samples(self.delay_samples)

    def set_delay_samples(self, delay_samples: float) -> None:
        """设置分数采样延时。"""
        if not math.isfinite(delay_samples):
            raise ValueError("延时必须是有限数")
        if delay_samples < 0.0 or delay_samples > self.capacity - 2:
            raise ValueError("延时超出缓冲区容量")
        self.delay_samples = delay_samples

    def reset(self) -> None:
        """清零历史状态。"""
        self.buffer[:] = [0.0] * self.capacity
        self.write_index = 0

    def process(self, sample: float) -> float:
        """处理一个采样点并返回延时结果。"""
        self.buffer[self.write_index] = sample
        integer_delay = int(math.floor(self.delay_samples))
        fraction = self.delay_samples - integer_delay
        index0 = (self.write_index - integer_delay) % self.capacity
        index1 = (index0 - 1) % self.capacity
        output = ((1.0 - fraction) * self.buffer[index0]
                  + fraction * self.buffer[index1])
        self.write_index = (self.write_index + 1) % self.capacity
        return output


@dataclass
class NlmsFilter:
    """与Contest_LMSProcess计算顺序一致的归一化LMS模型。"""

    taps: int  # 滤波器抽头数。
    mu: float  # 归一化LMS步长。
    leak: float = 0.0  # 权重泄漏系数。

    def __post_init__(self) -> None:
        if self.taps <= 0:
            raise ValueError("LMS抽头数必须大于0")
        self.weights = [0.0] * self.taps  # 自适应权重。
        self.state = [0.0] * self.taps  # 参考输入历史。

    def reset(self) -> None:
        """清零权重和输入历史。"""
        self.weights[:] = [0.0] * self.taps
        self.state[:] = [0.0] * self.taps

    def process(self, reference: float, desired: float) -> tuple[float, float]:
        """处理一个样本，返回干扰估计值和误差信号。"""
        for index in range(self.taps - 1, 0, -1):
            self.state[index] = self.state[index - 1]
        self.state[0] = reference

        output = 0.0
        power = 1.0e-9
        for index in range(self.taps):
            output += self.weights[index] * self.state[index]
            power += self.state[index] * self.state[index]

        error = desired - output
        step = self.mu * error / power
        keep = 1.0 - self.leak
        for index in range(self.taps):
            self.weights[index] = (keep * self.weights[index]
                                   + step * self.state[index])
        return output, error


@dataclass
class SineAccumulator:
    """流式正弦分量幅值估计器，避免保存长时间测试数组。"""

    ss: float = 0.0
    cc: float = 0.0
    sc: float = 0.0
    sy: float = 0.0
    cy: float = 0.0

    def push(self, value: float, phase: float) -> None:
        """加入一个样本。"""
        sine = math.sin(phase)
        cosine = math.cos(phase)
        self.ss += sine * sine
        self.cc += cosine * cosine
        self.sc += sine * cosine
        self.sy += value * sine
        self.cy += value * cosine

    def amplitude(self) -> float:
        """返回估计幅值。"""
        determinant = self.ss * self.cc - self.sc * self.sc
        if abs(determinant) < 1.0e-12:
            raise RuntimeError("正弦分量估计矩阵退化")
        sine_gain = (self.sy * self.cc - self.cy * self.sc) / determinant
        cosine_gain = (self.cy * self.ss - self.sy * self.sc) / determinant
        return math.hypot(sine_gain, cosine_gain)


def wrap_phase_deg(phase_deg: float) -> float:
    """把相位误差折叠到-180~180°。"""
    return (phase_deg + 180.0) % 360.0 - 180.0


def estimate_sine_component(samples: list[float], fs: float, freq: float,
                            start: int) -> tuple[float, float]:
    """用正弦/余弦相关估计幅值和相位。"""
    ss = cc = sc = sy = cy = 0.0
    for index in range(start, len(samples)):
        phase = TWO_PI * freq * index / fs
        sine = math.sin(phase)
        cosine = math.cos(phase)
        value = samples[index]
        ss += sine * sine
        cc += cosine * cosine
        sc += sine * cosine
        sy += value * sine
        cy += value * cosine

    determinant = ss * cc - sc * sc
    if abs(determinant) < 1.0e-12:
        raise RuntimeError("正弦分量估计矩阵退化")
    sine_gain = (sy * cc - cy * sc) / determinant
    cosine_gain = (cy * ss - sy * sc) / determinant
    amplitude = math.hypot(sine_gain, cosine_gain)
    phase_deg = math.degrees(math.atan2(cosine_gain, sine_gain))
    return amplitude, phase_deg


def run_single_tone_test() -> tuple[float, float, int]:
    """验证单频10~100 kHz、0~180°移相。"""
    fs = 1_000_000.0  # 1 MSPS，在100 kHz下每周期10点。
    frequencies = (10_000.0, 30_000.0, 50_000.0, 70_000.0, 100_000.0)
    phases = (0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0)
    sample_count = 20_000
    warmup = 256
    delay = FractionalDelay(capacity=64)
    worst_gain_error = 0.0
    worst_phase_error = 0.0
    cases = 0

    for freq in frequencies:
        for phase_deg in phases:
            delay.set_delay_samples(fs * phase_deg / (360.0 * freq))
            delay.reset()
            output = []
            for index in range(sample_count):
                input_sample = math.sin(TWO_PI * freq * index / fs)
                output.append(delay.process(input_sample))

            amplitude, measured_phase = estimate_sine_component(
                output, fs, freq, warmup
            )
            gain_error = abs(amplitude - 1.0)
            phase_error = abs(wrap_phase_deg(measured_phase + phase_deg))
            worst_gain_error = max(worst_gain_error, gain_error)
            worst_phase_error = max(worst_phase_error, phase_error)
            cases += 1

            if gain_error > 0.10 or phase_error > 2.0:
                raise AssertionError(
                    f"单频测试失败: f={freq:g}Hz, phase={phase_deg:g}°, "
                    f"gain={amplitude:.6f}, phase={measured_phase:.3f}°"
                )

    return worst_gain_error, worst_phase_error, cases


def run_e_signal_test() -> tuple[float, float, float, float]:
    """按E题构造C=A+B并验证软件移相后的D。"""
    fs = 1_000_000.0  # 采样率。
    freq_a = 30_000.0  # 有用信号频率。
    freq_b = 70_000.0  # 干扰信号频率。
    phase_b = 90.0  # 以B频率为基准设置移相角。
    delay_seconds = phase_b / (360.0 * freq_b)
    delay = FractionalDelay(capacity=64)
    delay.set_delay_samples(fs * delay_seconds)

    samples_c = []
    samples_d = []
    sample_count = 30_000
    for index in range(sample_count):
        time_s = index / fs
        signal_a = 0.75 * math.sin(TWO_PI * freq_a * time_s + 0.31)
        signal_b = 0.65 * math.sin(TWO_PI * freq_b * time_s - 0.27)
        signal_c = signal_a + signal_b
        samples_c.append(signal_c)
        samples_d.append(delay.process(signal_c))

    warmup = 256
    amp_c_a, phase_c_a = estimate_sine_component(samples_c, fs, freq_a, warmup)
    amp_d_a, phase_d_a = estimate_sine_component(samples_d, fs, freq_a, warmup)
    amp_c_b, phase_c_b = estimate_sine_component(samples_c, fs, freq_b, warmup)
    amp_d_b, phase_d_b = estimate_sine_component(samples_d, fs, freq_b, warmup)

    gain_a = amp_d_a / amp_c_a
    gain_b = amp_d_b / amp_c_b
    phase_error_b = abs(wrap_phase_deg((phase_d_b - phase_c_b) + phase_b))
    phase_error_a = abs(wrap_phase_deg(
        (phase_d_a - phase_c_a) + phase_b * freq_a / freq_b
    ))
    return gain_a, gain_b, phase_error_a, phase_error_b


def run_e_nlms_test() -> tuple[float, float]:
    """验证E题100 Hz频差下的软件移相和NLMS完整链路。"""
    fs = 1_000_000.0  # 采样率。
    phase_b = 137.0  # 选择非整采样点相移，覆盖分数插值路径。
    taps = 16  # 基本要求正弦干扰使用16抽头。
    mu = 5.0e-5  # 兼顾100 Hz分离能力和响应时间的步长。
    cases = (
        (10_100.0, 10_000.0),
        (50_100.0, 50_000.0),
        (99_900.0, 100_000.0),
    )
    sample_count = 700_000
    measure_start = 500_000
    worst_a_gain_error = 0.0
    worst_b_residual = 0.0

    for freq_a, freq_b in cases:
        delay = FractionalDelay(capacity=64)
        delay.set_delay_samples(fs * phase_b / (360.0 * freq_b))
        nlms = NlmsFilter(taps=taps, mu=mu)
        error_a = SineAccumulator()
        error_b = SineAccumulator()
        desired_a = SineAccumulator()
        desired_b = SineAccumulator()

        for index in range(sample_count):
            time_s = index / fs
            signal_a = 0.75 * math.sin(TWO_PI * freq_a * time_s + 0.31)
            signal_b = 0.65 * math.sin(TWO_PI * freq_b * time_s - 0.27)
            desired = delay.process(signal_a + signal_b)
            _, error = nlms.process(signal_b, desired)

            if index >= measure_start:
                phase_a = TWO_PI * freq_a * time_s
                phase_b_now = TWO_PI * freq_b * time_s
                error_a.push(error, phase_a)
                error_b.push(error, phase_b_now)
                desired_a.push(desired, phase_a)
                desired_b.push(desired, phase_b_now)

        a_gain = error_a.amplitude() / desired_a.amplitude()
        b_residual = error_b.amplitude() / desired_b.amplitude()
        worst_a_gain_error = max(worst_a_gain_error, abs(a_gain - 1.0))
        worst_b_residual = max(worst_b_residual, b_residual)
        if abs(a_gain - 1.0) > 0.10 or b_residual > 0.01:
            raise AssertionError(
                f"E题NLMS失败: fA={freq_a:g}Hz, fB={freq_b:g}Hz, "
                f"A增益={a_gain:.6f}, B残余={b_residual:.6f}"
            )

    return worst_a_gain_error, worst_b_residual


def run_e_response_test() -> float:
    """按题目方法验证仅加入B时衰减到1%的最长响应时间。"""
    fs = 1_000_000.0  # 采样率。
    taps = 16  # LMS抽头数。
    mu = 5.0e-5  # LMS步长。
    window_samples = 1_000  # 1 ms有效值观察窗口。
    frequencies = (10_000.0, 50_000.0, 100_000.0)
    phases = (0.0, 45.0, 90.0, 135.0, 180.0)
    max_sample_count = 500_000
    worst_response_time = 0.0

    for freq_b in frequencies:
        for phase_b in phases:
            delay = FractionalDelay(capacity=64)
            delay.set_delay_samples(fs * phase_b / (360.0 * freq_b))
            nlms = NlmsFilter(taps=taps, mu=mu)
            error_power = 0.0
            desired_power = 0.0
            response_time = None

            for index in range(max_sample_count):
                time_s = index / fs
                signal_b = 0.65 * math.sin(TWO_PI * freq_b * time_s - 0.27)
                desired = delay.process(signal_b)
                _, error = nlms.process(signal_b, desired)
                error_power += error * error
                desired_power += desired * desired

                if (index + 1) % window_samples == 0:
                    ratio = math.sqrt(error_power / max(desired_power, 1.0e-20))
                    if ratio <= 0.01:
                        response_time = (index + 1) / fs
                        break
                    error_power = 0.0
                    desired_power = 0.0

            if response_time is None:
                raise AssertionError(
                    f"响应时间失败: fB={freq_b:g}Hz, phase={phase_b:g}°"
                )
            worst_response_time = max(worst_response_time, response_time)

    return worst_response_time


def main() -> None:
    """运行全部预验证并输出结果。"""
    gain_error, phase_error, cases = run_single_tone_test()
    gain_a, gain_b, phase_a, phase_b = run_e_signal_test()
    print(f"单频测试: {cases}组通过")
    print(f"单频最坏幅值误差: {gain_error * 100.0:.3f}%")
    print(f"单频最坏相位误差: {phase_error:.3f}°")
    print(f"E题C=A+B测试: A幅值比={gain_a:.6f}, B幅值比={gain_b:.6f}")
    print(f"E题相位误差: A={phase_a:.3f}°, B={phase_b:.3f}°")
    if abs(gain_a - 1.0) > 0.10 or abs(gain_b - 1.0) > 0.10:
        raise AssertionError("E题混合信号的幅值误差超过10%")
    if phase_a > 2.0 or phase_b > 2.0:
        raise AssertionError("E题混合信号的相位误差超过2°")
    a_gain_error, b_residual = run_e_nlms_test()
    response_time = run_e_response_test()
    print(f"E题NLMS最坏A幅值误差: {a_gain_error * 100.0:.3f}%")
    print(f"E题NLMS最坏B残余: {b_residual * 100.0:.4f}%")
    print(f"E题NLMS最长响应时间: {response_time:.3f}s")
    print("结论: 软件移相和现有NLMS算法通过本次E题基本要求预验证。")


if __name__ == "__main__":
    main()
