"""SPS-204/205: controller behaviour (Python reference; the SCL is checked against it)."""

import numpy as np

from tcutune.pid import PidParams, ZonePID


def test_p_only_and_split_range():
    c = ZonePID(PidParams(Gain=10.0, Ti=0.0))
    assert c.step(70.0, 68.0) == (20.0, 0.0)
    h, cool = c.step(70.0, 72.0)
    assert h == 0.0 and cool == 20.0


def test_cool_gain_ratio_scales_cooling_and_limit():
    c = ZonePID(PidParams(Gain=10.0, Ti=0.0, CoolGainRatio=2.5))
    h, cool = c.step(70.0, 74.0)  # u = -40 -> valve 40/2.5 = 16 %
    assert h == 0.0 and abs(cool - 16.0) < 1e-5
    h, cool = c.step(70.0, 100.0)  # u = -300 -> limited at -250 -> valve 100 %
    assert cool == 100.0 and float(c.U) == -250.0


def test_integral_removes_offset_and_anti_windup():
    c = ZonePID(PidParams(Gain=5.0, Ti=50.0), Ts=0.1)
    for _ in range(5000):  # saturated for 500 s, error +30 K
        c.step(100.0, 70.0)
    assert float(c.U) == 100.0
    # back-calculation keeps the integral near the limit, not thousands of %
    assert float(c.I) < 100.0 + 5.0 * 30.0
    # when the error reverses, the output leaves saturation within a few samples
    for k in range(50):
        c.step(100.0, 101.0)
        if float(c.U) < 100.0:
            break
    assert k < 20


def test_bumpless_manual_to_auto():
    c = ZonePID(PidParams(Gain=8.0, Ti=100.0), Ts=0.1)
    for _ in range(100):
        c.step(70.0, 69.0, Manual=True, ManualValue=35.0)
    assert float(c.U) == 35.0
    c.step(70.0, 69.0)  # back to auto: same output (+ one integration step)
    assert abs(float(c.U) - 35.0) < 0.01


def test_derivative_on_measurement_not_setpoint():
    c = ZonePID(PidParams(Gain=5.0, Ti=0.0, Td=20.0, N=10.0), Ts=0.1)
    c.step(70.0, 70.0)
    c.step(80.0, 70.0)  # SP step: no derivative kick
    assert abs(float(c.D)) < 1e-9
    c.step(80.0, 70.5)  # PV moves: derivative acts against it
    assert float(c.D) < 0.0


def test_deadband_switches_both_off():
    c = ZonePID(PidParams(Gain=1.0, Ti=0.0, Deadband=2.0))
    assert c.step(70.0, 69.0) == (0.0, 0.0)
    assert c.step(70.0, 67.0) == (3.0, 0.0)


def test_reset():
    c = ZonePID(PidParams(Gain=5.0, Ti=50.0))
    for _ in range(100):
        c.step(80.0, 70.0)
    assert c.step(80.0, 70.0, Reset=True) == (0.0, 0.0)
    assert float(c.I) == 0.0
    assert isinstance(c.I, np.float32)


def test_conditional_integration_when_cooling_valve_saturates():
    """SPS-205: with CoolSaturated the integrator does not wind further into cooling."""
    from tcutune.pid import PidParams, ZonePID
    prm = PidParams(Gain=10.0, Ti=100.0, CoolGainRatio=3.0)
    free, held = ZonePID(prm), ZonePID(prm)
    for _ in range(50):  # too hot: the PID asks for cooling
        free.step(70.0, 72.0)
        held.step(70.0, 72.0, CoolSaturated=True)
    assert held.U < 0 and free.I < held.I  # the held integrator stopped
    # heating direction is never blocked
    i0 = held.I
    held.step(70.0, 60.0, CoolSaturated=True)
    assert held.I > i0
