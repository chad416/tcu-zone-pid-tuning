"""V01: the SCL source text (plc/*.scl), executed by tcutune.scl, matches the
Python reference models.  Requirement IDs refer to SPEC.md."""

from pathlib import Path

import numpy as np
import pytest

from tcutune import scl
from tcutune.experiments import Scenario, closed_loop
from tcutune.pid import PidParams, ZonePID
from tcutune.plant import PRESETS, Zone

PLC = sorted((Path(__file__).resolve().parents[1] / "plc").glob("*.scl"))


@pytest.fixture(scope="module")
def fbs():
    return scl.load(PLC)


def _set(fb, **kw):
    for k, v in kw.items():
        setattr(fb, k, np.float32(v) if isinstance(v, float) else v)


@pytest.mark.parametrize("td,dead,ratio", [(0.0, 0.0, 1.0), (30.0, 0.0, 2.5), (12.0, 0.5, 0.7)])
def test_pid_bit_exact_random_inputs(fbs, td, dead, ratio):
    """F08/V01: FB_ZonePID == ZonePID in float32, incl. manual, reset, saturation."""
    rng = np.random.default_rng(42)
    prm = PidParams(Gain=18.0, Ti=120.0, Td=td, N=8.0, SpWeight=0.7, Tt=0.0,
                    CoolGainRatio=ratio, Deadband=dead)
    ref = ZonePID(prm, Ts=0.1)
    fb = fbs["FB_ZonePID"]()
    _set(fb, Gain=prm.Gain, Ti=prm.Ti, Td=prm.Td, DerivFilterN=prm.N, SpWeight=prm.SpWeight,
         Tt=prm.Tt, CoolGainRatio=prm.CoolGainRatio, Deadband=prm.Deadband, CycleTime=0.1)
    pv = 40.0
    for k in range(6000):
        sp = 70.0 if k < 2000 else (85.0 if k < 4000 else 60.0)
        pv += rng.normal(0, 0.3) + 0.002 * (sp - pv)
        manual = 2500 <= k < 2700
        mv = float(rng.uniform(-80, 100)) if manual else 0.0
        reset = k in (100, 4500)
        h, c = ref.step(sp, pv, manual, mv, reset)
        _set(fb, Setpoint=float(sp), Input=float(pv), ManualEnable=manual,
             ManualValue=float(mv), Reset=reset)
        fb()
        assert fb.OutHeat == np.float32(h) and fb.OutCool == np.float32(c), k
        assert fb.Output == ref.U and fb.sI == ref.I and fb.sD == ref.D, k


def test_plant_matches_python_model(fbs):
    """F09: FB_ZonePlant (float32) follows plant.Zone (float64) within 0.01 K."""
    p = PRESETS["A"]
    ref = Zone(p, Ts=0.1, T0=25.0)
    fb = fbs["FB_ZonePlant"]()
    rng = np.random.default_rng(7)
    heat, cool, q = 60.0, 0.0, 0.0
    worst = 0.0
    for k in range(30000):
        if k % 1500 == 0:
            heat = float(rng.uniform(0, 100))
            cool = float(rng.uniform(0, 30)) if rng.random() < 0.4 else 0.0
            q = float(rng.uniform(0, 12))
        y = ref.step(heat, cool, q)
        _set(fb, Heat=heat, Cool=cool, Qproc=q, Init=(k == 0), InitTemp=25.0)
        fb()
        worst = max(worst, abs(float(fb.Pv) - y))
    assert worst < 0.01, worst


def test_scenario_profile(fbs):
    """F11: FB_Scenario switches SP and load at the same times as Scenario."""
    sc = Scenario().scaled(0.05)
    fb = fbs["FB_Scenario"]()
    _set(fb, tSpHigh=sc.t_sp_high, tLoadOn=sc.t_load_on, tSpLow=sc.t_sp_low,
         tLoadOff=sc.t_load_off, tEnd=sc.t_end, Restart=True)
    for k in range(int(sc.t_end / 0.1) + 5):
        fb()
        fb.Restart = False
        t = k * 0.1
        sp = sc.sp_low if t >= sc.t_sp_low else (sc.sp_high if t >= sc.t_sp_high else sc.sp_start)
        q = sc.q_run if sc.t_load_on <= t < sc.t_load_off else 0.0
        near_edge = any(abs(t - e) < 0.15 for _, e in sc.events())
        if not near_edge:
            assert float(fb.Setpoint) == sp and float(fb.Qproc) == q, t
    assert fb.Done


def test_closed_loop_harness_matches_python(fbs):
    """V01: FB_ZoneTest (whole SCL program) == experiments.closed_loop within 0.05 K."""
    sc = Scenario().scaled(0.1)  # 1,200 s compressed profile
    prm = PidParams(Gain=21.0, Ti=105.0, CoolGainRatio=2.7)
    ref = closed_loop(PRESETS["A"], prm, sc)
    fb = fbs["FB_ZoneTest"]()
    _set(fb, Gain=prm.Gain, Ti=prm.Ti, CoolGainRatio=prm.CoolGainRatio, Restart=True)
    _set(fb.iScenario, tSpHigh=sc.t_sp_high, tLoadOn=sc.t_load_on, tSpLow=sc.t_sp_low,
         tLoadOff=sc.t_load_off, tEnd=sc.t_end)
    pv = np.empty(len(ref.t))
    for k in range(len(ref.t)):
        fb()
        fb.Restart = False
        pv[k] = fb.Pv
    # the restart call is sample 0 in both; edges may differ by one sample (float32 time)
    assert np.max(np.abs(pv - ref.pv)) < 0.05


def test_translator_rejects_unsupported():
    with pytest.raises(scl.SclError):
        scl.translate(['FUNCTION_BLOCK "X"\nVAR a : Real; END_VAR\nBEGIN\n'
                       '#a := SIN(#a);\nEND_FUNCTION_BLOCK'])
    with pytest.raises(scl.SclError):
        scl.translate(['FUNCTION_BLOCK "X"\nVAR a : Real; END_VAR\nBEGIN\n'
                       'CASE #a OF 1: #a := 0.0; END_CASE;\nEND_FUNCTION_BLOCK'])
    with pytest.raises(scl.SclError):
        scl.translate(['FUNCTION_BLOCK "X"\nVAR a : Real; END_VAR\nBEGIN\n'
                       '#b := 1.0;\nEND_FUNCTION_BLOCK'])


def test_translator_rejects_reserved_names():
    """TIA V18 rejected an output named 'Time' (first compile, 9 Nov 2026)."""
    with pytest.raises(scl.SclError, match="reserved"):
        scl.translate(['FUNCTION_BLOCK "X"\nVAR_OUTPUT Time : Real; END_VAR\nBEGIN\n'
                       '#Time := 1.0;\nEND_FUNCTION_BLOCK'])


def test_translator_integer_semantics():
    code, _ = scl.translate(['FUNCTION_BLOCK "X"\nVAR a : Int; b : Int; c : DInt; END_VAR\nBEGIN\n'
                             '#a := -7 / 2;\n#b := 32767 + 1;\n#c := -7 MOD 2;\nEND_FUNCTION_BLOCK'])
    ns = {"_FB": {}}
    exec(code, ns)
    x = ns["_FB"]["X"]()
    x()
    assert x.a == -3  # SCL integer division truncates toward zero
    assert x.b == -32768  # Int wraps
    assert x.c == -1
