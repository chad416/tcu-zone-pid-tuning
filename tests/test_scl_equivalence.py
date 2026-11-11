"""SPS-363: the SCL source text (plc/*.scl), executed by tcutune.scl, matches the
Python reference models.  Requirement IDs refer to DOSSIER.md section 4 (SPS-363)."""

from pathlib import Path

import numpy as np
import pytest

from tcutune import scl
from tcutune.experiments import Scenario, closed_loop
from tcutune.pid import PidParams, ZonePID
from tcutune.plant import PRESETS, Zone
from tcutune.linearise import spec_for

PLC = sorted((Path(__file__).resolve().parents[1] / "plc").glob("*.scl"))


@pytest.fixture(scope="module")
def fbs():
    return scl.load(PLC)


def _set(fb, **kw):
    for k, v in kw.items():
        setattr(fb, k, np.float32(v) if isinstance(v, float) else v)


@pytest.mark.parametrize("td,dead,ratio", [(0.0, 0.0, 1.0), (30.0, 0.0, 2.5), (12.0, 0.5, 0.7)])
def test_pid_bit_exact_random_inputs(fbs, td, dead, ratio):
    """SPS-363: FB_ZonePID == ZonePID in float32, incl. manual, reset, saturation."""
    rng = np.random.default_rng(42)
    prm = PidParams(Gain=18.0, Ti=120.0, Td=td, N=8.0, SpWeight=0.7, Tt=0.0,
                    CoolGainRatio=ratio, Deadband=dead, FfTime=60.0 if td > 0 else 0.0)
    ref = ZonePID(prm, Ts=0.1)
    fb = fbs["FB_ZonePID"]()
    _set(fb, Gain=prm.Gain, Ti=prm.Ti, Td=prm.Td, DerivFilterN=prm.N, SpWeight=prm.SpWeight,
         Tt=prm.Tt, CoolGainRatio=prm.CoolGainRatio, Deadband=prm.Deadband, CycleTime=0.1,
         FfTime=prm.FfTime)
    pv = 40.0
    for k in range(6000):
        sp = 70.0 if k < 2000 else (85.0 if k < 4000 else 60.0)
        pv += rng.normal(0, 0.3) + 0.002 * (sp - pv)
        manual = 2500 <= k < 2700
        mv = float(rng.uniform(-80, 100)) if manual else 0.0
        reset = k in (100, 4500)
        ff = -30.0 if 3000 <= k < 3600 else 0.0
        sat = 3200 <= k < 3500 or 5000 <= k < 5300
        h, c = ref.step(sp, pv, manual, mv, reset, FeedForward=ff, CoolSaturated=sat)
        _set(fb, Setpoint=float(sp), Input=float(pv), ManualEnable=manual,
             ManualValue=float(mv), Reset=reset, FeedForward=float(ff), CoolSaturated=sat)
        fb()
        assert fb.OutHeat == np.float32(h) and fb.OutCool == np.float32(c), k
        assert fb.Output == ref.U and fb.sI == ref.I and fb.sD == ref.D and fb.sF == ref.F, k


@pytest.mark.parametrize("zone", ["A", "B", "D"])
def test_cool_linearise_matches_reference(fbs, zone):
    """FB_CoolLinearise (float32) == linearise.LinSpec (float64) within 0.01 % valve."""
    p = PRESETS[zone]
    rng = np.random.default_rng(3)
    for mode in (0, 1, 2):
        ref = spec_for(p, mode)
        fb = fbs["FB_CoolLinearise"]()
        _set(fb, Mode=mode, DTref=ref.DTref, DTmin=ref.DTmin, UA=ref.UA, CsMax=ref.Cs_max,
             Cp=ref.Cp, ValveR=ref.valve_R, FlowExp=0.8, Gref=0.0)
        worst = 0.0
        for _ in range(400):
            c, tw, tcw = float(rng.uniform(0, 100)), float(rng.uniform(30, 95)), float(rng.uniform(8, 30))
            v, sat = ref.valve(c, tw, tcw)
            _set(fb, CoolIn=c, Tw=tw, Tcw=tcw)
            fb()
            assert bool(fb.Saturated) == sat or abs(float(fb.Valve) - 100.0) < 0.01
            worst = max(worst, abs(float(fb.Valve) - v))
        assert worst < 0.01, (mode, worst)


def test_cool_linearise_holds_tcw_on_sensor_fault(fbs):
    """FMEA item: a failed cooling-water sensor must not move the valve."""
    fb = fbs["FB_CoolLinearise"]()
    _set(fb, Mode=2, CoolIn=40.0, Tw=80.0, Tcw=15.0, TcwValid=True)
    fb()
    v_ok = float(fb.Valve)
    _set(fb, Tcw=-50.0, TcwValid=False)  # broken wire reads far off
    fb()
    assert float(fb.Valve) == v_ok and fb.TcwHeld


def test_cool_linearise_holds_power_constant():
    """Mode 2: the delivered cooling power for a given demand does not depend on
    Tw - Tcw (as long as the exchanger is not saturated)."""
    from tcutune.plant import cool_conductance
    p = PRESETS["B"]
    lin = spec_for(p, 2)
    for c in (10.0, 30.0, 60.0):
        powers = []
        for tw, tcw in ((80.0, 15.0), (60.0, 25.0), (90.0, 8.0)):
            v, sat = lin.valve(c, tw, tcw)
            if not sat:
                powers.append(cool_conductance(p, v) * (tw - tcw))
        assert np.ptp(powers) / np.mean(powers) < 1e-3, (c, powers)


def test_plant_matches_python_model(fbs):
    """SPS-363: FB_ZonePlant (float32) follows plant.Zone (float64) within 0.01 K."""
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
    """SPS-363: FB_Scenario switches SP and load at the same times as Scenario."""
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
    """SPS-363: FB_ZoneTest (whole SCL program) == experiments.closed_loop within 0.05 K."""
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


def test_closed_loop_harness_with_linearisation_and_ff(fbs):
    """SPS-363: FB_ZoneTest with LinMode 2, Tcw 25 degC and feedforward == Python within 0.05 K."""
    from dataclasses import replace
    sc = Scenario().scaled(0.1)
    prm = PidParams(Gain=21.0, Ti=105.0, CoolGainRatio=3.2, FfTime=30.0)
    p = replace(PRESETS["A"], Tcw=25.0)
    ref = closed_loop(p, prm, sc, lin=spec_for(PRESETS["A"], 2), ff_gain=2.0)
    fb = fbs["FB_ZoneTest"]()
    _set(fb, Gain=prm.Gain, Ti=prm.Ti, CoolGainRatio=prm.CoolGainRatio, Restart=True,
         LinMode=2, Tcw=25.0, FfGain=2.0, FfTime=30.0)
    _set(fb.iScenario, tSpHigh=sc.t_sp_high, tLoadOn=sc.t_load_on, tSpLow=sc.t_sp_low,
         tLoadOff=sc.t_load_off, tEnd=sc.t_end)
    pv = np.empty(len(ref.t))
    for k in range(len(ref.t)):
        fb()
        fb.Restart = False
        pv[k] = fb.Pv
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
