"""SPS-202/203 (tool), SPS-201 inputs: plant, identification and tuning against known truths."""

import numpy as np
import pytest

from tcutune import experiments as ex
from tcutune import identify as idf
from tcutune import plant
from tcutune import tuning as tu


def test_steady_state_matches_long_simulation():
    p = plant.PRESETS["A"]
    z = plant.Zone(p, Ts=0.1)
    z.set_steady(40.0, 0.0)
    for _ in range(36000):  # 1 h at the same input: no drift
        y = z.step(40.0, 0.0)
    Tw, _ = z.steady_state(40.0, 0.0)
    assert abs(y - Tw) < 1e-6


def test_euler_step_is_accurate():
    p = plant.PRESETS["B"]
    a, b = plant.Zone(p, Ts=0.1), plant.Zone(p, Ts=0.01)
    for k in range(6000):
        ya = a.step(80.0, 0.0)
        for _ in range(10):
            yb = b.step(80.0, 0.0)
    assert abs(ya - yb) < 0.02


def test_linear_frf_dc_gain():
    p = plant.PRESETS["C"]
    z = plant.Zone(p)
    g_static = (z.steady_state(51.0, 0)[1] - z.steady_state(49.0, 0)[1]) / 2.0
    g = plant.linear_frf(p, np.array([1e-9]), "heat")[0]
    assert abs(g.real - g_static) < 1e-6


def test_fopdt_fit_recovers_true_model():
    true = idf.Fopdt(K=1.5, T=300.0, L=40.0)
    t = np.arange(0, 2400, 1.0)
    rng = np.random.default_rng(0)
    dy = true.step(t, 10.0) + rng.normal(0, 0.05, len(t))
    m = idf.fit_fopdt(idf.StepData(t, dy, 10.0, 60.0))
    assert m.K == pytest.approx(1.5, rel=0.02)
    assert m.T == pytest.approx(300.0, rel=0.05)
    assert m.L == pytest.approx(40.0, abs=3.0)


def test_lead_lag_fit_beats_fopdt_on_tcu_zone():
    p = plant.with_noise(plant.PRESETS["A"])
    rec = ex.heat_step_test(p, duration=3600)
    sd = idf.find_step(rec.t, rec.heat, rec.pv)
    f = idf.fit_fopdt(sd)
    m = idf.fit_lead_lag(sd, f)
    assert m.rmse < 0.6 * f.rmse
    assert m.L == pytest.approx(p.L + p.tau_s, abs=6.0)  # dead time is visible again


def _ultimate(p):
    w = np.logspace(-4, 0, 20000)
    G = plant.linear_frf(p, w, "heat")
    k = int(np.argmax(np.unwrap(np.angle(G)) < -np.pi))
    return 1.0 / abs(G[k]), 2 * np.pi / w[k]


def test_relay_finds_ultimate_point_without_noise():
    p = plant.PRESETS["D"]
    rec = ex.relay_test(p, hyst=0.0)
    r = idf.relay_analysis(rec.t, rec.heat, rec.pv, hyst=0.0)
    ku, pu = _ultimate(p)
    assert r.Ku == pytest.approx(ku, rel=0.10)  # describing-function error
    assert r.Pu == pytest.approx(pu, rel=0.05)


def test_relay_with_noise_and_hysteresis_is_conservative():
    """Known limitation (README): with 0.1 K resolution the oscillation is only
    ~1 K, the 0.2 K hysteresis needed against noise moves the measured point
    away from -180 deg, and Ku comes out low.  Low Ku -> conservative tuning."""
    p = plant.with_noise(plant.PRESETS["D"])
    rec = ex.relay_test(p, hyst=0.2)
    r = idf.relay_analysis(rec.t, rec.heat, rec.pv, hyst=0.2)
    ku, pu = _ultimate(p)
    assert 0.3 * ku < r.Ku < ku
    assert r.Pu > pu


def test_rules_reject_missing_dead_time():
    m = idf.Fopdt(K=1.0, T=1600.0, L=0.0)
    for rule in (tu.zn_step, tu.simc_pi, tu.amigo_pi):
        with pytest.raises(tu.NotApplicable):
            rule(m)
    assert tu.lambda_pi(m).Gain > 0


def test_textbook_values():
    m = idf.Fopdt(K=2.0, T=100.0, L=10.0)
    zn = tu.zn_step(m)
    assert (zn.Gain, zn.Ti, zn.Td) == pytest.approx((6.0, 20.0, 5.0))
    s = tu.simc_pi(m)
    assert (s.Gain, s.Ti) == pytest.approx((2.5, 80.0))


def test_simc_ms_meets_target_on_model():
    m = idf.LeadLag(K=1.1, T1=1800.0, T2=45.0, Ta=135.0, L=11.0)
    prm, tau_c = tu.simc_ms(m, np.logspace(-5, 0.5, 3000), 1.4)
    assert tu.max_sensitivity(prm, m.frf(np.logspace(-5, 0.5, 3000)), np.logspace(-5, 0.5, 3000)) <= 1.4
    assert tau_c >= m.L


def test_crossover_ratio_equals_static_for_same_dynamics():
    w = np.logspace(-5, 0.5, 3000)
    h = idf.LeadLag(K=1.0, T1=1500.0, T2=40.0, Ta=100.0, L=10.0)
    c = idf.LeadLag(K=2.0, T1=1500.0, T2=40.0, Ta=100.0, L=10.0)
    r = tu.crossover_ratio(tu.simc_ms(h, w)[0], h.frf(w), c.frf(w), w)
    assert r == pytest.approx(2.0, rel=1e-6)
