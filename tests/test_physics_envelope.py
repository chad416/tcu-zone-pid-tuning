"""Physics derivation (SPS-201 inputs) and the envelope analysis."""

from dataclasses import replace

import numpy as np
import pytest

from tcutune import envelope as en
from tcutune import physics as ph
from tcutune import plant
from tcutune.linearise import spec_for
from tcutune.pid import PidParams


def test_water_properties_match_table():
    w = ph.water(80.0)
    assert w["rho"] == pytest.approx(971.8)
    assert w["cp"] == pytest.approx(4.197)
    assert w["Pr"] == pytest.approx(2.22, abs=0.02)  # [H1] Table A.6


def test_effectiveness_limits():
    assert ph.eps_counterflow(0.0, 0.5) == 0.0
    assert ph.eps_counterflow(50.0, 0.5) == pytest.approx(1.0, abs=1e-6)
    assert ph.eps_counterflow(1.0, 1.0) == pytest.approx(0.5)
    assert ph.eps_counterflow(1.0, 0.0) == pytest.approx(1 - np.exp(-1.0))


def test_valve_characteristics_end_points_and_shape():
    for R in (0.0, 30.0):
        assert ph.valve_fraction(0.0, R) == pytest.approx(0.0, abs=1e-12)
        assert ph.valve_fraction(100.0, R) == pytest.approx(1.0)
    assert ph.valve_fraction(50.0, 30.0) < 0.2  # equal-percentage is low at mid-travel


def test_gnielinski_turbulent_range():
    h, Re = ph.gnielinski_h(0.65, 0.0133, 2e-4, 80.0)
    assert 3000 < Re < 5e6
    assert 5e3 < h < 5e4  # W/m2K, water in a 13 mm channel at about 3 m/s


def test_derived_presets_are_physical():
    for z, d in ph.PRESET_DESIGNS.items():
        p = plant.PRESETS[z]
        assert p.Cw > d.tcu_water_l * 4.0  # at least the water in the unit
        assert p.Cm == pytest.approx(d.steel_kg * ph.CP_STEEL)
        assert 0.3 < p.UAwm < 5.0
        assert p.L > 0
        # the exchanger can remove the process heat at the test point with margin
        assert plant.cool_conductance(p, 100.0) * (80.0 - 15.0) > 2.0 * p.q_run


def test_cooling_conductance_monotonic():
    for p in plant.PRESETS.values():
        g = [plant.cool_conductance(p, u) for u in np.linspace(0, 100, 51)]
        assert np.all(np.diff(g) > 0)


def test_partials_frf_matches_plant_linearisation():
    p = plant.PRESETS["B"]
    w = np.logspace(-4, 0, 60)
    op = en.op_point(p, 70.0, p.q_run)
    a = en.frf_cool(p, None, op, w)
    b = -plant.linear_frf(p, w, "cool", op.Tw, op.valve)
    assert np.max(np.abs(a - b)) / np.max(np.abs(b)) < 2e-3


def test_fouling_lowers_ua():
    p = plant.PRESETS["A"]
    f = en.fouled(p)
    assert f.hx_UA == pytest.approx(p.hx_UA / (1 + 5000 * 0.000176))


def test_linearised_cooling_gain_does_not_depend_on_cooling_water():
    """With mode 2 the loop gain at crossover is the same in winter and summer."""
    p = plant.PRESETS["A"]
    w = np.logspace(-4, 0, 200)
    lin = spec_for(p, 2)
    gains = []
    for Tcw in (8.0, 15.0, 28.0):
        pt = replace(p, Tcw=Tcw)
        op = en.op_point(pt, 70.0, p.q_run)
        gains.append(abs(en.frf_cool(pt, lin, op, w)[120]))
    assert np.ptp(gains) / np.mean(gains) < 0.02
    # without it the gain moves a lot
    raw = []
    for Tcw in (8.0, 15.0, 28.0):
        pt = replace(p, Tcw=Tcw)
        raw.append(abs(en.frf_cool(pt, None, en.op_point(pt, 70.0, p.q_run), w)[120]))
    assert np.ptp(raw) / np.mean(raw) > 0.3


def test_envelope_evaluate_runs_and_is_bounded():
    p = plant.PRESETS["A"]
    w = np.logspace(-5, 0.5, 400)
    pts = en.grid(p, Tsp=(60, 80), qf=(1.0,), Tcw=(10, 25))
    r = en.evaluate(p, p, PidParams(Gain=20.0, Ti=120.0), en.ALTERNATIVES[4], w, pts)
    assert len(r.ms) == 4
    assert np.all((r.ms > 1.0) & (r.ms < 2.0))
