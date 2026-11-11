"""Robustness of one tuning over the whole operating envelope of a zone.

A TCU zone is tuned at one test point (setpoint, process load, cooling-water
temperature, clean exchanger).  In production the cooling path moves away
from that point in three ways:

* Tw - Tcw changes with the setpoint and with the cooling-water temperature
  over the year: the heat removed per % of valve scales with it;
* the valve and the exchanger are nonlinear in the opening (valve
  characteristic, effectiveness-NTU), so the slope dPc/dvalve changes with load;
* the plate exchanger fouls between cleanings, which lowers its UA.

This module linearises the true (simulated) zone at any operating point,
including the cooling linearisation of linearise.py in front of the valve, and
returns the frequency response the PID sees.  Only a simulation can do this;
the tool itself only sees step-test records.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from . import experiments as ex
from . import plant
from .linearise import LinSpec
from .pid import PidParams
from .tuning import controller_frf, crossover_ratio, max_sensitivity

#: TEMA fouling resistance for treated cooling-tower water [H3]; clean plate U assumed.
RF_COOLING_TOWER = 0.000176  # m2K/W
U_CLEAN_PLATE = 5000.0  # W/m2K, water-water plate exchanger, clean (assumed)


def fouled(p: plant.ZoneParams, Rf: float = RF_COOLING_TOWER, U_clean: float = U_CLEAN_PLATE,
           ) -> plant.ZoneParams:
    """Same zone with the exchanger UA reduced by a fouling resistance Rf."""
    return replace(p, hx_UA=p.hx_UA / (1.0 + U_clean * Rf))


@dataclass
class OpPoint:
    Tsp: float
    q: float
    Tcw: float
    valve: float  # % physical valve opening holding Tsp, heater off
    Tw: float
    feasible: bool


def op_point(p: plant.ZoneParams, Tsp: float, q: float) -> OpPoint:
    v = ex.cool_for(p, Tsp, q)
    Tw, _ = plant.Zone(p).steady_state(0.0, v, q)
    ok = 0.5 < v < 99.5
    return OpPoint(Tsp, q, p.Tcw, v, Tw, ok)


def _pc(p: plant.ZoneParams, lin: LinSpec | None, c: float, Tw: float) -> float:
    v = c if lin is None else lin.valve(c, Tw, p.Tcw)[0]
    return plant.cool_conductance(p, v) * max(Tw - p.Tcw, 0.0)


def demand_for(p: plant.ZoneParams, lin: LinSpec | None, op: OpPoint) -> float:
    """Cooling demand c (%) that gives the physical valve opening of the op point."""
    if lin is None or lin.mode == 0:
        return op.valve
    target = plant.cool_conductance(p, op.valve) * (op.Tw - p.Tcw)
    lo, hi = 0.0, 100.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if _pc(p, lin, mid, op.Tw) < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def actuator_partials(p: plant.ZoneParams, lin: LinSpec | None, op: OpPoint,
                      h: float = 0.5, hT: float = 0.5) -> tuple[float, float]:
    """(dPc/dc in kW per %, dPc/dTw in kW/K) of the composite actuator.

    Central differences with steps of 0.5 % and 0.5 K: large enough that the
    bisection resolution of the linearisation (2^-15 of the valve travel) does
    not show up as noise, small enough to stay local."""
    c0 = demand_for(p, lin, op)
    lo, hi = max(c0 - h, 0.0), min(c0 + h, 100.0)
    b = (_pc(p, lin, hi, op.Tw) - _pc(p, lin, lo, op.Tw)) / (hi - lo)
    a = (_pc(p, lin, c0, op.Tw + hT) - _pc(p, lin, c0, op.Tw - hT)) / (2.0 * hT)
    return b, a


def _two_mass(p: plant.ZoneParams, a: float, w: np.ndarray, b_water: float,
              b_metal: float) -> np.ndarray:
    """C (sI - A)^-1 B for the two-mass zone, closed form (vectorised over w)."""
    a11, a12 = -(p.UAwm + a) / p.Cw, p.UAwm / p.Cw
    a21, a22 = p.UAwm / p.Cm, -(p.UAwm + p.UAloss) / p.Cm
    s = 1j * np.asarray(w)
    det = (s - a11) * (s - a22) - a12 * a21
    x_water = ((s - a22) * b_water + a12 * b_metal) / det
    x_metal = (a21 * b_water + (s - a11) * b_metal) / det
    y = x_water if p.sensor == "water" else x_metal
    return y * np.exp(-p.L * s) / (1.0 + p.tau_s * s)


def frf_from_partials(p: plant.ZoneParams, b: float, a: float, w: np.ndarray) -> np.ndarray:
    """PV response (K per % of cooling demand, sign made positive) for a linear
    actuator Pc = b*c + a*Tw around the operating point."""
    return _two_mass(p, a, w, b / p.Cw, 0.0)


def frf_cool(p: plant.ZoneParams, lin: LinSpec | None, op: OpPoint, w: np.ndarray) -> np.ndarray:
    b, a = actuator_partials(p, lin, op)
    return frf_from_partials(p, b, a, w)


def frf_heat(p: plant.ZoneParams, w: np.ndarray, Tw0: float = 75.0) -> np.ndarray:
    return plant.linear_frf(p, w, "heat", Tw0)


def frf_load(p: plant.ZoneParams, lin: LinSpec | None, op: OpPoint, w: np.ndarray) -> np.ndarray:
    """PV response (K per kW) to process heat into the machine part, open loop
    at the cooling operating point."""
    _, a = actuator_partials(p, lin, op)
    return _two_mass(p, a, w, 0.0, 1.0 / p.Cm)


@dataclass(frozen=True)
class Alternative:
    """One way of running the cooling path (AoA alternative)."""
    key: str
    label: str
    ratio: str  # "one", "static", "crossover"
    lin_mode: int  # 0, 1, 2


ALTERNATIVES = (
    Alternative("A", "one gain set (ratio 1)", "one", 0),
    Alternative("B", "static gain ratio", "static", 0),
    Alternative("C", "crossover ratio (tool v0.1)", "crossover", 0),
    Alternative("D", "crossover ratio + Tw-Tcw scaling", "crossover", 1),
    Alternative("E", "crossover ratio + exchanger/valve inversion", "crossover", 2),
)

TEST_POINT = dict(Tsp=80.0, Tcw=15.0)  # identification test: q = q_run, clean exchanger


def ratio_at_test(p: plant.ZoneParams, prm: PidParams, alt: Alternative, w: np.ndarray,
                  lin: LinSpec | None) -> float:
    """The CoolGainRatio the tool sets for this alternative, from the test point.

    Uses the true linear models at the test point (the study also runs the
    identification route; the two agree within the identification error)."""
    if alt.ratio == "one":
        return 1.0
    pt = replace(p, Tcw=TEST_POINT["Tcw"])
    op = op_point(pt, TEST_POINT["Tsp"], p.q_run)
    Gc = frf_cool(pt, lin, op, w)
    Gh = frf_heat(pt, w)
    if alt.ratio == "static":
        return float(abs(Gc[0]) / abs(Gh[0]))
    return crossover_ratio(prm, Gh, Gc, w)


@dataclass
class EnvelopeResult:
    alt: str
    points: list[tuple[float, float, float]]  # (Tsp, q, Tcw)
    ms: np.ndarray
    ratio: float


def grid(p: plant.ZoneParams, Tsp=(50, 60, 70, 80, 90), qf=(0.5, 0.75, 1.0, 1.25, 1.5),
         Tcw=(8, 12, 15, 20, 25, 30)) -> list[tuple[float, float, float]]:
    return [(float(a), float(b) * p.q_run, float(c)) for c in Tcw for a in Tsp for b in qf]


def evaluate(p_true: plant.ZoneParams, p_model: plant.ZoneParams, prm: PidParams,
             alt: Alternative, w: np.ndarray, pts: list[tuple[float, float, float]],
             DTref: float = 65.0) -> EnvelopeResult:
    """Ms in cooling at every feasible grid point.

    p_true: the plant (possibly fouled); p_model: the nameplate data the
    linearisation and the tuning were made with (clean)."""
    from .linearise import spec_for
    lin = None if alt.lin_mode == 0 else spec_for(p_model, alt.lin_mode, DTref)
    r = ratio_at_test(p_model, prm, alt, w, lin)
    prm_r = replace(prm, CoolGainRatio=r)
    ms, used = [], []
    for Tsp, q, Tcw in pts:
        pt = replace(p_true, Tcw=Tcw)
        op = op_point(pt, Tsp, q)
        if not op.feasible:
            continue
        c0 = demand_for(pt, lin, op)
        if c0 >= 99.5 or (lin is not None and lin.valve(c0, op.Tw, Tcw)[1]):
            continue  # demand saturated: a capacity limit, not a tuning question
        G = frf_cool(pt, lin, op, w)
        ms.append(max_sensitivity(prm_r, G / r, w))
        used.append((Tsp, q, Tcw))
    return EnvelopeResult(alt.key, used, np.array(ms), r)


def sensitivity_mag(prm: PidParams, G: np.ndarray, w: np.ndarray) -> np.ndarray:
    return np.abs(1.0 / (1.0 + controller_frf(prm, w) * G))
