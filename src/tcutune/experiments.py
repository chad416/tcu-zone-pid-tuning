"""Plant experiments: open-loop step tests, relay test, closed-loop scenario."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .pid import PidParams, ZonePID
from .plant import Zone, ZoneParams, const, run_open_loop, steps


# --------------------------------------------------------------------------
# operating points
# --------------------------------------------------------------------------

def heat_for(p: ZoneParams, T: float, Qproc: float = 0.0) -> float:
    """Heater output (%) that holds the measured temperature at T (no cooling)."""
    z = Zone(p)
    lo, hi = 0.0, 100.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        Tw, Tm = z.steady_state(mid, 0.0, Qproc)
        y = Tw if p.sensor == "water" else Tm
        lo, hi = (mid, hi) if y < T else (lo, mid)
    return 0.5 * (lo + hi)


def cool_for(p: ZoneParams, T: float, Qproc: float) -> float:
    """Cooling-valve output (%) that holds T with the heater off."""
    z = Zone(p)
    lo, hi = 0.0, 100.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        Tw, Tm = z.steady_state(0.0, mid, Qproc)
        y = Tw if p.sensor == "water" else Tm
        lo, hi = (mid, hi) if y > T else (lo, mid)
    return 0.5 * (lo + hi)


# --------------------------------------------------------------------------
# step tests (what a technician would record on the machine)
# --------------------------------------------------------------------------

@dataclass
class StepRecord:
    t: np.ndarray
    heat: np.ndarray
    cool: np.ndarray
    pv: np.ndarray

    def to_csv(self, path: str | Path, every: int = 10) -> None:
        """Write t_s, heat_pct, cool_pct, pv_degC (default 1 s sampling)."""
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_s", "heat_pct", "cool_pct", "pv_degC"])
            for k in range(0, len(self.t), every):
                w.writerow([f"{self.t[k]:.1f}", f"{self.heat[k]:.2f}",
                            f"{self.cool[k]:.2f}", f"{self.pv[k]:.2f}"])

    @staticmethod
    def from_csv(path: str | Path) -> "StepRecord":
        data = np.genfromtxt(path, delimiter=",", names=True)
        return StepRecord(np.asarray(data["t_s"]), np.asarray(data["heat_pct"]),
                          np.asarray(data["cool_pct"]), np.asarray(data["pv_degC"]))


def heat_step_test(p: ZoneParams, T_op: float = 75.0, du: float = 10.0,
                   pre: float = 120.0, duration: float = 3600.0, Ts: float = 0.1,
                   seed: int = 1) -> StepRecord:
    """Zone at steady state at T_op (heater only), heater stepped by +du %."""
    u0 = heat_for(p, T_op)
    z = Zone(p, Ts=Ts, seed=seed)
    z.set_steady(u0, 0.0)
    r = run_open_loop(z, steps((pre, u0 + du), initial=u0), const(0.0), duration)
    return StepRecord(r.t, r.heat, r.cool, r.pv)


def cool_step_test(p: ZoneParams, T_op: float = 80.0, Qproc: float | None = None,
                   du: float = 4.0, pre: float = 120.0, duration: float = 3600.0,
                   Ts: float = 0.1, seed: int = 2) -> StepRecord:
    """Machine running (Qproc, default p.q_run), heater off, valve stepped by +du %."""
    Qproc = p.q_run if Qproc is None else Qproc
    c0 = cool_for(p, T_op, Qproc)
    z = Zone(p, Ts=Ts, seed=seed)
    z.set_steady(0.0, c0, Qproc)
    r = run_open_loop(z, const(0.0), steps((pre, c0 + du), initial=c0), duration,
                      Qproc=const(Qproc))
    return StepRecord(r.t, r.heat, r.cool, r.pv)


def relay_test(p: ZoneParams, sp: float = 75.0, d: float = 25.0, hyst: float = 0.2,
               duration: float = 2400.0, Ts: float = 0.1, seed: int = 3) -> StepRecord:
    """Heater relay u0 +/- d around the setpoint, with hysteresis (Astrom-Hagglund)."""
    u0 = heat_for(p, sp)
    z = Zone(p, Ts=Ts, seed=seed)
    z.set_steady(u0, 0.0)
    n = int(round(duration / Ts))
    t = np.arange(n) * Ts
    h = np.empty(n)
    pv = np.empty(n)
    y = sp
    high = True
    for k in range(n):
        if high and y > sp + hyst:
            high = False
        elif not high and y < sp - hyst:
            high = True
        h[k] = u0 + d if high else u0 - d
        y = z.step(h[k], 0.0)
        pv[k] = y
    return StepRecord(t, h, np.zeros(n), pv)


# --------------------------------------------------------------------------
# closed loop
# --------------------------------------------------------------------------

@dataclass
class Scenario:
    sp_start: float = 70.0
    sp_high: float = 80.0
    sp_low: float = 75.0
    q_run: float = 9.0
    t_sp_high: float = 3000.0
    t_load_on: float = 5400.0
    t_sp_low: float = 8400.0
    t_load_off: float = 10200.0
    t_end: float = 12000.0

    def events(self) -> list[tuple[str, float]]:
        return [("cold start", 0.0), ("SP up", self.t_sp_high), ("load on", self.t_load_on),
                ("SP down", self.t_sp_low), ("load off", self.t_load_off)]

    def scaled(self, k: float) -> "Scenario":
        """Same profile with every time multiplied by k (for fast tests)."""
        return Scenario(self.sp_start, self.sp_high, self.sp_low, self.q_run,
                        self.t_sp_high * k, self.t_load_on * k, self.t_sp_low * k,
                        self.t_load_off * k, self.t_end * k)


def production_scenario(p: ZoneParams | None = None) -> Scenario:
    """Cold start to 70, SP 80, process load on (cooling), SP 75, load off.
    The load is the zone's q_run (zone A: 9 kW = the FB_Scenario default)."""
    return Scenario(q_run=p.q_run) if p is not None else Scenario()


@dataclass
class ClosedLoopRun:
    t: np.ndarray
    sp: np.ndarray
    pv: np.ndarray
    heat: np.ndarray
    cool: np.ndarray
    u: np.ndarray
    q: np.ndarray
    scenario: Scenario


def closed_loop(p: ZoneParams, prm: PidParams, sc: Scenario | None = None,
                Ts: float = 0.1, seed: int = 4) -> ClosedLoopRun:
    """Same order as FB_ZoneTest: scenario -> controller(last PV) -> plant."""
    sc = sc or production_scenario(p)
    z = Zone(p, Ts=Ts, T0=25.0, seed=seed)
    pid = ZonePID(prm, Ts=Ts)
    n = int(round(sc.t_end / Ts))
    t = np.arange(n) * Ts
    sp = np.where(t >= sc.t_sp_low, sc.sp_low, np.where(t >= sc.t_sp_high, sc.sp_high, sc.sp_start))
    q = np.where((t >= sc.t_load_on) & (t < sc.t_load_off), sc.q_run, 0.0)
    pv, heat, cool, u = (np.empty(n) for _ in range(4))
    y = 25.0
    for k in range(n):
        h, c = pid.step(sp[k], y)
        heat[k], cool[k], u[k] = h, c, float(pid.U)
        y = z.step(h, c, q[k])
        pv[k] = y
    return ClosedLoopRun(t, sp, pv, heat, cool, u, q, sc)
