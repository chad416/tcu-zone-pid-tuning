"""Closed-loop performance numbers for one scenario run."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .experiments import ClosedLoopRun


@dataclass
class SegmentMetrics:
    event: str
    iae: float  # K*s
    max_dev: float  # K: overshoot past the new SP (SP events) or peak |SP-PV| (load events)
    settle: float  # s to stay within +/- band, nan if never
    tv_heat: float  # %, total variation of the heater output
    tv_cool: float  # %, total variation of the cooling output
    switchovers: int  # heating (u > +h) <-> cooling (u < -h) changes, h = 1 %


def count_switchovers(u: np.ndarray, h: float = 1.0) -> int:
    state, n = 0, 0
    for v in u:
        new = 1 if v > h else (-1 if v < -h else state)
        if state != 0 and new != state:
            n += 1
        state = new
    return n


def _settle_time(t, e, band):
    outside = np.nonzero(np.abs(e) > band)[0]
    if len(outside) == 0:
        return 0.0
    last = outside[-1]
    if last >= len(t) - 1:
        return float("nan")
    return float(t[last + 1] - t[0])


def segments(run: ClosedLoopRun, band: float = 1.0) -> list[SegmentMetrics]:
    ev = run.scenario.events() + [("end", run.scenario.t_end)]
    out = []
    dt = run.t[1] - run.t[0]
    for (name, t0), (_, t1) in zip(ev[:-1], ev[1:]):
        m = (run.t >= t0) & (run.t < t1)
        t, e = run.t[m], run.sp[m] - run.pv[m]
        settle = _settle_time(t, e, band)
        if name in ("cold start", "SP up", "SP down"):
            # overshoot past the new setpoint (the step itself is not a deviation)
            direction = np.sign(e[0]) if abs(e[0]) > 1e-9 else 1.0
            max_dev = float(max(0.0, np.max(-direction * e)))
        else:
            max_dev = float(np.max(np.abs(e)))
        out.append(SegmentMetrics(
            event=name,
            iae=float(np.sum(np.abs(e)) * dt),
            max_dev=max_dev,
            settle=settle,
            tv_heat=float(np.sum(np.abs(np.diff(run.heat[m])))),
            tv_cool=float(np.sum(np.abs(np.diff(run.cool[m])))),
            switchovers=count_switchovers(run.u[m]),
        ))
    return out


def totals(seg: list[SegmentMetrics]) -> dict[str, float]:
    return {
        "iae": sum(s.iae for s in seg),
        "max_dev": max(s.max_dev for s in seg),
        "tv": sum(s.tv_heat + s.tv_cool for s in seg),
        "switchovers": sum(s.switchovers for s in seg),
        "unsettled_segments": sum(1 for s in seg if np.isnan(s.settle)),
    }
