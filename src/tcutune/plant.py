"""Thermal model of one zone of a water temperature control unit (TCU).

Two lumped thermal masses:

* the circulating water (heater, cooling valve, pipes)        -> temperature Tw
* the machine part the water flows through (barrel, roll...)  -> temperature Tm

    Cw * dTw/dt = Ph - Pc + UAwm * (Tm - Tw)
    Cm * dTm/dt = UAwm * (Tw - Tm) - UAloss * (Tm - Tamb) + Qproc

    Ph = heat/100 * Pheat_max                     (electric heater, linear)
    Pc = cool/100 * kcool * max(Tw - Tcw, 0)      (cooling valve, bilinear)

The measured temperature is either Tw or Tm, delayed by a transport dead time L
and filtered by a first-order sensor lag tau_s.  Qproc is the heat the process
puts into the machine part (friction heat while the machine runs).

Integration is explicit Euler with the controller sample time (default 0.1 s);
the fastest mode of every preset is > 20 s, so Euler at 0.1 s is accurate to
well below the sensor resolution (checked in tests/test_plant.py).

Everything here is a generic, invented zone.  The parameter values are
plausible orders of magnitude for a water TCU, not data from any real machine.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Callable

import numpy as np


@dataclass(frozen=True)
class ZoneParams:
    name: str
    Cw: float = 63.0  # kJ/K   water + heater + pipes (about 15 l)
    Cm: float = 150.0  # kJ/K   machine part (about 300 kg steel)
    UAwm: float = 1.0  # kW/K   water <-> machine part
    UAloss: float = 0.12  # kW/K   machine part <-> ambient
    Pheat_max: float = 12.0  # kW
    kcool: float = 0.5  # kW/K at 100 % valve
    Tcw: float = 15.0  # degC   cooling water
    Tamb: float = 25.0  # degC
    L: float = 8.0  # s      transport dead time to the sensor
    tau_s: float = 4.0  # s      sensor time constant
    sensor: str = "water"  # "water" (flow line) or "metal" (sensor in the part)
    noise_std: float = 0.0  # K      white measurement noise
    resolution: float = 0.0  # K      quantisation (0 = none), e.g. 0.1
    q_run: float = 9.0  # kW     process heat while the machine runs (puts the zone in cooling)


#: Four invented zones of different size and dynamics ("similar TCUs").
PRESETS: dict[str, ZoneParams] = {
    "A": ZoneParams("A"),
    "B": ZoneParams("B", Cw=42.0, Cm=90.0, UAwm=0.8, UAloss=0.09, Pheat_max=9.0,
                    kcool=0.35, L=6.0, tau_s=3.0, q_run=7.0),
    "C": ZoneParams("C", Cw=84.0, Cm=260.0, UAwm=1.4, UAloss=0.18, Pheat_max=18.0,
                    kcool=0.8, L=12.0, tau_s=5.0, sensor="metal", q_run=14.0),
    "D": ZoneParams("D", Cw=105.0, Cm=420.0, UAwm=2.0, UAloss=0.22, Pheat_max=24.0,
                    kcool=1.1, L=15.0, tau_s=6.0, q_run=17.0),
}


def with_noise(p: ZoneParams, std: float = 0.05, resolution: float = 0.1) -> ZoneParams:
    return replace(p, noise_std=std, resolution=resolution)


@dataclass
class ZoneState:
    Tw: float
    Tm: float
    sensor: float
    delay: np.ndarray  # ring buffer of the undelayed measured temperature
    idx: int = 0


class Zone:
    """Discrete-time simulator of one zone (explicit Euler, step Ts)."""

    def __init__(self, p: ZoneParams, Ts: float = 0.1, T0: float | None = None,
                 seed: int | None = 0):
        self.p = p
        self.Ts = Ts
        self.n_delay = max(1, int(round(p.L / Ts)))
        T0 = p.Tamb if T0 is None else T0
        self.state = ZoneState(Tw=T0, Tm=T0, sensor=T0,
                               delay=np.full(self.n_delay, T0, dtype=float))
        self.rng = np.random.default_rng(seed)

    # -- steady state -------------------------------------------------------
    def steady_state(self, heat: float, cool: float, Qproc: float = 0.0) -> tuple[float, float]:
        """Solve the static balance for (Tw, Tm) with fixed actuator outputs."""
        p = self.p
        # Linear in (Tw, Tm) because Pc is linear in Tw when Tw > Tcw.
        a = cool / 100.0 * p.kcool
        # 0 = Ph - a*(Tw - Tcw) + UAwm*(Tm - Tw)
        # 0 = UAwm*(Tw - Tm) - UAloss*(Tm - Tamb) + Qproc
        Ph = heat / 100.0 * p.Pheat_max
        A = np.array([[-a - p.UAwm, p.UAwm], [p.UAwm, -p.UAwm - p.UAloss]])
        b = np.array([-Ph - a * p.Tcw, -p.UAloss * p.Tamb - Qproc])
        Tw, Tm = np.linalg.solve(A, b)
        return float(Tw), float(Tm)

    def set_steady(self, heat: float, cool: float, Qproc: float = 0.0) -> None:
        Tw, Tm = self.steady_state(heat, cool, Qproc)
        y = Tw if self.p.sensor == "water" else Tm
        self.state = ZoneState(Tw=Tw, Tm=Tm, sensor=y,
                               delay=np.full(self.n_delay, y, dtype=float))

    # -- one step -------------------------------------------------------------
    def step(self, heat: float, cool: float, Qproc: float = 0.0) -> float:
        """Advance one sample with actuator outputs in percent; return PV in degC."""
        p, s, Ts = self.p, self.state, self.Ts
        heat = min(max(heat, 0.0), 100.0)
        cool = min(max(cool, 0.0), 100.0)
        Ph = heat / 100.0 * p.Pheat_max
        Pc = cool / 100.0 * p.kcool * max(s.Tw - p.Tcw, 0.0)
        dTw = (Ph - Pc + p.UAwm * (s.Tm - s.Tw)) / p.Cw
        dTm = (p.UAwm * (s.Tw - s.Tm) - p.UAloss * (s.Tm - p.Tamb) + Qproc) / p.Cm
        s.Tw += Ts * dTw
        s.Tm += Ts * dTm
        raw = s.Tw if p.sensor == "water" else s.Tm
        delayed = s.delay[s.idx]
        s.delay[s.idx] = raw
        s.idx = (s.idx + 1) % self.n_delay
        s.sensor += Ts / (p.tau_s + Ts) * (delayed - s.sensor)
        y = s.sensor
        if p.noise_std > 0:
            y += self.rng.normal(0.0, p.noise_std)
        if p.resolution > 0:
            y = round(y / p.resolution) * p.resolution
        return float(y)


def linear_frf(p: ZoneParams, w: np.ndarray, actuator: str = "heat",
               Tw0: float = 75.0, cool0: float = 0.0) -> np.ndarray:
    """True small-signal frequency response PV/actuator (K per %) at an operating
    point: Tw0 = water temperature, cool0 = cooling valve opening (%).

    Only available in simulation; used to check the tools against the truth.
    """
    a_c = cool0 / 100.0 * p.kcool  # kW/K extra loss through the open valve
    A = np.array([[-(p.UAwm + a_c) / p.Cw, p.UAwm / p.Cw],
                  [p.UAwm / p.Cm, -(p.UAwm + p.UAloss) / p.Cm]])
    if actuator == "heat":
        B = np.array([p.Pheat_max / 100.0 / p.Cw, 0.0])
    else:  # more valve -> more heat removed -> PV falls (negative gain)
        B = np.array([-p.kcool * max(Tw0 - p.Tcw, 0.0) / 100.0 / p.Cw, 0.0])
    C = np.array([1.0, 0.0]) if p.sensor == "water" else np.array([0.0, 1.0])
    s = 1j * np.asarray(w)
    out = np.empty(len(s), dtype=complex)
    eye = np.eye(2)
    for k, sk in enumerate(s):
        out[k] = C @ np.linalg.solve(sk * eye - A, B)
    return out * np.exp(-p.L * s) / (1.0 + p.tau_s * s)


Profile = Callable[[float], float]


def const(v: float) -> Profile:
    return lambda t: v


def steps(*pairs: tuple[float, float], initial: float = 0.0) -> Profile:
    """Piecewise-constant profile: steps((t1, v1), (t2, v2), ...)."""
    pairs_sorted = sorted(pairs)

    def f(t: float) -> float:
        v = initial
        for tk, vk in pairs_sorted:
            if t >= tk:
                v = vk
        return v

    return f


@dataclass
class OpenLoopRun:
    t: np.ndarray
    heat: np.ndarray
    cool: np.ndarray
    pv: np.ndarray
    Qproc: np.ndarray = field(default_factory=lambda: np.zeros(0))


def run_open_loop(zone: Zone, heat: Profile, cool: Profile, duration: float,
                  Qproc: Profile = const(0.0)) -> OpenLoopRun:
    n = int(round(duration / zone.Ts))
    t = np.arange(n) * zone.Ts
    h = np.array([heat(tk) for tk in t])
    c = np.array([cool(tk) for tk in t])
    q = np.array([Qproc(tk) for tk in t])
    pv = np.empty(n)
    for k in range(n):
        pv[k] = zone.step(h[k], c[k], q[k])
    return OpenLoopRun(t, h, c, pv, q)
