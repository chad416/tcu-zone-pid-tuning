"""Thermal model of one zone of a water temperature control unit (TCU).

Two lumped thermal masses:

* the circulating water of the closed primary circuit   -> temperature Tw
* the machine part the water flows through (barrel, screw, head, mixer wall)
                                                          -> temperature Tm

    Cw * dTw/dt = Ph - Pc + UAwm * (Tm - Tw)
    Cm * dTm/dt = UAwm * (Tw - Tm) - UAloss * (Tm - Tamb) + Qproc

    Ph = heat/100 * Pheat_max                   (electric heater, linear)
    Pc = G(cool) * max(Tw - Tcw, 0)             (cooling)

G(cool) is the cooling conductance in kW/K.  For a plate heat exchanger in the
closed primary circuit (hx_UA > 0) it is eps*Cmin from effectiveness-NTU with
the valve characteristic in front of it (physics.hx_conductance); with
hx_UA = 0 it is the bilinear "direct cooling" law cool/100 * kcool.  Either
way Pc is linear in Tw for a fixed valve, so the steady state stays a linear
solve.

The measured temperature is either Tw or Tm, delayed by a transport dead time L
and filtered by a first-order sensor lag tau_s.  Qproc is the heat the process
puts into the machine part (viscous heating of the compound while it runs).

Integration is explicit Euler with the controller sample time (default 0.1 s);
the fastest mode of every preset is > 20 s, so Euler at 0.1 s is accurate to
well below the sensor resolution (checked in tests).

The presets are derived in physics.py from the hardware (heater, pump, hoses,
exchanger, channel geometry, steel mass) with textbook heat-transfer relations.
The hardware values themselves are sizing assumptions for this machine class,
not data from a real line.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Callable

import numpy as np

from . import physics


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
    # plate heat exchanger in the closed primary circuit (hx_UA = 0: direct cooling, kcool)
    hx_UA: float = 0.0  # kW/K   overall UA at full cooling-water flow
    Cs_max: float = 1.0  # kW/K   cooling-water capacity rate at full flow
    Cp: float = 2.7  # kW/K   primary (pump) capacity rate
    valve_R: float = 0.0  # 0 linear (PWM solenoid), > 1 equal-percentage rangeability


def from_design(d: physics.ZoneDesign) -> ZoneParams:
    """Lumped parameters from the hardware description (physics.derive)."""
    q = physics.derive(d).params
    return ZoneParams(d.name, Cw=q["Cw"], Cm=q["Cm"], UAwm=q["UAwm"], UAloss=q["UAloss"],
                      Pheat_max=q["Pheat_max"], kcool=0.0, Tcw=q["Tcw"], L=q["L"],
                      tau_s=q["tau_s"], sensor=d.sensor, q_run=q["q_run"], hx_UA=q["hx_UA"],
                      Cs_max=q["Cs_max"], Cp=q["Cp"], valve_R=q["valve_R"])


#: Four zones of a rubber-compound line served by similar TCUs (physics.PRESET_DESIGNS).
PRESETS: dict[str, ZoneParams] = {k: from_design(d) for k, d in physics.PRESET_DESIGNS.items()}


def cool_conductance(p: ZoneParams, cool: float) -> float:
    """G(cool) in kW/K: heat removed = G * (Tw - Tcw)."""
    cool = min(max(cool, 0.0), 100.0)
    if p.hx_UA <= 0.0:
        return cool / 100.0 * p.kcool
    return physics.hx_conductance(physics.valve_fraction(cool, p.valve_R), p.hx_UA, p.Cs_max, p.Cp)


def cool_conductance_slope(p: ZoneParams, cool: float, h: float = 0.25) -> float:
    """dG/d(cool) in kW/K per %, central difference (one-sided at the ends)."""
    lo, hi = max(cool - h, 0.0), min(cool + h, 100.0)
    return (cool_conductance(p, hi) - cool_conductance(p, lo)) / (hi - lo)


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
        a = cool_conductance(p, cool)
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
    def step(self, heat: float, cool: float, Qproc: float = 0.0,
             Tcw: float | None = None) -> float:
        """Advance one sample with actuator outputs in percent; return PV in degC.
        Tcw overrides the cooling-water temperature of the preset (seasons)."""
        p, s, Ts = self.p, self.state, self.Ts
        heat = min(max(heat, 0.0), 100.0)
        cool = min(max(cool, 0.0), 100.0)
        tcw = p.Tcw if Tcw is None else Tcw
        Ph = heat / 100.0 * p.Pheat_max
        Pc = cool_conductance(p, cool) * max(s.Tw - tcw, 0.0)
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
    a_c = cool_conductance(p, cool0)  # kW/K extra loss through the open valve
    A = np.array([[-(p.UAwm + a_c) / p.Cw, p.UAwm / p.Cw],
                  [p.UAwm / p.Cm, -(p.UAwm + p.UAloss) / p.Cm]])
    if actuator == "heat":
        B = np.array([p.Pheat_max / 100.0 / p.Cw, 0.0])
    else:  # more valve -> more heat removed -> PV falls (negative gain)
        B = np.array([-cool_conductance_slope(p, cool0) * max(Tw0 - p.Tcw, 0.0) / p.Cw, 0.0])
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
