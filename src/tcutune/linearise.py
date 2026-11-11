"""Cooling-actuator linearisation (reference model of plc/FB_CoolLinearise.scl).

The PID's cooling output c (0..100 %) is a request for cooling *power*.  How
much power a valve opening delivers depends on the valve characteristic, on
the exchanger (effectiveness-NTU) and on the driving temperature difference
Tw - Tcw, which moves with the setpoint and with the cooling-water temperature
over the year.  The linearisation makes the delivered power independent of
both, so the loop gain identified at one test point holds everywhere:

    Mode 0  valve = c                                       (no linearisation)
    Mode 1  valve = c * DTref / max(Tw - Tcw, DTmin)        (temperature only)
    Mode 2  G(valve) = c/100 * Gref * DTref / max(Tw - Tcw, DTmin)
            solved for the valve with the exchanger + valve model
            (bisection, 16 steps)                           (model inversion)

Gref is the modelled conductance at 100 % valve (from the nameplate data);
DTref is Tw - Tcw at the identification test.  Below DTmin the scaling is
frozen so that a cold start with Tw close to Tcw cannot blow the gain up.

This Python version is float64; tests/test_scl_equivalence.py checks the SCL
(float32, translated) against it.
"""

from __future__ import annotations

from dataclasses import dataclass

from . import physics


@dataclass(frozen=True)
class LinSpec:
    mode: int = 0  # 0 off, 1 temperature scaling, 2 model inversion
    DTref: float = 65.0  # K, Tw - Tcw at the identification test
    DTmin: float = 5.0  # K
    UA: float = 0.9  # kW/K, exchanger model (nameplate)
    Cs_max: float = 1.39  # kW/K
    Cp: float = 2.72  # kW/K
    valve_R: float = 0.0
    n: float = 0.8
    Gref: float = 0.0  # kW/K, <= 0: the model's G at 100 %

    def G(self, valve: float) -> float:
        return physics.hx_conductance(physics.valve_fraction(valve, self.valve_R),
                                      self.UA, self.Cs_max, self.Cp, self.n)

    def valve(self, c: float, Tw: float, Tcw: float) -> tuple[float, bool]:
        """Valve opening (%) for cooling demand c (%); second value: saturated."""
        c = min(max(c, 0.0), 100.0)
        if self.mode == 0:
            return c, False
        dT = max(Tw - Tcw, self.DTmin)
        if self.mode == 1:
            v = c * self.DTref / dT
            return (100.0, True) if v >= 100.0 else (v, False)
        gmax = self.G(100.0)
        gref = self.Gref if self.Gref > 0 else gmax
        greq = c / 100.0 * gref * self.DTref / dT
        if greq >= gmax:
            return 100.0, True
        lo, hi = 0.0, 1.0
        for _ in range(15):
            mid = 0.5 * (lo + hi)
            if self.G(100.0 * mid) < greq:
                lo = mid
            else:
                hi = mid
        return 100.0 * 0.5 * (lo + hi), False


def spec_for(p, mode: int, DTref: float = 65.0, ua_scale: float = 1.0) -> LinSpec:
    """Linearisation spec from a zone's own (nameplate) exchanger data.

    ua_scale lets the study give the model a different UA from the plant
    (e.g. the plant fouled, the model still clean)."""
    return LinSpec(mode=mode, DTref=DTref, UA=p.hx_UA * ua_scale, Cs_max=p.Cs_max, Cp=p.Cp,
                   valve_R=p.valve_R)
