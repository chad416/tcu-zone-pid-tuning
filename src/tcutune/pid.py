"""Reference model of plc/FB_ZonePID.scl in IEEE single precision.

Every arithmetic step is done in float32 and in the same order as the SCL
source, so the two can be compared sample by sample
(tests/test_scl_equivalence.py).  If you change one, change the other.

Controller (one zone, split range):

    P  = Gain * (SpWeight*Sp - Pv)
    D  = a*D - Gain*N*a*(Pv - PvPrev),        a = Td / (Td + N*Ts)
    u0 = P + I + D
    u  = LIMIT(OutLo, u0, 100),               OutLo = -100 * CoolGainRatio
    I += Gain*Ts/Ti*(Sp - Pv) + Ts/Tt*(u - u0)          (back-calculation)
    u >= 0 -> Heat = u,  Cool = 0
    u <  0 -> Heat = 0,  Cool = -u / CoolGainRatio

u is in "heating-equivalent percent".  CoolGainRatio = Kcool / Kheat (process
gains, K per % of each actuator).  Dividing the cooling output by it gives the
loop the same gain in cooling as in heating; OutLo keeps 100 % cooling
reachable.  CoolGainRatio = 1 is the "one set of gains for both" case.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

f32 = np.float32


@dataclass
class PidParams:
    Gain: float  # %/K
    Ti: float  # s, 0 = no integral action
    Td: float = 0.0  # s, 0 = no derivative action
    N: float = 10.0  # derivative filter: time constant Td/N
    SpWeight: float = 1.0  # setpoint weight b of the P part
    Tt: float = 0.0  # s, anti-windup tracking time, <= 0 -> Ti
    CoolGainRatio: float = 1.0  # Kcool / Kheat
    Deadband: float = 0.0  # %, |u| below this -> both outputs 0


class ZonePID:
    """float32 twin of FUNCTION_BLOCK "FB_ZonePID"."""

    def __init__(self, prm: PidParams, Ts: float = 0.1):
        self.prm = prm
        self.Ts = f32(Ts)
        self.I = f32(0.0)
        self.D = f32(0.0)
        self.PvPrev = f32(0.0)
        self.Initialised = False
        self.U = f32(0.0)
        self.Heat = f32(0.0)
        self.Cool = f32(0.0)

    def reset(self) -> None:
        self.I = f32(0.0)
        self.D = f32(0.0)
        self.Initialised = False

    def step(self, Sp: float, Pv: float, Manual: bool = False,
             ManualValue: float = 0.0, Reset: bool = False) -> tuple[float, float]:
        p = self.prm
        Sp, Pv, ManualValue = f32(Sp), f32(Pv), f32(ManualValue)
        Gain, Ti, Td, N = f32(p.Gain), f32(p.Ti), f32(p.Td), f32(p.N)
        SpWeight, Tt, Ratio, Deadband = (f32(p.SpWeight), f32(p.Tt),
                                         f32(p.CoolGainRatio), f32(p.Deadband))
        Ts = self.Ts

        if Reset:
            self.I = f32(0.0)
            self.D = f32(0.0)
            self.Initialised = False
            self.U, self.Heat, self.Cool = f32(0.0), f32(0.0), f32(0.0)
            return 0.0, 0.0

        if not self.Initialised:
            self.PvPrev = Pv
            self.Initialised = True

        if Ratio < f32(0.01):
            Ratio = f32(0.01)
        OutLo = f32(-100.0) * Ratio
        if Tt <= f32(0.0):
            Tt = Ti

        P = Gain * (SpWeight * Sp - Pv)

        if Td > f32(0.0):
            a = Td / (Td + N * Ts)
            self.D = a * self.D - Gain * N * a * (Pv - self.PvPrev)
        else:
            self.D = f32(0.0)

        U0 = P + self.I + self.D
        if Manual:
            U = min(max(ManualValue, OutLo), f32(100.0))
            self.I = U - P - self.D  # bumpless return to auto
        else:
            U = min(max(U0, OutLo), f32(100.0))
            if Ti > f32(0.0):
                self.I = self.I + Gain * Ts / Ti * (Sp - Pv) + Ts / Tt * (U - U0)
            else:
                self.I = f32(0.0)

        if U >= Deadband:
            Heat, Cool = U, f32(0.0)
        elif U <= -Deadband:
            Heat, Cool = f32(0.0), min(-U / Ratio, f32(100.0))
        else:
            Heat, Cool = f32(0.0), f32(0.0)

        self.PvPrev = Pv
        self.U, self.Heat, self.Cool = U, Heat, Cool
        return float(Heat), float(Cool)
