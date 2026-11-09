"""Process identification from recorded test data.

* FOPDT  G(s) = K e^{-Ls} / (1 + T s)   from an open-loop step
    - two-point method (Smith: 28.3 % and 63.2 % of the change)
    - least-squares fit of the step response, started from the two-point result
* relay test (Astrom-Hagglund): ultimate gain Ku and period Pu
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares


@dataclass
class Fopdt:
    K: float  # process gain, K per % of the stepped actuator
    T: float  # s
    L: float  # s
    rmse: float = float("nan")  # K, fit residual
    method: str = ""

    def step(self, t: np.ndarray, du: float = 1.0) -> np.ndarray:
        tt = np.clip(t - self.L, 0.0, None)
        return self.K * du * (1.0 - np.exp(-tt / self.T))


@dataclass
class StepData:
    t: np.ndarray  # s, time since the step
    dy: np.ndarray  # K, PV change since the step
    du: float  # %, actuator change (signed)
    y0: float  # PV before the step


def find_step(t: np.ndarray, u: np.ndarray, y: np.ndarray, pre_window: float = 60.0) -> StepData:
    """Locate the single largest actuator step and return the response after it."""
    du_all = np.diff(u)
    k = int(np.argmax(np.abs(du_all))) + 1
    if abs(du_all[k - 1]) < 1e-6:
        raise ValueError("no actuator step found in the record")
    t_step = t[k]
    pre = (t >= t_step - pre_window) & (t < t_step)
    if not pre.any():
        raise ValueError("need data before the step to know the starting PV")
    y0 = float(np.mean(y[pre]))
    du = float(np.mean(u[k:k + 5]) - np.mean(u[pre]))
    post = t >= t_step
    return StepData(t[post] - t_step, y[post] - y0, du, y0)


def two_point(sd: StepData, tail: float = 60.0) -> Fopdt:
    """Smith's method; the final value is the mean over the last `tail` seconds."""
    end = sd.t >= sd.t[-1] - tail
    dy_inf = float(np.mean(sd.dy[end]))
    if abs(dy_inf) < 1e-9:
        raise ValueError("PV did not change")
    frac = sd.dy / dy_inf
    t28 = _first_crossing(sd.t, frac, 0.283)
    t63 = _first_crossing(sd.t, frac, 0.632)
    T = 1.5 * (t63 - t28)
    L = max(t63 - T, 0.0)
    return Fopdt(K=dy_inf / sd.du, T=max(T, 1e-3), L=L, method="two-point")


def _first_crossing(t: np.ndarray, x: np.ndarray, level: float) -> float:
    idx = np.nonzero(x >= level)[0]
    if len(idx) == 0:
        raise ValueError(f"response never reached {level:.1%} of its final value")
    i = int(idx[0])
    if i == 0:
        return float(t[0])
    x0, x1 = x[i - 1], x[i]
    return float(t[i - 1] + (level - x0) / (x1 - x0) * (t[i] - t[i - 1]))


def fit_fopdt(sd: StepData, start: Fopdt | None = None) -> Fopdt:
    """Least-squares fit of K, T, L to the whole response after the step."""
    start = start or two_point(sd)

    def resid(x):
        K, T, L = x
        return Fopdt(K, T, L).step(sd.t, sd.du) - sd.dy

    lo = [-np.inf, 1e-3, 0.0]
    hi = [np.inf, np.inf, max(sd.t[-1] * 0.5, 1.0)]
    x0 = [start.K, max(start.T, 1e-3), min(max(start.L, 0.0), hi[2] * 0.99)]
    res = least_squares(resid, x0, bounds=(lo, hi), x_scale="jac")
    K, T, L = res.x
    rmse = float(np.sqrt(np.mean(res.fun ** 2)))
    return Fopdt(float(K), float(T), float(L), rmse, method="least-squares")


@dataclass
class LeadLag:
    """G(s) = K (1 + Ta s) e^{-Ls} / ((1 + T1 s)(1 + T2 s)),  T1 > T2.

    Captures the two time scales of a TCU zone: the fast water loop (T2) and
    the slow machine part (T1); the lead Ta gives the quick partial jump.
    """
    K: float
    T1: float
    T2: float
    Ta: float
    L: float
    rmse: float = float("nan")

    def step(self, t: np.ndarray, du: float = 1.0) -> np.ndarray:
        tt = np.clip(t - self.L, 0.0, None)
        T1, T2, Ta = self.T1, self.T2, self.Ta
        if abs(T1 - T2) < 1e-6:
            T2 = T1 * (1 - 1e-6)
        y = 1.0 - (T1 - Ta) / (T1 - T2) * np.exp(-tt / T1) - (T2 - Ta) / (T2 - T1) * np.exp(-tt / T2)
        return self.K * du * y

    def frf(self, w: np.ndarray) -> np.ndarray:
        s = 1j * w
        return abs(self.K) * (1 + self.Ta * s) * np.exp(-self.L * s) / ((1 + self.T1 * s) * (1 + self.T2 * s))


def fit_lead_lag(sd: StepData, start: Fopdt | None = None) -> LeadLag:
    """Least-squares fit of the 5-parameter lead-lag model; best of several starts."""
    f = start or fit_fopdt(sd)
    t_end = float(sd.t[-1])

    def resid(x):
        K, T1, T2, Ta, L = x
        return LeadLag(K, T1, T2, Ta, L).step(sd.t, sd.du) - sd.dy

    lo = [-np.inf, 1.0, 0.1, 0.0, 0.0]
    hi = [np.inf, 50.0 * t_end, t_end, 50.0 * t_end, 0.25 * t_end]
    best = None
    for T2 in (5.0, 20.0, 60.0, 150.0):
        for L in (1.0, 10.0, 30.0):
            for ta_frac in (0.0, 0.5, 2.0):
                x0 = [f.K, max(f.T, 2 * T2 + 1), T2, ta_frac * T2, L]
                x0 = [min(max(v, l_ + 1e-9), h_ - 1e-9) for v, l_, h_ in zip(x0, lo, hi)]
                res = least_squares(resid, x0, bounds=(lo, hi), x_scale="jac", max_nfev=400)
                if best is None or res.cost < best.cost:
                    best = res
    K, T1, T2, Ta, L = best.x
    if T2 > T1:
        T1, T2 = T2, T1
    rmse = float(np.sqrt(np.mean(best.fun ** 2)))
    return LeadLag(float(K), float(T1), float(T2), float(Ta), float(L), rmse)


@dataclass
class RelayResult:
    Ku: float  # %/K
    Pu: float  # s
    a: float  # K, PV oscillation amplitude
    d: float  # %, relay amplitude
    cycles: int


def relay_analysis(t: np.ndarray, u: np.ndarray, y: np.ndarray, hyst: float = 0.0,
                   settle_cycles: int = 2) -> RelayResult:
    """Describing-function estimate from a relay experiment.

    Uses the last full cycles after `settle_cycles` switch-overs; Ku = 4d / (pi a),
    corrected for hysteresis eps as 4d / (pi sqrt(a^2 - eps^2)).
    """
    sw = np.nonzero(np.diff(np.sign(u - np.mean(u))) != 0)[0] + 1
    rising = [k for k in sw if u[k] > u[k - 1]]
    if len(rising) < settle_cycles + 2:
        raise ValueError("not enough relay cycles in the record")
    use = rising[settle_cycles:]
    periods = np.diff(t[use])
    Pu = float(np.mean(periods))
    seg = slice(use[0], use[-1])
    a = 0.5 * float(np.max(y[seg]) - np.min(y[seg]))
    d = 0.5 * float(np.max(u[seg]) - np.min(u[seg]))
    a_eff = np.sqrt(max(a * a - hyst * hyst, 1e-12))
    Ku = 4.0 * d / (np.pi * a_eff)
    return RelayResult(float(Ku), Pu, a, d, len(use) - 1)
