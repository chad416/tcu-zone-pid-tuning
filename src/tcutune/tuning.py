"""PI/PID tuning rules and a robustness check.

All rules return parameters in the form FB_ZonePID uses:
    u = Gain * (b*Sp - Pv) + Gain/Ti * integral(e) - Gain*Td * dPv/dt (filtered)
Gain in %/K, Ti and Td in seconds.

Rules (references in README):
    zn_step     Ziegler-Nichols open-loop (reaction curve) PID
    simc_pi     Skogestad SIMC PI, tau_c = L (or a chosen tau_c)
    lambda_pi   lambda / IMC PI, lambda = max(3L, T)
    amigo_pi    Astrom-Hagglund AMIGO PI
    tl_pi       Tyreus-Luyben PI from a relay test (Ku, Pu)
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from .identify import Fopdt, LeadLag, RelayResult
from .pid import PidParams


class NotApplicable(ValueError):
    """The rule's assumptions do not hold for this model (e.g. no dead time)."""


#: FOPDT rules need a visible dead time; below this L/T they give absurd gains.
MIN_L_OVER_T = 0.01


def _check_fopdt(m: Fopdt) -> None:
    if m.L / m.T < MIN_L_OVER_T:
        raise NotApplicable(f"L/T = {m.L / m.T:.4f} < {MIN_L_OVER_T}: the step fit shows "
                            "no usable dead time")


def zn_step(m: Fopdt) -> PidParams:
    _check_fopdt(m)
    return PidParams(Gain=1.2 * m.T / (abs(m.K) * m.L), Ti=2.0 * m.L, Td=0.5 * m.L)


def simc_pi(m: Fopdt, tau_c: float | None = None) -> PidParams:
    _check_fopdt(m)
    return _simc(m, m.L if tau_c is None else tau_c)


def _simc(m: Fopdt, tau_c: float) -> PidParams:
    Kc = m.T / (abs(m.K) * (tau_c + m.L))
    Ti = min(m.T, 4.0 * (tau_c + m.L))
    return PidParams(Gain=Kc, Ti=Ti)


def lambda_pi(m: Fopdt, lam: float | None = None) -> PidParams:
    lam = max(3.0 * m.L, m.T) if lam is None else lam
    return PidParams(Gain=m.T / (abs(m.K) * (lam + m.L)), Ti=m.T)


def amigo_pi(m: Fopdt) -> PidParams:
    _check_fopdt(m)
    K, T, L = abs(m.K), m.T, m.L
    Kc = 0.15 / K + (0.35 - L * T / (L + T) ** 2) * T / (K * L)
    Ti = 0.35 * L + 13.0 * L * T * T / (T * T + 12.0 * L * T + 7.0 * L * L)
    return PidParams(Gain=Kc, Ti=Ti)


def tl_pi(r: RelayResult) -> PidParams:
    return PidParams(Gain=r.Ku / 3.2, Ti=2.2 * r.Pu)


RULES = {"zn_step": zn_step, "simc_pi": simc_pi, "lambda_pi": lambda_pi, "amigo_pi": amigo_pi}


def effective_fopdt(m: LeadLag) -> Fopdt:
    """Reduce the lead-lag model to a FOPDT with Skogestad's approximation rules.

    The lead (Ta s + 1) and the fast lag (T2 s + 1) are combined (SIMC 2003):
        Ta >= T2 >= L : (Ta s+1)/(T2 s+1) ~ Ta/T2      (cancel, gain up)
        Ta >= L >= T2 : (Ta s+1)/(T2 s+1) ~ Ta/L       (fast lag inside the delay)
        L  >= Ta      : lead ignored, half rule on T2
    """
    K, T1, T2, Ta, L = abs(m.K), m.T1, m.T2, m.Ta, max(m.L, 1e-3)
    if Ta >= T2 >= L:
        return Fopdt(K * Ta / T2, T1, L, method="lead-lag -> FOPDT (Ta/T2)")
    if Ta >= L >= T2:
        return Fopdt(K * Ta / L, T1, L + T2 / 2.0, method="lead-lag -> FOPDT (Ta/L)")
    return Fopdt(K, T1 + T2 / 2.0, L + T2 / 2.0, method="lead-lag -> FOPDT (half rule)")


def simc_ms(m: LeadLag, w: np.ndarray, ms_max: float = 1.4) -> tuple[PidParams, float]:
    """SIMC PI on the reduced model; tau_c raised from L until Ms <= ms_max on
    the full lead-lag model.  Returns (parameters, tau_c)."""
    f = effective_fopdt(m)
    G = m.frf(w)
    tau_c = max(f.L, 1.0)
    for _ in range(200):
        prm = _simc(f, tau_c)
        if max_sensitivity(prm, G, w) <= ms_max:
            return prm, tau_c
        tau_c *= 1.1
    raise NotApplicable(f"no SIMC tau_c reaches Ms <= {ms_max}")


def crossover_ratio(prm: PidParams, G_heat: np.ndarray, G_cool: np.ndarray,
                    w: np.ndarray) -> float:
    """CoolGainRatio = |G_cool| / |G_heat| at the heating loop's gain crossover.

    The static ratio Kcool/Kheat is not the right number when the open cooling
    valve also shortens the slow time constant: around the crossover frequency
    the cooling path is then relatively stronger than its static gain says.
    """
    L = np.abs(controller_frf(prm, w) * G_heat)
    k = int(np.argmax(L < 1.0)) if np.any(L < 1.0) else len(w) - 1
    return float(np.abs(G_cool[k]) / np.abs(G_heat[k]))


def with_cooling(prm: PidParams, k_heat: float, k_cool: float) -> PidParams:
    """Set CoolGainRatio = Kcool / Kheat (magnitudes of the two process gains)."""
    return replace(prm, CoolGainRatio=abs(k_cool) / abs(k_heat))


# --------------------------------------------------------------------------
# robustness: maximum sensitivity on a FOPDT model
# --------------------------------------------------------------------------

def controller_frf(prm: PidParams, w: np.ndarray) -> np.ndarray:
    s = 1j * w
    C = prm.Gain * (1.0 + (1.0 / (prm.Ti * s) if prm.Ti > 0 else 0.0))
    if prm.Td > 0:
        C = C + prm.Gain * prm.Td * s / (1.0 + prm.Td / prm.N * s)
    return C


def fopdt_frf(m: Fopdt, w: np.ndarray, gain_scale: float = 1.0) -> np.ndarray:
    s = 1j * w
    return gain_scale * abs(m.K) * np.exp(-m.L * s) / (1.0 + m.T * s)


def max_sensitivity(prm: PidParams, G: np.ndarray, w: np.ndarray) -> float:
    """Ms = max |1 / (1 + C G)|.  Ms <= 1.4 robust, 1.4-2 normal, > 2 aggressive."""
    L = controller_frf(prm, w) * G
    return float(np.max(np.abs(1.0 / (1.0 + L))))


def default_grid(m: Fopdt) -> np.ndarray:
    L = max(m.L, 0.1)
    return np.logspace(np.log10(1e-3 / max(m.T, 1.0)), np.log10(20.0 / L), 4000)
