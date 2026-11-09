"""From recorded zone tests to recommended FB_ZonePID settings.

Input  : a heater step test (required), a cooling-valve step test and a relay
         test (both optional), as CSV with t_s, heat_pct, cool_pct, pv_degC.
Output : identified models, every applicable tuning rule with its robustness
         (Ms) in the heating and the cooling range, and one recommendation.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from . import identify as idf
from . import tuning as tu
from .experiments import StepRecord
from .pid import PidParams

W = np.logspace(-5, 0.5, 3000)  # rad/s, covers T1 ~ hours down to L ~ seconds


@dataclass
class Candidate:
    name: str
    prm: PidParams | None
    ms_heat: float = float("nan")
    ms_cool: float = float("nan")
    note: str = ""


@dataclass
class ZoneTuning:
    zone: str
    fopdt: idf.Fopdt
    model: idf.LeadLag
    cool_model: idf.LeadLag | None
    relay: idf.RelayResult | None
    ratio: float  # Kcool / Kheat, 1.0 when no cooling test
    candidates: list[Candidate] = field(default_factory=list)
    recommended: str = ""

    def best(self) -> Candidate:
        return next(c for c in self.candidates if c.name == self.recommended)


def tune_zone(heat: StepRecord, cool: StepRecord | None = None,
              relay: StepRecord | None = None, ms_target: float = 1.4,
              relay_hyst: float = 0.2, zone: str = "") -> ZoneTuning:
    sd = idf.find_step(heat.t, heat.heat, heat.pv)
    fopdt = idf.fit_fopdt(sd)
    model = idf.fit_lead_lag(sd, fopdt)

    cool_model, ratio = None, 1.0
    if cool is not None:
        sdc = idf.find_step(cool.t, cool.cool, cool.pv)
        cool_model = idf.fit_lead_lag(sdc)
        ratio = abs(cool_model.K) / abs(model.K)

    rr = None
    if relay is not None:
        rr = idf.relay_analysis(relay.t, relay.heat, relay.pv, hyst=relay_hyst)

    G_heat = model.frf(W)
    G_cool = cool_model.frf(W) if cool_model is not None else None

    def make(name: str, prm: PidParams | None, note: str = "") -> Candidate:
        if prm is None:
            return Candidate(name, None, note=note)
        ms_h = tu.max_sensitivity(prm, G_heat, W)
        if G_cool is None:
            return Candidate(name, prm, ms_h, float("nan"), note)
        r = tu.crossover_ratio(prm, G_heat, G_cool, W)
        prm = replace(prm, CoolGainRatio=r)
        ms_c = tu.max_sensitivity(prm, G_cool / r, W)
        return Candidate(name, prm, ms_h, ms_c, note)

    out = ZoneTuning(zone, fopdt, model, cool_model, rr, ratio)
    for name, rule in tu.RULES.items():
        try:
            out.candidates.append(make(name, rule(fopdt), "FOPDT fit"))
        except tu.NotApplicable as e:
            out.candidates.append(make(name, None, f"not applicable: {e}"))
    if rr is not None:
        out.candidates.append(make("tl_pi", tu.tl_pi(rr), "relay test"))
    try:
        prm, tau_c = tu.simc_ms(model, W, ms_target)
        out.candidates.append(make(f"simc_ms{ms_target:g}", prm,
                                   f"lead-lag model, tau_c = {tau_c:.1f} s"))
        out.recommended = f"simc_ms{ms_target:g}"
    except tu.NotApplicable as e:
        out.candidates.append(make(f"simc_ms{ms_target:g}", None, f"not applicable: {e}"))
        ok = [c for c in out.candidates if c.prm is not None and c.ms_heat <= ms_target]
        if ok:
            out.recommended = max(ok, key=lambda c: c.prm.Gain / c.prm.Ti).name
    return out


def format_report(zt: ZoneTuning) -> str:
    f, m = zt.fopdt, zt.model
    lines = [f"Zone {zt.zone}".strip(),
             f"  FOPDT     K={f.K:.3f} K/%  T={f.T:.0f} s  L={f.L:.1f} s  (rmse {f.rmse:.3f} K)",
             f"  lead-lag  K={m.K:.3f} K/%  T1={m.T1:.0f} s  T2={m.T2:.1f} s  Ta={m.Ta:.1f} s  "
             f"L={m.L:.1f} s  (rmse {m.rmse:.3f} K)"]
    if zt.cool_model is not None:
        c = zt.cool_model
        lines.append(f"  cooling   K={c.K:.3f} K/%  T1={c.T1:.0f} s  T2={c.T2:.1f} s  L={c.L:.1f} s  "
                     f"(static Kcool/Kheat = {zt.ratio:.2f}; the rules below use the ratio at their crossover)")
    if zt.relay is not None:
        r = zt.relay
        lines.append(f"  relay     Ku={r.Ku:.2f} %/K  Pu={r.Pu:.0f} s  ({r.cycles} cycles)")
    lines.append(f"  {'rule':12s} {'Gain %/K':>9s} {'Ti s':>8s} {'Td s':>6s} {'CoolRatio':>9s} {'Ms heat':>8s} {'Ms cool':>8s}  note")
    for c in zt.candidates:
        mark = "*" if c.name == zt.recommended else " "
        if c.prm is None:
            lines.append(f" {mark}{c.name:12s} {'-':>9s} {'-':>8s} {'-':>6s} {'-':>9s} {'-':>8s} {'-':>8s}  {c.note}")
        else:
            p = c.prm
            lines.append(f" {mark}{c.name:12s} {p.Gain:9.2f} {p.Ti:8.1f} {p.Td:6.1f} "
                         f"{p.CoolGainRatio:9.2f} {c.ms_heat:8.2f} {c.ms_cool:8.2f}  {c.note}")
    lines.append("  * = recommended. Ms <= 1.4 robust, 1.4-2 usual, > 2 aggressive.")
    return "\n".join(lines)
