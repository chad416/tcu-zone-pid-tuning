"""Full study for the dossier.

1. zone parameters from the hardware (physics.py)
2. identification + tuning from simulated technician tests, with and without
   the cooling linearisation
3. production scenario at the test point, every cooling alternative
4. robustness over the operating envelope (setpoint, load, cooling water,
   exchanger fouling) on the true linearised plant
5. the same alternatives in the time domain at the envelope corners
6. batch load on the mixer zone: sensitivity peak and power feedforward
7. PLC execution budget

Writes results/study.md, results/*.csv and docs/img/*.png.
Run:  python scripts/run_study.py   (about 15 min on a laptop)
      python scripts/run_study.py --quick   (CI smoke run, coarse grids)
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
import time
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from tcutune import envelope as en  # noqa: E402
from tcutune import experiments as ex  # noqa: E402
from tcutune import identify as idf  # noqa: E402
from tcutune import metrics as me  # noqa: E402
from tcutune import physics as ph  # noqa: E402
from tcutune import plant  # noqa: E402
from tcutune import tuning as tu  # noqa: E402
from tcutune.linearise import spec_for  # noqa: E402
from tcutune.tool import W, tune_zone  # noqa: E402

RES = ROOT / "results"
IMG = ROOT / "docs" / "img"

C1, C2, C3, C4, C5 = "#2a78d6", "#eb6834", "#1baf7a", "#8b5cf6", "#d4a017"
INK, INK2, MUTED, GRID, AXIS, SURF = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#fcfcfb"
ALT_COLORS = {"A": C2, "B": C5, "C": MUTED, "D": C4, "E": C1}

plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "lines.linewidth": 1.8, "font.size": 10, "axes.titlesize": 11,
    "axes.titlecolor": INK, "legend.frameon": False, "legend.labelcolor": INK2,
})

ZONES = "ABCD"
CORNERS = {  # name: (Tcw, Scenario factory)
    "test point": (15.0, lambda q: ex.Scenario(70.0, 80.0, 75.0, q)),
    "winter, hot": (8.0, lambda q: ex.Scenario(75.0, 90.0, 85.0, 1.25 * q)),
    "summer, cool": (28.0, lambda q: ex.Scenario(45.0, 55.0, 50.0, 1.25 * q)),
}
BATCH_PERIOD = 150.0  # s, mixing cycle (assumed, 120-240 s range studied)
BATCH_MIX_FRACTION = 0.65  # share of the cycle with the rotors loaded (assumed)


def md_table(header: list[str], rows: list[list]) -> list[str]:
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return out


# --------------------------------------------------------------------------
# 1-2: parameters, identification, tuning
# --------------------------------------------------------------------------

def stage_tuning(quick: bool):
    dur = 3600 if quick else 5400
    out = {}
    for z in ZONES:
        p0 = plant.PRESETS[z]
        pn = plant.with_noise(p0)
        heat = ex.heat_step_test(pn, duration=dur)
        relay = ex.relay_test(pn, hyst=0.2)
        cool0 = ex.cool_step_test(pn, duration=dur)
        cool2 = ex.cool_step_test(pn, duration=dur, lin=spec_for(p0, 2))
        zt0 = tune_zone(heat, cool0, relay, zone=z)
        zt2 = tune_zone(heat, cool2, relay, zone=z)
        out[z] = {"zt0": zt0, "zt2": zt2, "heat": heat}
        print(f"zone {z}: {zt0.best().prm}  E ratio {zt2.best().prm.CoolGainRatio:.2f}", flush=True)
    return out


def params_for(tun, z: str, alt: en.Alternative):
    """FB_ZonePID settings and linearisation the tool gives for an alternative."""
    zt0, zt2 = tun[z]["zt0"], tun[z]["zt2"]
    base = zt0.best().prm
    p0 = plant.PRESETS[z]
    if alt.key == "A":
        return replace(base, CoolGainRatio=1.0), None
    if alt.key == "B":
        return replace(base, CoolGainRatio=zt0.ratio), None
    if alt.key == "C":
        return base, None
    if alt.key == "D":
        # temperature scaling: the identified crossover ratio of C still applies at the test point
        return base, spec_for(p0, 1)
    return zt2.best().prm, spec_for(p0, 2)


def true_ms_at_test(z: str, prm, lin) -> tuple[float, float]:
    p = plant.PRESETS[z]
    op = en.op_point(p, 80.0, p.q_run)
    Gc = en.frf_cool(p, lin, op, W)
    Gh = en.frf_heat(p, W)
    return (tu.max_sensitivity(prm, Gh, W), tu.max_sensitivity(prm, Gc / prm.CoolGainRatio, W))


# --------------------------------------------------------------------------
# 4: envelope
# --------------------------------------------------------------------------

def stage_envelope(tun, quick: bool):
    w = np.logspace(-5, 0.5, 600 if quick else 1500)
    rows, raw = [], {}
    for z in ZONES:
        p = plant.PRESETS[z]
        pts = en.grid(p, Tsp=(50, 70, 90), qf=(0.5, 1.0, 1.5), Tcw=(8, 15, 30)) if quick else en.grid(p)
        for cond, pt in (("clean", p), ("fouled", en.fouled(p))):
            for alt in en.ALTERNATIVES:
                prm, _ = params_for(tun, z, alt)
                r = en.evaluate(pt, p, prm, alt, w, pts)
                raw[(z, cond, alt.key)] = r
                ms = r.ms
                rows.append([z, cond, alt.key, f"{r.ratio:.2f}", len(ms), f"{ms.min():.2f}",
                             f"{np.median(ms):.2f}", f"{ms.max():.2f}",
                             f"{100 * np.mean(ms > 2.0):.0f}", f"{100 * np.mean(ms < 1.2):.0f}"])
        print(f"envelope zone {z} done", flush=True)
    return rows, raw


def stage_gain_spread(quick: bool):
    """Deciding parameter: cooling-path gain at the heating crossover, relative
    to the test point, over the envelope (no linearisation)."""
    w = np.logspace(-5, 0.5, 600 if quick else 1500)
    rows, pts_all = [], {}
    for z in ZONES:
        p = plant.PRESETS[z]
        prm = tu.PidParams(Gain=1.0, Ti=0.0)  # only the crossover frequency is needed
        Gh = en.frf_heat(p, w)
        op0 = en.op_point(p, 80.0, p.q_run)
        g0 = en.frf_cool(p, None, op0, w)
        # heating crossover with the tool's PI is near 1/(L + tau_s + T2); use |Gh| = 1/Kc point
        k = int(np.argmin(np.abs(np.abs(Gh) * 20.0 - 1.0)))
        del prm
        vals = []
        for Tsp, q, Tcw in en.grid(p):
            pt = replace(p, Tcw=Tcw)
            op = en.op_point(pt, Tsp, q)
            if not op.feasible:
                continue
            g = en.frf_cool(pt, None, op, w)
            vals.append((op.Tw - Tcw, op.valve, abs(g[k]) / abs(g0[k])))
        a = np.array(vals)
        pts_all[z] = a
        rows.append([z, f"{a[:, 2].min():.2f}", f"{a[:, 2].max():.2f}", f"{a[:, 2].max() / a[:, 2].min():.1f}",
                     f"{a[:, 0].min():.0f}-{a[:, 0].max():.0f}", f"{a[:, 1].min():.0f}-{a[:, 1].max():.0f}"])
    return rows, pts_all


# --------------------------------------------------------------------------
# 3, 5: time domain
# --------------------------------------------------------------------------

def stage_time_domain(tun, quick: bool):
    rows, runs = [], {}
    scale = 0.35 if quick else 1.0
    for z in ZONES:
        p0 = plant.with_noise(plant.PRESETS[z])
        for cname, (Tcw, mk) in CORNERS.items():
            sc = mk(p0.q_run)
            if quick:
                sc = sc.scaled(scale)
            p = replace(p0, Tcw=Tcw)
            for alt in en.ALTERNATIVES:
                if cname != "test point" and alt.key in ("B", "D"):
                    continue
                prm, lin = params_for(tun, z, alt)
                run = ex.closed_loop(p, prm, sc, lin=lin)
                seg = me.segments(run)
                tot = me.totals(seg)
                ms_h, ms_c = true_ms_at_test(z, prm, lin) if cname == "test point" else (np.nan, np.nan)
                rows.append([z, cname, alt.key, f"{prm.Gain:.1f}", f"{prm.Ti:.0f}",
                             f"{prm.CoolGainRatio:.2f}",
                             "" if np.isnan(ms_h) else f"{ms_h:.2f}", "" if np.isnan(ms_c) else f"{ms_c:.2f}",
                             f"{seg[2].iae:.0f}", f"{seg[2].max_dev:.2f}", f"{seg[3].max_dev:.2f}",
                             f"{seg[4].max_dev:.2f}", tot["switchovers"], tot["unsettled_segments"],
                             f"{run.cool.max():.0f}", f"{tot['tv'] / (run.t[-1] / 3600.0):.0f}"])
                runs[(z, cname, alt.key)] = run
        print(f"time domain zone {z} done", flush=True)
    return rows, runs


# --------------------------------------------------------------------------
# 6: batch load on the mixer zone
# --------------------------------------------------------------------------

def batch_profile(q_mean: float, amp: float, period: float, frac: float, t_on: float, t_off: float):
    hi = q_mean + amp * (1.0 - frac) * 2.0
    lo = q_mean - amp * frac * 2.0

    def q(t: float) -> float:
        if not (t_on <= t < t_off):
            return 0.0
        return hi if ((t - t_on) % period) < frac * period else max(lo, 0.0)
    return q


def stage_batch(tun, quick: bool):
    z = "D"
    p = plant.PRESETS[z]
    prm, lin = params_for(tun, z, en.ALTERNATIVES[4])
    op = en.op_point(p, 80.0, p.q_run)
    w = np.logspace(-4, -0.5, 400)
    Gc = en.frf_cool(p, lin, op, w)
    S = en.sensitivity_mag(prm, Gc / prm.CoolGainRatio, w)
    Gd = en.frf_load(p, lin, op, w)
    periods = 2 * np.pi / w
    # feedforward gain: heating-equivalent % per kW, from the linearised cooling scale
    gref = lin.G(100.0)
    k_ff = 100.0 * prm.CoolGainRatio / (gref * lin.DTref)  # %/kW
    ff_tau = p.Cm / p.UAwm  # s, the steel buffers the load before the water sees it
    sc = ex.Scenario(70.0, 80.0, 80.0, p.q_run, 3000, 5400, 20000, 20000, 9000 if quick else 12000)
    qf = batch_profile(p.q_run, 0.6 * p.q_run, BATCH_PERIOD, BATCH_MIX_FRACTION, 5400, 20000)
    cases = [("no feedforward", 0.0, 1.0), ("feedforward, exact", k_ff, 1.0),
             ("feedforward, +30 % error", k_ff, 1.3), ("feedforward, -30 % error", k_ff, 0.7)]
    rows, runs = [], {}
    pn = plant.with_noise(p)
    for name, g, err in cases:
        prm_ff = replace(prm, FfTime=ff_tau if g else 0.0)
        run = ex.closed_loop(pn, prm_ff, sc, lin=lin, q_fn=qf, ff_gain=g, ff_meas=err)
        m = run.t >= sc.t_end - 1800  # last 30 min, periodic steady state
        e = run.pv[m] - run.sp[m]
        rows.append([name, f"{k_ff * err if g else 0:.2f}", f"{np.ptp(run.pv[m]):.2f}",
                     f"{np.sqrt(np.mean(e ** 2)):.3f}", f"{np.max(np.abs(e)):.2f}"])
        runs[name] = run
    # open-loop ripple (valve held at its mean) for reference
    k_bp = int(np.argmin(np.abs(periods - BATCH_PERIOD)))
    # the same feedforward on the head zone C (sensor in the steel): a process-load step
    zc = "C"
    pc = plant.PRESETS[zc]
    prm_c, lin_c = params_for(tun, zc, en.ALTERNATIVES[4])
    k_ff_c = 100.0 * prm_c.CoolGainRatio / (lin_c.G(100.0) * lin_c.DTref)
    sc_c = ex.Scenario(q_run=pc.q_run)
    if quick:
        sc_c = sc_c.scaled(0.35)
    c_rows = []
    for name, g, err in [("no feedforward", 0.0, 1.0), ("feedforward, exact", k_ff_c, 1.0),
                         ("feedforward, +30 % error", k_ff_c, 1.3), ("feedforward, -30 % error", k_ff_c, 0.7)]:
        run = ex.closed_loop(plant.with_noise(pc), prm_c, sc_c, lin=lin_c, ff_gain=g, ff_meas=err)
        seg = me.segments(run)
        c_rows.append([name, f"{k_ff_c * err if g else 0:.2f}", f"{seg[2].max_dev:.2f}", f"{seg[2].iae:.0f}",
                       f"{seg[4].max_dev:.2f}", f"{seg[4].iae:.0f}"])
    info = {"S_at_batch": float(S[k_bp]), "S_max": float(S.max()), "c_rows": c_rows,
            "S_band": (float(periods[S > 1.0].min()), float(periods[S > 1.0].max())),
            "Gd_at_batch": float(abs(Gd[k_bp])), "k_ff": k_ff, "ff_tau": ff_tau,
            "periods": periods, "S": S, "Gd": np.abs(Gd)}
    return rows, runs, info


# --------------------------------------------------------------------------
# 7: PLC budget (static operation count x datasheet times)
# --------------------------------------------------------------------------

T_FLOAT_NS = 256.0  # CPU 1513-1 PN, typical floating-point instruction [P1]
T_WORD_NS = 48.0
T_EXP_EQUIV = 20  # EXP / LN counted as 20 float operations each (assumed)


def count_ops(path: Path) -> dict[str, int]:
    """Upper-bound count: every arithmetic/comparison operator in the body,
    FOR bodies multiplied by their trip count, all branches counted."""
    text = path.read_text(encoding="utf-8")
    body = text[text.index("BEGIN"):]
    body = re.sub(r"//[^\n]*", "", body)
    total_f, total_w = 0, 0
    loops = list(re.finditer(r"FOR\s+#\w+\s*:=\s*(\d+)\s+TO\s+(\d+)\s+DO(.*?)END_FOR;", body, re.S))
    outside = body
    for m in loops:
        n = int(m.group(2)) - int(m.group(1)) + 1
        f, wd = _ops(m.group(3))
        total_f += n * f
        total_w += n * wd
        outside = outside.replace(m.group(0), "")
    f, wd = _ops(outside)
    return {"float": total_f + f, "word": total_w + wd}


def _ops(s: str) -> tuple[int, int]:
    arith = len(re.findall(r"(?<![:<>=])[-+*/](?!=)", s))
    cmp_ = len(re.findall(r"<=|>=|<>|(?<![:<>])=|(?<!-)<|>", s))
    fns = len(re.findall(r"\b(LIMIT|MIN|MAX|ABS)\s*\(", s))
    expln = len(re.findall(r"\b(EXP|LN)\s*\(", s))
    assigns = len(re.findall(r":=", s))
    return arith + cmp_ + 2 * fns + T_EXP_EQUIV * expln, assigns


def stage_budget() -> list[list]:
    rows = []
    zone_total = 0.0
    for name in ("FB_ZonePID", "FB_CoolLinearise"):
        c = count_ops(ROOT / "plc" / f"{name}.scl")
        us = (c["float"] * T_FLOAT_NS + c["word"] * T_WORD_NS) / 1000.0
        zone_total += us
        rows.append([name, c["float"], c["word"], f"{us:.1f}"])
    rows.append(["per zone (PID + linearisation)", "", "", f"{zone_total:.1f}"])
    rows.append(["8 zones, x3 margin", "", "", f"{8 * 3 * zone_total:.0f}"])
    return rows


# --------------------------------------------------------------------------
# figures
# --------------------------------------------------------------------------

def fig_envelope(raw) -> None:
    fig, axes = plt.subplots(1, 4, figsize=(12, 4.0), sharey=True)
    for ax, z in zip(axes, ZONES):
        for i, alt in enumerate(en.ALTERNATIVES):
            for j, cond in enumerate(("clean", "fouled")):
                ms = raw[(z, cond, alt.key)].ms
                x = i + (j - 0.5) * 0.32
                jit = np.random.default_rng(i * 10 + j).uniform(-0.09, 0.09, len(ms))
                ax.scatter(x + jit, np.clip(ms, 0, 4.0), s=5, color=ALT_COLORS[alt.key],
                           alpha=0.45 if cond == "clean" else 0.25, linewidths=0)
                ax.plot([x - 0.13, x + 0.13], [np.median(ms)] * 2, color=INK, lw=1.2)
        ax.axhline(2.0, color=AXIS, lw=0.8, ls=":")
        ax.axhspan(1.3, 1.6, color=GRID, alpha=0.5, lw=0)
        ax.set_xticks(range(5), [a.key for a in en.ALTERNATIVES])
        ax.set_title(f"zone {z}: {ph.PRESET_DESIGNS[z].part.split(',')[0]}", fontsize=9)
        ax.set_ylim(0.9, 4.05)
    axes[0].set_ylabel("Ms in cooling (true plant), clipped at 4")
    fig.suptitle("Robustness over the operating envelope (50-90 degC, 0.5-1.5 x load, cooling water 8-30 degC); "
                 "left dots clean, right dots fouled exchanger", fontsize=10, color=INK)
    fig.text(0.5, 0.005, "A one gain set   B static ratio   C crossover ratio (v0.1)   "
             "D + Tw-Tcw scaling   E + exchanger/valve inversion (v0.2)", ha="center", fontsize=9, color=INK2)
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    fig.savefig(IMG / "envelope_ms.png", dpi=150)
    plt.close(fig)


def fig_gain_spread(pts) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), sharey=True)
    for ax, z in zip(axes, ("A", "B")):
        a = pts[z]
        sc = ax.scatter(a[:, 0], a[:, 2], c=a[:, 1], cmap="viridis", s=14)
        ax.axhline(1.0, color=AXIS, lw=0.8)
        ax.set_xlabel("Tw - Tcw at the operating point (K)")
        valve = "PWM solenoid (linear)" if plant.PRESETS[z].valve_R == 0 else "equal-% valve, R = 30"
        ax.set_title(f"zone {z}, {valve}", fontsize=10)
        ax.set_yscale("log")
        fig.colorbar(sc, ax=ax, label="valve opening (%)")
    axes[0].set_ylabel("cooling loop gain / gain at test point")
    fig.suptitle("Why one tuning cannot hold: cooling-path gain near crossover over the envelope",
                 fontsize=11, color=INK)
    fig.tight_layout()
    fig.savefig(IMG / "cooling_gain_spread.png", dpi=150)
    plt.close(fig)


def fig_corners(runs, z: str = "B") -> None:
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.2), sharex="col",
                             gridspec_kw={"height_ratios": [3, 2]})
    for col, cname in enumerate(("winter, hot", "summer, cool")):
        ax, ax2 = axes[0, col], axes[1, col]
        base = runs[(z, cname, "A")]
        t = base.t / 60
        ax.plot(t, base.sp, color=INK2, ls="--", lw=1.0, label="setpoint")
        for k in ("A", "C", "E"):
            r = runs[(z, cname, k)]
            ax.plot(t, r.pv, color=ALT_COLORS[k], lw=1.3, label={"A": "A one gain set",
                    "C": "C crossover ratio", "E": "E + inversion"}[k])
            ax2.plot(t, r.cool, color=ALT_COLORS[k], lw=1.0)
        sc = base.scenario
        m = (base.t >= sc.t_load_on - 300) & (base.t <= sc.t_load_off + 600)
        lo = min(min(runs[(z, cname, k)].pv[m].min() for k in ("A", "C", "E")), base.sp[m].min())
        hi = max(max(runs[(z, cname, k)].pv[m].max() for k in ("A", "C", "E")), base.sp[m].max())
        ax.set_ylim(lo - 0.5, hi + 0.5)
        ax.set_xlim((sc.t_load_on - 300) / 60, (sc.t_load_off + 600) / 60)
        for name, te in sc.events()[2:]:
            ax.axvline(te / 60, color=AXIS, lw=0.8)
            ax.text(te / 60 + 0.3, hi + 0.3, name, color=MUTED, fontsize=8)
        ax.set_title(f"zone {z}, {cname} (cooling water {CORNERS[cname][0]:.0f} degC)", fontsize=10)
        ax2.set_xlabel("time (min)")
    axes[0, 0].set_ylabel("temperature (degC)")
    axes[1, 0].set_ylabel("cooling valve (%)")
    axes[0, 0].legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(IMG / f"corners_zone{z}.png", dpi=150)
    plt.close(fig)


def fig_batch(runs, info) -> None:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 3.9))
    ax1.semilogx(info["periods"], info["S"], color=C1, label="|S|: closed loop / open loop")
    ax1.axhline(1.0, color=AXIS, lw=0.8)
    ax1.axvspan(120, 240, color=GRID, alpha=0.7, lw=0, label="mixing cycle 2-4 min")
    ax1.set_xlabel("disturbance period (s)")
    ax1.set_ylabel("|S(jw)|")
    ax1.set_title("Zone D: feedback amplifies loads at batch periods", fontsize=10)
    ax1.legend(fontsize=8, loc="lower left")
    for name, col in (("no feedforward", C2), ("feedforward, exact", C1), ("feedforward, +30 % error", MUTED)):
        r = runs[name]
        m = r.t >= r.t[-1] - 900
        ax2.plot((r.t[m] - r.t[m][0]) / 60, r.pv[m], color=col, lw=1.2, label=name)
    ax2.axhline(80.0, color=INK2, ls="--", lw=1.0)
    ax2.set_xlabel("time (min), last 15 min of the batch run")
    ax2.set_ylabel("temperature (degC)")
    ax2.set_title(f"Batch load {BATCH_PERIOD:.0f} s cycle, linearised cooling", fontsize=10)
    ax2.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(IMG / "batch_feedforward.png", dpi=150)
    plt.close(fig)


def fig_identification(zt, heat) -> None:
    sd = idf.find_step(heat.t, heat.heat, heat.pv)
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot(sd.t / 60, sd.dy, color=MUTED, lw=1.0, label="recorded PV change (noise, 0.1 K steps)")
    ax.plot(sd.t / 60, zt.fopdt.step(sd.t, sd.du), color=C2,
            label=f"FOPDT fit: L = {zt.fopdt.L:.1f} s, RMSE {zt.fopdt.rmse:.3f} K")
    ax.plot(sd.t / 60, zt.model.step(sd.t, sd.du), color=C1,
            label=f"lead-lag fit: L = {zt.model.L:.1f} s, RMSE {zt.model.rmse:.3f} K")
    ins = ax.inset_axes([0.55, 0.12, 0.42, 0.42])
    m = sd.t <= 300
    ins.plot(sd.t[m], sd.dy[m], color=MUTED, lw=1.0)
    ins.plot(sd.t[m], zt.fopdt.step(sd.t[m], sd.du), color=C2)
    ins.plot(sd.t[m], zt.model.step(sd.t[m], sd.du), color=C1)
    ins.set_title("first 300 s (x in seconds)", fontsize=9, color=INK2)
    ins.tick_params(labelsize=8)
    ax.set_xlabel("time after heater step +10 % (min)")
    ax.set_ylabel("PV change (K)")
    ax.set_title("Zone A heater step: FOPDT misses the fast water mode, lead-lag fits both")
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    fig.savefig(IMG / "identification_zoneA.png", dpi=150)
    plt.close(fig)


def fig_exchanger() -> None:
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    u = np.linspace(0, 100, 201)
    for z, col in zip(ZONES, (C1, C2, C3, C4)):
        p = plant.PRESETS[z]
        g = np.array([plant.cool_conductance(p, x) for x in u])
        ax.plot(u, g / g[-1], color=col, label=f"zone {z} ({'linear' if p.valve_R == 0 else 'eq-%'} valve)")
    ax.plot(u, u / 100, color=AXIS, ls=":", lw=1.0, label="ideal linear actuator")
    ax.set_xlabel("cooling valve opening (%)")
    ax.set_ylabel("G / G(100 %)")
    ax.set_title("Cooling conductance of valve + plate exchanger (effectiveness-NTU)", fontsize=10)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(IMG / "exchanger_characteristic.png", dpi=150)
    plt.close(fig)


# --------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    RES.mkdir(exist_ok=True)
    IMG.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    md = ["# Study results (generated by scripts/run_study.py" + (" --quick" if a.quick else "") + ")", "",
          "Everything is simulation. Ms values are on the true linearised plant, which only a",
          "simulation can know. Zone hardware is in src/tcutune/physics.py.", ""]

    # 1
    rows = []
    for z in ZONES:
        d = ph.PRESET_DESIGNS[z]
        p = plant.PRESETS[z]
        op = en.op_point(p, 80.0, p.q_run)
        rows.append([z, d.part, f"{p.Cw:.0f}", f"{p.Cm:.0f}", f"{p.UAwm:.2f}", f"{p.UAloss:.3f}",
                     f"{p.Pheat_max:.0f}", f"{plant.cool_conductance(p, 100):.2f}",
                     "linear" if p.valve_R == 0 else f"eq-% R={p.valve_R:.0f}", f"{p.L:.1f}",
                     f"{p.tau_s:.0f}", p.sensor, f"{p.q_run:.0f}", f"{op.valve:.0f}"])
    md += ["## 1. Zone parameters derived from the hardware", ""]
    md += md_table(["Zone", "part", "Cw kJ/K", "Cm kJ/K", "UAwm kW/K", "UAloss kW/K", "heater kW",
                    "G(100 %) kW/K", "valve", "L s", "tau_s s", "sensor", "q_run kW",
                    "valve at test point %"], rows)
    md.append("")
    fig_exchanger()

    # 2
    tun = stage_tuning(a.quick)
    rows = []
    for z in ZONES:
        zt0, zt2 = tun[z]["zt0"], tun[z]["zt2"]
        f, m, b = zt0.fopdt, zt0.model, zt0.best()
        rows.append([z, f"{f.K:.2f}, {f.T:.0f} s, {f.L:.1f} s", f"{m.K:.2f}, {m.T1:.0f}, {m.T2:.0f}, {m.Ta:.0f}, "
                     f"{m.L:.1f} s", f"{f.rmse:.3f} / {m.rmse:.3f}", f"{b.prm.Gain:.1f}", f"{b.prm.Ti:.0f}",
                     f"{zt0.ratio:.2f}", f"{b.prm.CoolGainRatio:.2f}", f"{zt2.best().prm.CoolGainRatio:.2f}"])
    md += ["## 2. Identification and the tool's recommendation (SIMC on the lead-lag model, Ms 1.4)", ""]
    md += md_table(["Zone", "FOPDT K, T, L", "lead-lag K, T1, T2, Ta, L", "RMSE FOPDT / lead-lag K",
                    "Gain %/K", "Ti s", "static Kc/Kh", "crossover ratio (C)",
                    "crossover ratio with inversion (E)"], rows)
    md.append("")
    fig_identification(tun["A"]["zt0"], tun["A"]["heat"])

    # 3 + 5
    td_rows, runs = stage_time_domain(tun, a.quick)
    with open(RES / "time_domain.csv", "w", newline="") as fh:
        wtr = csv.writer(fh)
        hdr = ["zone", "case", "alt", "Gain", "Ti", "ratio", "Ms_heat_test", "Ms_cool_test", "IAE_load_on",
               "peak_load_on", "overshoot_SP_down", "peak_load_off", "switchovers", "unsettled", "max_valve",
               "travel_pct_per_h"]
        wtr.writerow(hdr)
        wtr.writerows(td_rows)
    md += ["## 3. Production scenario at the test point and at the envelope corners (nonlinear plant)", "",
           "Scenario: cold start, setpoint up, process load on (cooling), setpoint down, load off.",
           "Corners: winter, hot = cooling water 8 degC, setpoints 75/90/85, 1.25 x load;",
           "summer, cool = cooling water 28 degC, setpoints 45/55/50, 1.25 x load.", ""]
    md += md_table(["Zone", "case", "alt", "Gain", "Ti", "ratio", "Ms heat", "Ms cool", "IAE load-on K s",
                    "peak load-on K", "overshoot SP-down K", "peak load-off K", "switchovers", "unsettled",
                    "max valve %", "actuator travel %/h"], td_rows)
    md.append("")
    fig_corners(runs, "B")
    fig_corners(runs, "A")

    # 4
    g_rows, g_pts = stage_gain_spread(a.quick)
    md += ["## 4. Deciding parameter: cooling-path gain near crossover over the envelope (no linearisation)", ""]
    md += md_table(["Zone", "min gain / test", "max gain / test", "spread", "Tw - Tcw range K", "valve range %"],
                   g_rows)
    md.append("")
    fig_gain_spread(g_pts)
    e_rows, raw = stage_envelope(tun, a.quick)
    with open(RES / "envelope.csv", "w", newline="") as fh:
        wtr = csv.writer(fh)
        wtr.writerow(["zone", "exchanger", "alt", "ratio", "points", "Ms_min", "Ms_median", "Ms_max",
                      "pct_Ms_gt_2", "pct_Ms_lt_1.2"])
        wtr.writerows(e_rows)
    md += ["## 5. Robustness over the envelope (Ms in cooling, true linearised plant)", "",
           "Grid: setpoint 50-90 degC, load 0.5-1.5 x q_run, cooling water 8-30 degC; points where the",
           "valve or the linearised demand saturates are left out (capacity, not tuning).",
           f"Fouled: exchanger UA / (1 + U Rf), U = {en.U_CLEAN_PLATE:.0f} W/m2K, Rf = {en.RF_COOLING_TOWER} m2K/W "
           "(the tuning and the linearisation keep the clean nameplate values).", ""]
    md += md_table(["Zone", "exchanger", "alt", "ratio", "points", "Ms min", "Ms median", "Ms max",
                    "% points Ms > 2", "% points Ms < 1.2"], e_rows)
    md.append("")
    fig_envelope(raw)

    # 6
    b_rows, b_runs, info = stage_batch(tun, a.quick)
    md += ["## 6. Batch load on the mixer zone (D)", "",
           f"Load: {BATCH_PERIOD:.0f} s cycle, rotors loaded {100 * BATCH_MIX_FRACTION:.0f} % of the cycle, "
           f"mean = q_run, linearised cooling (E).",
           f"|S| at the batch period: {info['S_at_batch']:.2f}; |S| > 1 for periods "
           f"{info['S_band'][0]:.0f}-{info['S_band'][1]:.0f} s; peak |S| = {info['S_max']:.2f}.",
           f"Feedforward gain {info['k_ff']:.2f} %/kW (from Gref, DTref and the ratio), lag {info['ff_tau']:.0f} s "
           "(Cm / UAwm).", ""]
    md += md_table(["case", "FF gain %/kW", "PV peak-to-peak K", "RMS error K", "max |error| K"], b_rows)
    md += ["", "Head zone C (sensor in the steel), process-load step at the test point, alternative E, "
           "feedforward without lag:", ""]
    md += md_table(["case", "FF gain %/kW", "peak load-on K", "IAE load-on K s", "peak load-off K",
                    "IAE load-off K s"], info["c_rows"])
    md.append("")
    fig_batch(b_runs, info)

    # 7
    md += ["## 7. PLC execution budget (static count x CPU 1513-1 PN datasheet times)", "",
           f"Float op {T_FLOAT_NS:.0f} ns, word op {T_WORD_NS:.0f} ns, EXP/LN counted as {T_EXP_EQUIV} float ops "
           "(assumed); all branches and all 16 bisection steps counted (upper bound).", ""]
    md += md_table(["block", "float ops", "word ops (assignments)", "time us"], stage_budget())
    md.append("")
    md.append(f"Run time {time.time() - t0:.0f} s.")
    (RES / ("study_quick.md" if a.quick else "study.md")).write_text("\n".join(md) + "\n", encoding="utf-8")
    print("\n".join(md))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
