"""tcu-tune: command-line tuning tool for TCU zones.

    tcu-tune demo  DIR [--zones ABCD] [--lin 2] simulated test records to try the tool
    tcu-tune nameplate [--zone A] [--dtref 65]  FB_CoolLinearise settings from the TCU hardware
    tcu-tune fit   --heat H.csv [--cool C.csv] [--relay R.csv] [--ms 1.4] [--json F]
    tcu-tune batch DIR [--ms 1.4] [--out summary.csv]
                   DIR/<zone>/heat.csv (+ optional cool.csv, relay.csv) per zone
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import asdict
from pathlib import Path

from . import experiments as ex
from .experiments import StepRecord
from .linearise import spec_for
from .physics import PRESET_DESIGNS, derive
from .plant import PRESETS, cool_conductance, with_noise
from .tool import ZoneTuning, format_report, tune_zone


def _opt(path: str | None) -> StepRecord | None:
    return StepRecord.from_csv(path) if path else None


def _zone_json(zt: ZoneTuning) -> dict:
    return {
        "zone": zt.zone,
        "fopdt": asdict(zt.fopdt),
        "model": asdict(zt.model),
        "cool_model": asdict(zt.cool_model) if zt.cool_model else None,
        "relay": asdict(zt.relay) if zt.relay else None,
        "cool_gain_ratio": zt.ratio,
        "recommended": zt.recommended,
        "candidates": [{"name": c.name, "params": asdict(c.prm) if c.prm else None,
                        "ms_heat": c.ms_heat, "ms_cool": c.ms_cool, "note": c.note}
                       for c in zt.candidates],
    }


def cmd_demo(a: argparse.Namespace) -> int:
    out = Path(a.dir)
    for z in a.zones:
        p = with_noise(PRESETS[z])
        d = out / z
        d.mkdir(parents=True, exist_ok=True)
        ex.heat_step_test(p, duration=a.duration).to_csv(d / "heat.csv")
        lin = spec_for(PRESETS[z], a.lin) if a.lin else None
        ex.cool_step_test(p, duration=a.duration, lin=lin).to_csv(d / "cool.csv")
        ex.relay_test(p, hyst=0.2).to_csv(d / "relay.csv")
        print(f"zone {z}: {d / 'heat.csv'}, cool.csv, relay.csv")
    return 0


def cmd_fit(a: argparse.Namespace) -> int:
    zt = tune_zone(StepRecord.from_csv(a.heat), _opt(a.cool), _opt(a.relay),
                   ms_target=a.ms, relay_hyst=a.hyst, zone=a.zone)
    print(format_report(zt))
    if a.json:
        Path(a.json).write_text(json.dumps(_zone_json(zt), indent=2))
    return 0


def cmd_nameplate(a: argparse.Namespace) -> int:
    """Print the FB_CoolLinearise parameters derived from the TCU hardware (UC2)."""
    for z in a.zone:
        d = PRESET_DESIGNS[z]
        q = derive(d)
        p = PRESETS[z]
        print(f"Zone {z}: {d.part}")
        for k in ("Cw", "Cm", "UAwm", "UAloss", "L", "Cp", "Cs_max"):
            note = q.notes.get(k, "")
            print(f"  {k:7s} = {q.params[k]:8.3f}   {note}")
        print(f"  FB_CoolLinearise: Mode := 2; DTref := {a.dtref:.1f}; UA := {p.hx_UA:.3f}; "
              f"CsMax := {p.Cs_max:.3f}; Cp := {p.Cp:.3f}; ValveR := {p.valve_R:.1f}; FlowExp := 0.8")
        print(f"  G(100 %) = {cool_conductance(p, 100.0):.3f} kW/K -> 100 % demand = "
              f"{cool_conductance(p, 100.0) * a.dtref:.1f} kW of cooling")
    return 0


def cmd_batch(a: argparse.Namespace) -> int:
    root = Path(a.dir)
    zones = sorted(d for d in root.iterdir() if (d / "heat.csv").exists())
    if not zones:
        print(f"no <zone>/heat.csv found under {root}", file=sys.stderr)
        return 2
    rows = []
    for d in zones:
        cool = d / "cool.csv"
        relay = d / "relay.csv"
        zt = tune_zone(StepRecord.from_csv(d / "heat.csv"),
                       StepRecord.from_csv(cool) if cool.exists() else None,
                       StepRecord.from_csv(relay) if relay.exists() else None,
                       ms_target=a.ms, relay_hyst=a.hyst, zone=d.name)
        print(format_report(zt), end="\n\n")
        b = zt.best()
        rows.append({"zone": d.name, "rule": b.name, "Gain": f"{b.prm.Gain:.3f}",
                     "Ti": f"{b.prm.Ti:.1f}", "Td": f"{b.prm.Td:.1f}",
                     "CoolGainRatio": f"{b.prm.CoolGainRatio:.3f}",
                     "Ms_heat": f"{b.ms_heat:.2f}", "Ms_cool": f"{b.ms_cool:.2f}"})
    if a.out:
        with open(a.out, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"wrote {a.out}")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="tcu-tune", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("demo", help="write simulated test records")
    d.add_argument("dir")
    d.add_argument("--zones", default="ABCD")
    d.add_argument("--duration", type=float, default=5400.0)
    d.add_argument("--lin", type=int, default=0, choices=(0, 1, 2),
                   help="record the cooling test through FB_CoolLinearise in this mode")
    d.set_defaults(fn=cmd_demo)

    n = sub.add_parser("nameplate", help="FB_CoolLinearise settings from the TCU hardware")
    n.add_argument("--zone", default="ABCD")
    n.add_argument("--dtref", type=float, default=65.0, help="Tw - Tcw at the cooling test, K")
    n.set_defaults(fn=cmd_nameplate)

    f = sub.add_parser("fit", help="tune one zone from its test records")
    f.add_argument("--heat", required=True)
    f.add_argument("--cool")
    f.add_argument("--relay")
    f.add_argument("--zone", default="")
    f.add_argument("--ms", type=float, default=1.4)
    f.add_argument("--hyst", type=float, default=0.2, help="relay hysteresis, K")
    f.add_argument("--json")
    f.set_defaults(fn=cmd_fit)

    b = sub.add_parser("batch", help="tune every zone folder under DIR")
    b.add_argument("dir")
    b.add_argument("--ms", type=float, default=1.4)
    b.add_argument("--hyst", type=float, default=0.2)
    b.add_argument("--out")
    b.set_defaults(fn=cmd_batch)

    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    raise SystemExit(main())
