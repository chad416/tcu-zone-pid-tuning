"""SPS-202: the tuning tool end to end on simulated records."""

import csv
import json

from tcutune.cli import main


def test_demo_fit_batch(tmp_path, capsys):
    assert main(["demo", str(tmp_path / "d"), "--zones", "AB", "--duration", "3600"]) == 0
    assert (tmp_path / "d" / "A" / "heat.csv").exists()

    out_json = tmp_path / "a.json"
    assert main(["fit", "--heat", str(tmp_path / "d/A/heat.csv"), "--cool", str(tmp_path / "d/A/cool.csv"),
                 "--relay", str(tmp_path / "d/A/relay.csv"), "--zone", "A", "--json", str(out_json)]) == 0
    rep = json.loads(out_json.read_text())
    assert rep["recommended"] == "simc_ms1.4"
    best = next(c for c in rep["candidates"] if c["name"] == rep["recommended"])
    assert best["ms_heat"] <= 1.4 + 1e-9
    assert 1.5 < best["params"]["CoolGainRatio"] < 8.0
    text = capsys.readouterr().out
    assert "not applicable" in text  # FOPDT rules refused on a water-sensor zone

    summary = tmp_path / "s.csv"
    assert main(["batch", str(tmp_path / "d"), "--out", str(summary)]) == 0
    rows = list(csv.DictReader(summary.open()))
    assert [r["zone"] for r in rows] == ["A", "B"]
    assert all(float(r["Gain"]) > 0 and float(r["Ti"]) > 0 for r in rows)


def test_nameplate_and_linearised_demo(tmp_path, capsys):
    assert main(["nameplate", "--zone", "B"]) == 0
    assert "Mode := 2" in capsys.readouterr().out
    assert main(["demo", str(tmp_path / "l"), "--zones", "B", "--duration", "2400", "--lin", "2"]) == 0
    assert main(["fit", "--heat", str(tmp_path / "l/B/heat.csv"), "--cool", str(tmp_path / "l/B/cool.csv"),
                 "--zone", "B"]) == 0
