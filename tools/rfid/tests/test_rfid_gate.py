"""The RFID tools refuse to run unless explicitly enabled (Nick, 4 Nov 2026)."""
import os
import subprocess
import sys

import pytest

RFID = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RFID)

import rfid_gate as G     # noqa: E402


@pytest.mark.gate_off
def test_off_by_default_and_on_by_flag_or_env(monkeypatch):
    monkeypatch.delenv(G.ENV, raising=False)
    assert not G.enabled()
    with pytest.raises(SystemExit) as e:
        G.require("x")
    assert "--jay-requested" in str(e.value) and "OFF" in str(e.value)
    assert G.enable_from_argv(["a", "--jay-requested"]) == ["a"] and G.enabled()


@pytest.mark.gate_off
def test_each_tool_refuses_without_the_flag(tmp_path, monkeypatch):
    env = dict(os.environ)
    env.pop(G.ENV, None)
    for script, args in (("school_rfid.py", ["latest", "x.csv"]), ("simulate_jay_csv.py", ["s", "--out", "o.csv"])):
        r = subprocess.run([sys.executable, os.path.join(RFID, script)] + args, capture_output=True, text=True, env=env,
                           cwd=str(tmp_path), timeout=60)
        assert r.returncode != 0 and "switched OFF" in (r.stderr + r.stdout)
    monkeypatch.delenv(G.ENV, raising=False)
    import rfid_model as RM
    with pytest.raises(SystemExit):
        RM.evaluate([], {})
