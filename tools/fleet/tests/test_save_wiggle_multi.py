"""Build e03 (5 Nov 2026): calibration saved on the robot (firmware `save` / `unsave`, `corr save` on older firmware),
wiggle-first naming for every run, every robot found calibrated in turn (the other parked and kept out of the way),
waking any robot from any address (robot_sleep.py wake any / --name). Simulator only: no robot, no Pi."""
import json
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
TOOLS = os.path.dirname(FLEET)
REPO = os.path.dirname(TOOLS)
for p in (FLEET, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "pi"), os.path.join(TOOLS, "recognition"),
          os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)


def _harness(tmp_path, specs, rs_args, timeout=200):
    cmd = ["unshare", "-rn", sys.executable, os.path.join(HERE, "sim_wake_harness.py"), str(tmp_path)] + specs + \
          ["--"] + rs_args
    out = subprocess.run(cmd, capture_output=True, text=True, cwd=REPO, timeout=timeout)
    return out.stdout + out.stderr


# ------------------------------------------------------------------------------------------------- waking
def test_wake_any_wakes_the_robot_from_whatever_address_it_checks_in_from(tmp_path):
    # the broadcast goes nowhere (127.0.0.9): the robot is only heard at its check-in, and woken at that address
    out = _harness(tmp_path, ["AL-1DC0:0.3:0.3"], ["wake", "any", "--broadcast", "127.0.0.9", "--max-min", "1.5"])
    assert "AL-1DC0: awake at 127.0.0.2, battery" in out and "RC=0" in out, out


def test_wake_by_name_leaves_the_other_robot_alone(tmp_path):
    out = _harness(tmp_path, ["AL-1DC0:0.2:0.1", "AL-2B7F:0.3:0.5"],
                   ["wake", "--name", "AL-2B7F", "--broadcast", "127.0.0.9", "--max-min", "1.5"])
    assert "AL-2B7F: awake at 127.0.0.3" in out and "AL-1DC0: awake" not in out and "RC=0" in out, out


def test_wake_by_address_still_works_and_names_robots_heard_elsewhere(tmp_path):
    out = _harness(tmp_path, ["AL-1DC0:0.2:0.1"], ["wake", "127.0.0.50", "--max-min", "0.3"])
    assert "heard AL-1DC0 at 127.0.0.2" in out and "wake --name AL-1DC0" in out and "RC=1" in out, out
    out = _harness(tmp_path / "b", ["AL-1DC0:0.2:0.1"], ["wake", "127.0.0.2", "--max-min", "1"])
    assert "127.0.0.2: awake" in out and "RC=0" in out, out
