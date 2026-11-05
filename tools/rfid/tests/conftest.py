"""The RFID tools are switched off unless Jay-Sun asks (rfid_gate.py, Nick 4 Nov 2026): their own tests switch them on
for the test run only (the gate itself is tested in test_rfid_gate.py)."""
import os
import sys

import pytest

RFID = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if RFID not in sys.path:
    sys.path.insert(0, RFID)


@pytest.fixture(autouse=True)
def _rfid_on(monkeypatch, request):
    if "gate_off" not in request.keywords:
        monkeypatch.setenv("MIRLT_RFID_JAY_REQUESTED", "1")
    yield


def pytest_configure(config):
    config.addinivalue_line("markers", "gate_off: the RFID tools left switched off (the gate's own tests)")
