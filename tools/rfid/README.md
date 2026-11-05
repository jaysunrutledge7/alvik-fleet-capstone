# tools/rfid: switched off (Nick, 4 Nov 2026)

Jay-Sun asked that we not build models or reports on his RFID data or align his clocks from it. The code here is
kept (nothing deleted) and is **available to Jay-Sun to use on his own end if he wants it**, but every tool refuses to
run unless it is explicitly enabled:

    python3 tools/rfid/school_rfid.py --jay-requested <session> <capture.csv>
    MIRLT_RFID_JAY_REQUESTED=1 python3 ...           # the same, for a whole shell

It is in no runbook, how-to or automatic step. Our side only hands over our own raw files (camera track, raw odometry,
logs: `./run-fleet-pi.sh jay-bundle`), never anything from his reader.

| File | What it does (when enabled) |
| --- | --- |
| `school_rfid.py` | loads a reader capture, lines up the clocks, labels rows with our poses |
| `rfid_model.py` | windows, a nearest-neighbour model, held-out zones and points |
| `rfid_report.py` | the one-page report and charts |
| `simulate_jay_csv.py` | fake captures from a simulator run (invented tags) |
| `rfid_gate.py` | the switch |
