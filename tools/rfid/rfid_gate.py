"""Off unless asked (Nick, 4 Nov 2026): Jay-Sun asked that we not build models or reports on his RFID data or align his
clocks from it. The code stays (never deleted) and is his to use on his own end; it refuses to run unless explicitly
enabled: the flag --jay-requested on the command line, or MIRLT_RFID_JAY_REQUESTED=1 in the environment."""
import os

ENV = "MIRLT_RFID_JAY_REQUESTED"


class RfidOff(SystemExit):
    pass


def enabled(env=None):
    env = os.environ if env is None else env
    return str(env.get(ENV, "")).strip().lower() in ("1", "yes", "true", "on")


def enable_from_argv(argv):
    """--jay-requested in argv (removed from it): switches the tools on for this process. Returns the argv left."""
    argv = list(argv)
    if "--jay-requested" in argv:
        argv.remove("--jay-requested")
        os.environ[ENV] = "1"
    return argv


def require(what="the RFID tools"):
    if not enabled():
        raise RfidOff("%s: switched OFF (Nick, 4 Nov 2026: Jay-Sun asked that we not build models or reports on his "
                      "RFID data or align his clocks from it). Kept for Jay-Sun to use on his own end: run with "
                      "--jay-requested, or set %s=1, only if he asks for it." % (what, ENV))
