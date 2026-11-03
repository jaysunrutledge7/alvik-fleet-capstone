#!/usr/bin/env python3
"""The Pi's own address for the screens (risk review 2 Nov, #3: a fresh Pi printed the HOME Pi's Tailscale address,
because an empty site address fell back to a hard-coded one).

Detected on the Pi itself, in this order:
  1. its Tailscale address (`tailscale ip -4`), when Tailscale is up: works from anywhere the laptop is on Tailscale;
  2. else its address on the WiFi / LAN (the first IPv4 of `hostname -I` that is not Tailscale's 100.64-100.127 range);
  3. else its name with .local (the laptop can often find it by name on the same network).

A site may still pin an address (the old way, kept behind a switch): "pi": {"address": "100.122.222.75",
"use_site_address": true}. Off by default, so a site file copied to a new Pi can never point the screens at another Pi.

    python3 tools/fleet/pi_addr.py            # one word: the address to type in the laptop's browser
    python3 tools/fleet/pi_addr.py --all      # every address found, in words
"""
import ipaddress
import os
import socket
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def _run(cmd, timeout=2.0):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout if r.returncode == 0 else ""
    except (OSError, subprocess.SubprocessError):
        return ""


def _is_tailscale(ip):
    try:
        return ipaddress.ip_address(ip) in ipaddress.ip_network("100.64.0.0/10")
    except ValueError:
        return False


def tailscale_ip(run=_run):
    for w in run(["tailscale", "ip", "-4"]).split():
        if _is_tailscale(w):
            return w
    return None


def lan_ips(run=_run):
    out = []
    for w in run(["hostname", "-I"]).split():
        if w.count(".") == 3 and not _is_tailscale(w) and not w.startswith("127.") and not w.startswith("169.254."):
            out.append(w)
    return out


def site_pin():
    """The site's pinned address when it asks for it (use_site_address), else None. Never raises."""
    try:
        sys.path.insert(0, HERE)
        import sites as SI
        pi = SI.load().get("pi") or {}
        if pi.get("use_site_address") and pi.get("address"):
            return str(pi["address"])
    except BaseException:     # noqa: BLE001  (the launcher reports a broken site file itself)
        pass
    return None


def detect(run=_run, pin=None):
    """(address, how) for the screens."""
    pin = site_pin() if pin is None else pin
    if pin:
        return pin, "the site's pinned address (pi.use_site_address)"
    ts = tailscale_ip(run)
    if ts:
        return ts, "Tailscale"
    lan = lan_ips(run)
    if lan:
        return lan[0], "the WiFi / LAN (Tailscale is not up on this Pi)"
    return socket.gethostname() + ".local", "the Pi's name (no network address found)"


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--all" in argv:
        ts = tailscale_ip()
        print("Tailscale: %s" % (ts or "not up"))
        print("WiFi / LAN: %s" % (", ".join(lan_ips()) or "none"))
        print("Name: %s.local" % socket.gethostname())
        a, how = detect()
        print("Shown on the screens: %s (%s)" % (a, how))
        return 0
    print(detect()[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
