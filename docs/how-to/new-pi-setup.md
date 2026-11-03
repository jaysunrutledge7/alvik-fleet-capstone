# Set up a new Raspberry Pi from a blank SD card (about 1 hour, most of it waiting)

For the school Pi (or any second Pi). Do it at home, the night before. Every step says which window to type in,
gives one command per box, and says what you should see. Type each box exactly, then press **Enter**.

You need: the new Pi 4, its official power supply (5.1 V, 3 A), a blank micro SD card (32 GB or more), the laptop
with an SD card slot or a USB card reader, and your home WiFi name and password.

## A. Write the card (laptop, Raspberry Pi Imager)

1. On the laptop, open **Chrome**, click the address bar, type `raspberrypi.com/software` and press **Enter**. Click
   **Download for Windows**. Open the downloaded file and click through the installer (Next, Install, Finish).
2. Put the SD card in the laptop.
3. Open **Raspberry Pi Imager** (Start menu, type `Imager`, press **Enter**).
4. Click **CHOOSE DEVICE**, click **Raspberry Pi 4**.
5. Click **CHOOSE OS**, click **Raspberry Pi OS (64-bit)** (the first one, with the desktop: it opens USB drives by
   itself, which the recorder uses; the "Lite" one does not).
6. Click **CHOOSE STORAGE**, click the SD card (check its size: never pick the laptop's own drive).
7. Click **NEXT**. It asks *Would you like to apply OS customisation settings?* Click **EDIT SETTINGS**.
8. On the **GENERAL** tab, tick and fill in:
   - **Set hostname:** `mirlt-school`
   - **Set username and password:** username `nick`, and a password you will remember (you type it on the Pi often).
   - **Configure wireless LAN:** SSID = your **home** WiFi's name exactly, its password, **Wireless LAN country: US**.
     (At school you add the school's network with one command: step 4 of "At school" below.)
   - **Set locale settings:** Time zone **America/New_York**, Keyboard layout **us**.
9. Click the **SERVICES** tab. Tick **Enable SSH**, choose **Use password authentication**.
10. Click **SAVE**, then **YES** (use the settings), then **YES** (erase the card). Wait for **Write Successful**
    (5-10 minutes). Click **CONTINUE** and take the card out.

## B. First start (the Pi)

1. Put the card in the Pi (underneath, contacts up). Plug in the power supply. The red light comes on; the green
   one flickers.
2. **Wait 3 minutes.** The first start resizes the card and restarts once by itself.

## C. Connect from the laptop (PowerShell)

1. On the laptop: Start menu, type `PowerShell`, press **Enter**. A blue window opens.
2. Type:
   ```
   ssh nick@mirlt-school.local
   ```
   The first time it asks *Are you sure you want to continue connecting (yes/no/[fingerprint])?* Type `yes` and press
   **Enter**. Then type the Pi's password (nothing shows while you type) and press **Enter**.
   You should see a line ending in `nick@mirlt-school:~ $`. Everything below in a box is typed here (the Pi).
   - *Could not resolve hostname:* wait 2 more minutes and try again. Still not: the WiFi name or password in step A8
     was wrong; write the card again.

## D. Update the system (the Pi, in the blue window)

```
sudo apt update
```
It asks for the password once. The last line is `Reading state information... Done` (maybe with "N packages can
be upgraded").
```
sudo apt full-upgrade -y
```
5-20 minutes of text. It ends back at `nick@mirlt-school:~ $`.
```
sudo reboot
```
The window says the connection closed. Wait 1 minute, then type the `ssh nick@mirlt-school.local` line again (step C2).

## E. Install what the tools use (the Pi)

These are every package the school site's programs import (checked 3 Nov by scanning their imports and by importing
them all in an empty Python with only these four: numpy, scipy, OpenCV, matplotlib):
```
sudo apt install -y git tmux v4l-utils python3-pip python3-numpy python3-scipy python3-opencv python3-matplotlib python3-pandas
```
It ends back at the prompt. (pandas is only for the heat tracker's chart.)

## F. Get the code (the Pi)

```
git clone https://github.com/nfmaxfield-cmd/alvik-fleet-capstone.git
```
You should see `Receiving objects: 100%` and `Resolving deltas: 100%`.
```
cd alvik-fleet-capstone
```
The prompt now ends in `alvik-fleet-capstone $`. Every later step starts here: after a new `ssh` login, type this
`cd` line first.

*If the repository is ever made private:* the clone asks for a username and password. The username is
`nfmaxfield-cmd`; the password is a GitHub token (made as in step H, with **Contents: Read-only** enough), not your
GitHub password. Nothing else changes; `git pull` asks the same way unless the token is saved.

## G. Check OpenCV (the Pi)

```
python3 -c "import cv2; print(cv2.__version__, hasattr(cv2, 'aruco'))"
```
You should see a version and `True`, for example `4.6.0 True` (Raspberry Pi OS Bookworm) or `4.10.0 True` (a newer
image). The tag detector was tested on 4.6 and 5.0; 4.10 uses the same newer calls as 5.0. If you see `False`, type:
```
pip3 install --break-system-packages pupil-apriltags
```
(the tag detector then uses pupil-apriltags instead; the self-check below says which one it uses).

## H. The GitHub token for uploads (laptop, then the Pi)

Uploads of a session's logs need a token on the Pi. Make a **new** one for this Pi (keep home's): follow
`docs/how-to/mark-the-robot.md`, part 6, steps 1-4, but name it `mirlt-school-pi`. Then on the Pi:
```
python3 tools/pi/upload_session.py --set-token
```
It says `Paste the GitHub token (it will not show) and press Enter:`. **Right-click** once inside the blue window
(that pastes; nothing shows), then press **Enter**. You should see `Saved. This computer can upload to
nfmaxfield-cmd/alvik-fleet-capstone now`. Never paste the token anywhere else (not in a chat, not in a file).
The Pi stays in a shared room: after the school sessions, delete it with `rm ~/.config/mirlt/github_token`.

## I. Tailscale (the Pi, then the laptop's browser)

So the laptop can reach the Pi from anywhere, not only on the same WiFi.
```
curl -fsSL https://tailscale.com/install.sh | sh
```
It ends with `Installation complete!` (and maybe a hint to run tailscale up).
```
sudo tailscale up
```
It prints `To authenticate, visit:` and a link. Select the link with the mouse in the blue window, right-click to
copy, paste it in **Chrome**, log in with the **same account as the laptop's Tailscale**, click **Connect**. The blue
window then says `Success.`
```
tailscale ip -4
```
It prints this Pi's Tailscale address (100.something). Mission Control prints it for you from now on.

## J. The site and the cameras (the Pi)

```
./run-fleet-pi.sh site school-conference
```
You should see `Site now: school-conference.` and the site's lines (robots Alpha and Bravo, AprilTags, 1280 x 960).

Plug both cameras into the Pi's blue USB ports.
```
python3 tools/pi/camnames.py
```
It lists each camera: its `/dev/video` number, its name, its USB port, and (new) its stable name under
`/dev/v4l/by-id`. The numbers can swap after a restart; the launchers find each camera by its name and port every
time, so that never matters. Tell the Pi which is which (with the name and port it printed; the camera over corner
1's half is `overhead`):
```
python3 tools/pi/camnames.py --set overhead "USB 2.0 Camera" 1.4
```
You should see `Saved: the overhead camera is the one named ...`. Then the same with `side` and the other port.

## K. The self-check (the Pi)

```
./run-fleet-pi.sh selfcheck
```
One line for each thing the tools need: the system, the programs, every package, OpenCV and the tag detector (it
reads drawn robot and reference tags), the repository and GitHub, the site, the token (never shown), Tailscale, the
clock, each camera (does it offer 1280 x 960?), the card and the USB drive (hours of video that fit), power and heat,
and the address for the laptop. Each line starts **PASS**, **WARN** or **FAIL**, and a FAIL line says how to fix it.
The last line should be `... 0 FAIL. Ready.` Fix every FAIL line and run it again.

## L. Mission Control (the Pi, then the laptop's browser)

```
./run-fleet-pi.sh mc
```
You should see `Mission Control is open: http://100.x.y.z:8095 in the laptop's browser`. Type that address in
Chrome on the laptop. It keeps running when the blue window closes or the laptop sleeps (it runs in tmux session
`mc`). To see its window: `tmux attach -t mc` (leave it running: **Ctrl+B**, then **D**). To close it:
`./run-fleet-pi.sh mc-stop`.

## At school

1. Plug in the Pi and the cameras. On the laptop, open PowerShell and type `ssh nick@mirlt-school.local`. If the
   laptop is not on the same WiFi, use the Tailscale address instead: `ssh nick@100.x.y.z`.
2. `cd alvik-fleet-capstone`, then `git pull` (gets today's fixes).
3. `./run-fleet-pi.sh address` shows the Pi's addresses.
4. To join the school's WiFi (the name and password from Jay):
   ```
   sudo nmcli device wifi connect "<SSID>" password "<password>"
   ```
5. `./run-fleet-pi.sh selfcheck` again: everything PASS or WARN.
6. **Emergency stop** from the Pi: `./run-fleet-pi.sh stopall`. It stops every run, presses Mission Control's Stop,
   and sends `stop` to the site's robots, to the whole WiFi (a broadcast) and to every robot heard on it. The real
   last resort is still to pick the robot up and switch it off.
