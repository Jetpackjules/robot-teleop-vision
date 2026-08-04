# Headless Linux and remote debugging

Robot Teleop Vision does not need a desktop session for normal operation. The launcher starts Godot with `--headless`; camera capture, RGB-D encoding, the operator server, the selected robot module, and the optional Cloudflare Quick Tunnel all run from a terminal. The browser is the operational display.

The bundled native RealSense extension currently supports Linux x86-64. An ARM64 host needs a separately built extension before following this procedure.

## First installation

Connect over SSH and run:

```bash
git clone https://github.com/Jetpackjules/robot-teleop-vision.git
cd robot-teleop-vision
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[realsense,calibration,test]"
robot-teleop init --example vision_only
robot-teleop doctor
robot-teleop devices --json
robot-teleop modules --json
```

Keep the generated `config/local.toml` on that installation. It is ignored by Git and may contain site identities and credentials. Do not copy another site's robot profile or calibration merely to make Doctor pass.

Begin in vision-only mode. After the live point cloud and RGB view pass inspection, follow the selected robot module's reviewed site procedure before enabling physical motion.

## Start and inspect

`robot-teleop start` remains in the foreground so one supervisor owns every child process and can Hold the selected robot before shutdown. Run it inside `tmux` if it must survive an SSH interruption:

```bash
tmux new -s robot-teleop
source .venv/bin/activate
robot-teleop start
```

Detach with `Ctrl+B`, then `D`. Reattach with `tmux attach -t robot-teleop`. From another SSH terminal:

```bash
source .venv/bin/activate
robot-teleop status
robot-teleop stop
```

In Quick Tunnel mode, `status` publishes the current temporary URL only after its login/operator page passes the public health check. The URL changes whenever the tunnel restarts.

## Journal diagnostics

On a systemd-based Linux host, the supervisor automatically mirrors its own messages and every supervised child's combined output into the native journal. Console output remains unchanged, and journal failure can never stop capture, control, or safe shutdown.

Follow the stack live:

```bash
journalctl -f SYSLOG_IDENTIFIER=robot-teleop
```

Inspect a recent failure:

```bash
journalctl --since=-30m --no-pager SYSLOG_IDENTIFIER=robot-teleop
journalctl -k --since=-30m --no-pager
```

The second command is useful for USB resets and camera disconnects. Journal retention is controlled by the host's normal `journald` policy.

## Share a sanitized support bundle

After reproducing a problem, run:

```bash
robot-teleop support-bundle
```

The command writes a timestamped `robot-teleop-support-*.zip` in the current directory. Use `--output /path/name.zip` to choose another path, or `--journal-lines 2000` to change the capped journal sample.

The bundle contains system/repository versions, sanitized launcher state and configuration shape, Doctor checks, pseudonymized camera information, and recent Robot Teleop Vision journal entries. It excludes raw configuration, environment variables, credentials, robot profiles, calibration payloads, captures, RGB/depth frames, and debug imagery. Camera serials, home paths, temporary tunnel hosts, and remote hostnames are pseudonymized. Quickly review the archive before sending it.

## Clock synchronization

Clock synchronization means the host periodically corrects its wall clock against a trusted network time source using NTP. It does not make the camera or network faster. It makes source timestamps, browser capture-age calculations, and logs from Sydney and the remote operator comparable; a badly wrong clock can make healthy data appear stale or make failure timelines misleading.

Check it with:

```bash
timedatectl status
timedatectl show -p NTPSynchronized --value
```

If the host administrator has not already configured time synchronization, they can enable the standard system service with `sudo timedatectl set-ntp true`. The operator computer should also use its operating system's automatic date/time setting.

## Minimum debugging report

Send the support bundle plus these observations:

- Exact UTC/local time and action that reproduced the issue.
- Whether point cloud, RGB, auxiliary view, or robot control failed.
- Browser name and observed stream FPS/capture age.
- Whether `robot-teleop status` still said `running`.
- Any physical power, USB, or robot changes immediately before the failure.

Do not remotely enable motion, clear calibration, or start an automatic calibration sweep solely as a diagnostic step.
