# Private GPU video of Godot

The operator gateway can serve `/godot/live.html` with full desktop resolution,
VAAPI H.264 video, and noVNC mouse/keyboard control. This is a separate debugging
page; it adds no controls to the normal robot controller. It is disabled unless
an explicit local X11 display is configured and requires the existing password
gate. A display such as `:99` must already exist; it does not access or unlock
the workstation's physical Wayland desktop.

Godot must itself have GPU acceleration configured (for example, VirtualGL EGL
on a private Xvfb display). The video encoder uses a separate VAAPI video engine;
it cannot turn a slow Godot renderer into a 30 FPS scene. The toolbar reports
decoded video FPS separately from scene FPS and buffering delay.

## Existing private desktop

Install FFmpeg with `h264_vaapi`, a working VAAPI driver, x11vnc, noVNC and
websockify. Keep the VNC and websockify listeners on loopback. For example, with
an existing authenticated private `:99` display:

```sh
x11vnc -display :99 -auth /absolute/path/Xauthority \
  -rfbport 5905 -localhost -forever -shared -nopw -noxdamage -quiet
python -m websockify --web /usr/share/novnc 127.0.0.1:6085 127.0.0.1:5905
```

Add these options to the existing password-protected operator gateway command:

```sh
--godot-desktop-display :99 \
--godot-desktop-xauthority /absolute/path/Xauthority \
--godot-desktop-size 1440x900 --godot-desktop-fps 30 \
--godot-desktop-device /dev/dri/renderD128 \
--godot-desktop-vnc-port 6085
```

Use `--godot-desktop-ffmpeg /absolute/path/ffmpeg` for a private installation.
FFmpeg must inherit any library search path that installation needs. Dimensions
must match the desktop; no scaling or camera-resolution changes are performed.

Open the gateway's `/godot/live.html` through the existing authenticated HTTPS
connection or tunnel. The page sends an initial noVNC image, then uses video for
the desktop while noVNC carries input. A one-pixel heartbeat keeps idle input
connections alive. The **Standard desktop** link retains normal noVNC streaming.

Encoding starts on demand and stops when the video connection closes. At most
two encoders run concurrently. Socket and media queues are bounded, old playback
is discarded when a client falls behind, and input reconnects independently.
Video uses high-quality, low-latency H.264 (QP 18, no B frames); video compression
is lossy and separate from the unchanged depth grid and geometry resolution.

The stream server's loopback `/diagnostics` endpoint supplies scene FPS and
camera sizes/stride to the authenticated desktop status route. It does not change
capture profiles, run calibration, or read GPU images back to the CPU.

The capture command follows [FFmpeg's X11 capture documentation](https://ffmpeg.org/ffmpeg-devices.html#x11grab)
and [VAAPI encoder options](https://ffmpeg.org/ffmpeg-all.html#VAAPI-encoders).
