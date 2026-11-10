"""Opt-in D435 correspondence tuning. The SDK keeps its other thresholds."""

from importlib.util import find_spec
import json
import logging
from pathlib import Path
import re
import threading
import time


SERIAL = re.compile(r"[0-9]{8,16}")
RECOVERY_THRESHOLD = 0
LOG = logging.getLogger(__name__)


class DepthRecovery:
    def __init__(self, settings_path: Path):
        self.path = settings_path
        self.lock = threading.Lock()
        self.available = find_spec("pyrealsense2") is not None

    def _read(self) -> dict[str, int]:
        try:
            saved = json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return {}
        if not isinstance(saved, dict) or not all(
            SERIAL.fullmatch(key) and type(value) is int and 0 <= value <= 1023
            for key, value in saved.items()
        ):
            raise ValueError("Invalid saved depth recovery settings.")
        return saved

    def _save(self, saved: dict[str, int]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(saved, indent=2), encoding="utf-8")
        temporary.replace(self.path)

    def status(self) -> dict:
        with self.lock:
            return {"available": self.available, "enabled": sorted(self._read())}

    @staticmethod
    def _set_threshold(advanced, threshold: int) -> None:
        control = advanced.get_depth_control()
        if control.deepSeaSecondPeakThreshold != threshold:
            control.deepSeaSecondPeakThreshold = threshold
            advanced.set_depth_control(control)
        if advanced.get_depth_control().deepSeaSecondPeakThreshold != threshold:
            raise RuntimeError("The camera did not accept the depth recovery setting.")

    @staticmethod
    def _device(context, serial: str):
        import pyrealsense2 as rs

        for device in context.query_devices():
            if device.get_info(rs.camera_info.serial_number) != serial:
                continue
            if not device.get_info(rs.camera_info.name).endswith(" D435"):
                raise ValueError("Measured depth recovery is available for D435 cameras.")
            advanced = rs.rs400_advanced_mode(device)
            if not advanced.is_enabled():
                raise ValueError("This camera's RealSense advanced controls are unavailable.")
            return advanced
        raise RuntimeError("That D435 camera is not connected.")

    def set_enabled(self, serial: str, enabled: bool) -> dict:
        if not isinstance(serial, str) or not SERIAL.fullmatch(serial) or type(enabled) is not bool:
            raise ValueError("Choose a camera and an on/off depth recovery state.")
        if not self.available:
            raise ValueError("The host has no RealSense Python SDK.")

        import pyrealsense2 as rs

        with self.lock:
            saved = self._read()
            if enabled or serial in saved:
                context = rs.context()
                advanced = self._device(context, serial)
                current = int(advanced.get_depth_control().deepSeaSecondPeakThreshold)
                if enabled:
                    if serial not in saved:
                        # Persist the original before changing the device so a
                        # process exit can't lose the baseline.
                        self._save({**saved, serial: current})
                    try:
                        self._set_threshold(advanced, RECOVERY_THRESHOLD)
                    except RuntimeError:
                        self._set_threshold(advanced, current)
                        self._save(saved)
                        raise
                else:
                    # Preserve a later threshold set outside this control.
                    if current == RECOVERY_THRESHOLD:
                        self._set_threshold(advanced, saved[serial])
                    remaining = {key: value for key, value in saved.items() if key != serial}
                    try:
                        self._save(remaining)
                    except OSError:
                        self._set_threshold(advanced, current)
                        raise
            return {"available": True, "enabled": sorted(self._read())}

    def start(self) -> None:
        if self.available and self.path.is_file():
            threading.Thread(target=self._apply_saved, daemon=True,
                             name="d435-depth-recovery").start()

    def _apply_saved(self) -> None:
        import pyrealsense2 as rs

        for attempt in range(3):
            try:
                with self.lock:
                    saved = self._read()
                    if not saved:
                        return
                    context = rs.context()
                    for serial in saved:
                        self._set_threshold(self._device(context, serial), RECOVERY_THRESHOLD)
                return
            except (OSError, RuntimeError, ValueError) as error:
                if attempt == 2:
                    LOG.warning("Could not reapply D435 depth recovery: %s", error)
                else:
                    time.sleep(2)
