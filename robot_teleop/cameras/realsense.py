from __future__ import annotations

from robot_teleop.interfaces import DeviceInfo
from robot_teleop.registry import register


class RealSenseCameraAdapter:
    name = "realsense"

    def discover(self) -> list[DeviceInfo]:
        try:
            import pyrealsense2 as rs
        except ImportError:
            return []
        devices: list[DeviceInfo] = []
        for device in rs.context().query_devices():
            def value(field) -> str:
                return device.get_info(field) if device.supports(field) else ""

            serial = value(rs.camera_info.serial_number)
            model = value(rs.camera_info.name) or "RealSense"
            devices.append(
                DeviceInfo(
                    adapter=self.name,
                    identifier=serial,
                    label=f"{model} ({serial})" if serial else model,
                    metadata={
                        "model": model,
                        "firmware": value(rs.camera_info.firmware_version),
                        "physical_port": value(rs.camera_info.physical_port),
                        "product_id": value(rs.camera_info.product_id),
                    },
                )
            )
        return devices


register("camera", RealSenseCameraAdapter.name, RealSenseCameraAdapter)
