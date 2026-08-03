from robot_teleop.registry import register


class DisabledRobotAdapter:
    name = "disabled"

    def __init__(self, **_kwargs) -> None:
        pass

    def launch_spec(self):
        return None

    def hold(self) -> None:
        return None

    def public_manifest(self) -> dict:
        return {
            "schema_version": 1,
            "id": self.name,
            "label": "Vision only",
            "version": "1.0.0",
            "description": "Run cameras and the operator view without physical robot control.",
            "capabilities": {
                "control_spaces": [],
                "inputs": [],
                "actions": [],
                "views": ["virtual_camera"],
                "features": [],
            },
            "entrypoints": {},
        }

    def operator_environment(self) -> dict[str, str]:
        return {}


register("robot", DisabledRobotAdapter.name, DisabledRobotAdapter)
