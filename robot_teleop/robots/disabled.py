from robot_teleop.registry import register


class DisabledRobotAdapter:
    name = "disabled"

    def __init__(self, **_kwargs) -> None:
        pass

    def launch_spec(self):
        return None

    def hold(self) -> None:
        return None


register("robot", DisabledRobotAdapter.name, DisabledRobotAdapter)
