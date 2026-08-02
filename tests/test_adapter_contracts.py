from robot_teleop.config import RobotConfig
from robot_teleop.registry import create


def test_disabled_robot_accepts_standard_factory_configuration():
    robot = create("robot", "disabled", config=RobotConfig())

    assert robot.launch_spec() is None
    assert robot.hold() is None
