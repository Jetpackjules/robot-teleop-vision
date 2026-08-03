from robot_teleop.cli import _initialize
from robot_teleop.config import load_config


def test_init_enables_cloudflare_with_generated_password(tmp_path):
    destination = tmp_path / "local.toml"

    assert _initialize(destination, "vision_only", force=False) == 0

    config = load_config(destination)
    assert config.stack.public_mode == "quick"
    assert config.stack.password_default != "change-me"
    assert len(config.stack.password_default) >= 24
    assert destination.stat().st_mode & 0o777 == 0o600
