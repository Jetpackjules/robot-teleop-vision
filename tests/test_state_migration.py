from pathlib import Path

from robot_teleop.state import migrate_project_state


def test_state_migration_copies_only_durable_files(tmp_path: Path):
    source = tmp_path / "Legacy"
    source.mkdir()
    (source / "so101_robot_registration.json").write_text('{"trusted": true}')
    (source / "so101_automated_joint_capture.json").write_text("private capture")

    copied = migrate_project_state(
        "Legacy",
        root=tmp_path,
        extra_state_files=("so101_robot_registration.json",),
    )

    assert [path.name for path in copied] == ["so101_robot_registration.json"]
    assert (tmp_path / "Robot Teleop Vision" / "so101_robot_registration.json").is_file()
    assert not (tmp_path / "Robot Teleop Vision" / "so101_automated_joint_capture.json").exists()


def test_state_migration_does_not_overwrite_without_force(tmp_path: Path):
    source = tmp_path / "Legacy"
    target = tmp_path / "Robot Teleop Vision"
    source.mkdir()
    target.mkdir()
    name = "unified_world_level.json"
    (source / name).write_text("new")
    (target / name).write_text("current")

    assert migrate_project_state("Legacy", root=tmp_path) == []
    assert (target / name).read_text() == "current"
    migrate_project_state("Legacy", root=tmp_path, force=True)
    assert (target / name).read_text() == "new"
