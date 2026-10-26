"""The camera-only process must retain the control owner's registered overlay."""
import os
import subprocess
from pathlib import Path

import pytest

from robot_teleop.doctor import find_godot, godot_console_executable

ROOT = Path(__file__).resolve().parents[1]


def functions(path, *names):
    source = (ROOT / path).read_text(encoding="utf-8")
    result = []
    for name in names:
        start = source.index(f"\nfunc {name}(")
        end = source.find("\nfunc ", start + 1)
        result.append(source[start:end if end != -1 else None])
    return "\n".join(result)


def test_registered_overlay_survives_independent_camera_capture(tmp_path):
    godot = find_godot(os.environ.get("ROBOT_TELEOP_TEST_GODOT", ""))
    if godot is None:
        pytest.skip("Set ROBOT_TELEOP_TEST_GODOT for executable GDScript tests")
    # Exercise the actual production writer, mirror reader, and RGBD1 packet
    # builder in an isolated user directory, without loading camera or motors.
    (tmp_path / "project.godot").write_text(
        '[application]\nconfig/name="Overlay transport test"\n', encoding="utf-8"
    )
    (tmp_path / "writer.gd").write_text(
        'extends Node\nconst OVERLAY_PATH = "../Overlay"\n'
        'var _last_status_send_msec = -1000\nvar _status_udp = PacketPeerUDP.new()\n'
        'func get_calibration_status() -> Dictionary: return {"state":"complete"}\n'
        + functions("robot_modules/so101/godot/so101_motion_calibrator.gd",
                    "_emit_status", "_write_runtime_mirror"), encoding="utf-8"
    )
    (tmp_path / "reader.gd").write_text(
        'extends Node\nvar rgbd_root_path = NodePath("..")\n'
        'var _last_rgbd_frame_bytes = PackedByteArray()\nvar _rgbd_frame_counter = 0\n'
        + functions("godot/runtime/streaming/viewport_stream_server.gd",
                    "_robot_overlay_metadata", "_publish_rgbd_packet", "_transform_array"),
        encoding="utf-8",
    )
    (tmp_path / "probe.gd").write_text(r'''
extends SceneTree
const PATH = "user://robot_overlay_render_state.json"
class Overlay:
    extends Node
    func get_hybrid_render_state():
        return {"available":true,"links":{"base_link":{"origin":[0.2,0.04,-0.4]}},
            "registration":{"registered":true,"confidence":0.72}}
class Gateway:
    extends Node
    var control_owner = true
    func owns_robot_control(): return control_owner
func write_mirror(value):
    var file = FileAccess.open(PATH, FileAccess.WRITE)
    file.store_string(JSON.stringify(value))
    file.close()
func _initialize(): call_deferred("run")
func run():
    OS.set_environment("ROBOT_TELEOP_CAPTURE_ONLY", "1")
    var view = Node.new()
    root.add_child(view)
    var writer = load("res://writer.gd").new()
    view.add_child(writer)
    # Avoid sending even the test's status packet to an unrelated socket.
    writer._status_udp.connect_to_host("127.0.0.1", 9)
    var reader = load("res://reader.gd").new()
    view.add_child(reader)
    var overlay = Overlay.new()
    overlay.name = "Overlay"
    view.add_child(overlay)
    var gateway = Gateway.new()
    gateway.name = "RemoteControlGateway"
    view.add_child(gateway)
    assert(reader._robot_overlay_metadata() == {"available":false})
    writer._emit_status(true)
    var expected = overlay.get_hybrid_render_state()
    assert(reader._robot_overlay_metadata() == expected)
    var status = JSON.parse_string(FileAccess.get_file_as_string("user://so101_runtime_calibration_status.json"))
    assert(status.status.state == "complete")
    var cameras: Array[Dictionary] = [{"id":"test", "color_bytes":3,"depth_bytes":2}]
    var payloads: Array[PackedByteArray] = [PackedByteArray([1,2,3]),PackedByteArray([4,5])]
    assert(reader._publish_rgbd_packet(cameras, payloads, 123.0))
    var packet: PackedByteArray = reader._last_rgbd_frame_bytes
    assert(packet.slice(0,5).get_string_from_ascii() == "RGBD1")
    var end = 9 + packet.decode_u32(5)
    var metadata = JSON.parse_string(packet.slice(9,end).get_string_from_utf8())
    assert(metadata.robot == expected)
    assert(metadata.cameras[0].id == "test" and metadata.capture_unix_ms == 123.0)
    assert(packet.slice(end) == PackedByteArray([1,2,3,4,5]))
    var good = JSON.parse_string(FileAccess.get_file_as_string(PATH))
    var bad = good.duplicate(true)
    bad.updated -= 10
    write_mirror(bad)
    assert(reader._robot_overlay_metadata() == {"available":false})
    bad = good.duplicate(true)
    bad.updated += 100
    write_mirror(bad)
    assert(reader._robot_overlay_metadata() == {"available":false})
    bad = good.duplicate(true)
    bad.project_root = "/another/project/"
    write_mirror(bad)
    assert(reader._robot_overlay_metadata() == {"available":false})
    bad = good.duplicate(true)
    bad.robot = []
    write_mirror(bad)
    assert(reader._robot_overlay_metadata() == {"available":false})
    write_mirror(null)
    assert(reader._robot_overlay_metadata() == {"available":false})
    # A spectator scene must not overwrite the control owner's mirror.
    write_mirror(good)
    var before = FileAccess.get_file_as_string(PATH)
    gateway.control_owner = false
    writer._emit_status(true)
    assert(FileAccess.get_file_as_string(PATH) == before)
    # A local overlay takes precedence, even over an expired mirror.
    overlay.add_to_group("robot_overlay")
    write_mirror(bad)
    assert(reader._robot_overlay_metadata() == expected)
    overlay.remove_from_group("robot_overlay")
    gateway.control_owner = true
    overlay.free()
    writer._emit_status(true)
    assert(reader._robot_overlay_metadata() == {"available":false})
    OS.set_environment("ROBOT_TELEOP_CAPTURE_ONLY", "0")
    assert(reader._robot_overlay_metadata() == {})
    view.free()
    print("OVERLAY_TRANSPORT_PASS")
    quit()
''', encoding="utf-8")
    result = subprocess.run(
        [str(godot_console_executable(godot)), "--headless", "--path", str(tmp_path),
         "--script", "res://probe.gd", "--quit-after", "60"],
        env={**os.environ, "APPDATA": str(tmp_path / "appdata"),
             "XDG_DATA_HOME": str(tmp_path / "data")},
        capture_output=True, text=True, timeout=20, check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "SCRIPT ERROR" not in output, output
    assert "OVERLAY_TRANSPORT_PASS" in output, output
