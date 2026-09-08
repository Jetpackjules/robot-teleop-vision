"""Validate the isolated browser -> UDP path with fake input, never hardware."""
from __future__ import annotations

import base64
import io
import json
import os
import queue
import shutil
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path
from urllib.request import Request, urlopen

import pytest

from robot_teleop.doctor import find_godot, godot_console_executable
from tools.serve_alignment_demo import DemoServer, receive_frame, validate_packet

ROOT = Path(__file__).resolve().parents[1]


def packet(now=None):
    return {
        "type": "alignment_demo", "version": 1, "seq": 1,
        "timestamp_ms": int(time.time() * 1000) if now is None else now,
        "source": "fake-browser",
        "head": {"active": True, "x": 2, "y": -1, "z": 35, "units": "cm"},
        "arm": {"active": True, "normalized": [-40, 130, 38, 78, 0, 25], "source": "leader"},
    }


def masked_frame(payload: bytes, opcode=1):
    mask = b"test"
    prefix = bytes([0x80 | opcode, 0x80 | len(payload)]) if len(payload) < 126 else bytes([0x80 | opcode, 0xFE]) + struct.pack("!H", len(payload))
    return prefix + mask + bytes(value ^ mask[index % 4] for index, value in enumerate(payload))


def test_simulation_envelope_is_a_whitelist():
    message = packet(1000)
    message.update(follower_port="COM99", positions=[4095] * 6, command="arm_enable")
    clean = validate_packet(message, now_ms=1000)
    assert set(clean) == {"type", "version", "seq", "timestamp_ms", "source", "head", "arm"}
    assert clean["head"]["x"] == 2
    assert clean["arm"]["normalized"][-1] == 25


@pytest.mark.parametrize("change", [
    {"type": "arm_command"}, {"type": "arm_enable"}, {"action": "arm_enable"},
    {"version": True},
    {"seq": True}, {"seq": -1}, {"timestamp_ms": 0}, {"timestamp_ms": 3000},
    {"head": {"active": True, "x": float("nan"), "y": 0, "z": 35, "units": "cm"}},
    {"head": {"active": True, "x": 0, "y": 0, "z": 35, "units": "meters"}},
    {"arm": {"active": True, "normalized": [0] * 5}},
    {"arm": {"active": True, "normalized": [0, 0, 0, 0, 0, 101]}},
    {"arm": {"active": True, "normalized": [361, 0, 0, 0, 0, 25]}},
])
def test_invalid_or_stale_input_is_rejected(change):
    value = packet(1500)
    value.update(change)
    with pytest.raises(ValueError):
        validate_packet(value, now_ms=1500)


def test_websocket_frames_reject_unmasked_fragmented_and_oversize_input():
    assert receive_frame(io.BytesIO(masked_frame(b"{}"))) == (1, b"{}")
    for content in [b"\x81\x02{}", b"\x01\x80test", b"\x82\x80test", b"\x81\xfe\xff\xff"]:
        with pytest.raises(ValueError):
            receive_frame(io.BytesIO(content))


def open_ws(port, origin=None):
    client = socket.create_connection(("127.0.0.1", port), timeout=3)
    key = base64.b64encode(b"simulation-tests").decode()
    origin = origin or f"http://127.0.0.1:{port}"
    request = (
        f"GET /input HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nOrigin: {origin}\r\n"
        f"Upgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: 13\r\n"
        f"Sec-WebSocket-Key: {key}\r\n\r\n"
    )
    client.sendall(request.encode())
    response = b""
    while b"\r\n\r\n" not in response:
        response += client.recv(1)
    return client, response


def test_real_websocket_relay_to_loopback_and_disconnect_pause():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(2)
        server = DemoServer(http_port=0, udp_port=receiver.getsockname()[1])
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            assert server.server_address[0] == "127.0.0.1"
            with urlopen(f"http://127.0.0.1:{server.server_port}/") as response:
                assert b"Start head tracking" in response.read()
            with urlopen(f"http://127.0.0.1:{server.server_port}/config") as response:
                assert json.load(response) == {
                    "service": "robot-teleop-alignment-demo", "protocol": "alignment_demo",
                    "version": 1, "udp_port": receiver.getsockname()[1],
                }
            bad, response = open_ws(server.server_port, origin="https://untrusted.example")
            with bad:
                assert b"403" in response.split(b"\r\n")[0]
            with pytest.raises(Exception) as failure:
                urlopen(Request(f"http://127.0.0.1:{server.server_port}/config", headers={"Host": "untrusted.example"}))
            assert "403" in str(failure.value)
            client, response = open_ws(server.server_port)
            with client:
                assert b"101" in response.split(b"\r\n")[0]
                message = packet()
                client.sendall(masked_frame(json.dumps(message).encode()))
                received = json.loads(receiver.recv(4096))
                assert received["type"] == "alignment_demo"
                assert received["source"].startswith("browser-")
                assert received["head"] == message["head"]
                assert received["arm"]["normalized"] == message["arm"]["normalized"]
                # A valid but repeated sequence cannot drive the scene twice.
                client.sendall(masked_frame(json.dumps(message).encode()))
                receiver.settimeout(0.15)
                with pytest.raises(TimeoutError):
                    receiver.recv(4096)
                receiver.settimeout(2)
                client.sendall(masked_frame(b"", opcode=8))
                paused = json.loads(receiver.recv(4096))
                assert paused["head"]["active"] is False
                assert paused["arm"]["active"] is False
                assert paused["seq"] == 2
        finally:
            server.shutdown()
            server.server_close()
            worker.join(timeout=3)


def test_browser_encoder_reader_uses_only_sync_read_and_calibration_math(tmp_path):
    node = shutil.which("node")
    if not node:
        pytest.skip("Node is needed for the isolated JavaScript reader test")
    module = tmp_path / "controller.mjs"
    shutil.copyfile(ROOT / "examples/alignment_demo/web/simulation_controller.js", module)
    fixture = json.loads((ROOT / "tests/fixtures/so101_arm_pair.json").read_text())
    script = tmp_path / "probe.mjs"
    script.write_text("""
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {makeLeaderReadPacket, packetChecksum, LeaderPacketDecoder,
  parseLeaderCalibration, normalizeLeaderPositions, ReadOnlyLeader} from './controller.mjs';
const fixture = JSON.parse(readFileSync(process.argv[2], 'utf8'));
const calibration = parseLeaderCalibration(fixture);
const raw = [2048, 2048, 2048, 2048, 2048, 2000];
assert.deepEqual(normalizeLeaderPositions(raw, calibration), [0, 0, 0, 0, 0, 50]);
assert.throws(() => parseLeaderCalibration({}));
assert.throws(() => normalizeLeaderPositions([NaN, ...raw.slice(1)], calibration));
const request = makeLeaderReadPacket();
assert.equal(request[4], 0x82); assert.equal(request[5], 56); assert.equal(request[6], 2);
assert.equal(request[request.length-1], packetChecksum(request));
function replies() {
  return Uint8Array.from(raw.flatMap((position, index) => {
    const packet = [255,255,index+1,4,0,position & 255,position >> 8,0];
    packet[7] = packetChecksum(packet); return packet;
  }));
}
const decoder = new LeaderPacketDecoder();
const response = replies();
assert.deepEqual(decoder.push(response.slice(0, 4)), []);
assert.equal(decoder.push(response.slice(4)).length, 6);
const bad = response.slice(0, 8); bad[7] ^= 1;
assert.deepEqual(decoder.push(bad), []);
let pending, queued, writes = [], closed = false, opened = false, chosen = 0;
const reader = {
  read: () => queued ? Promise.resolve({value: (() => {const result = queued; queued=null; return result;})(), done:false}) : new Promise(resolve => pending=resolve),
  cancel: async () => { if (pending) { pending({done:true}); pending=null; } },
  releaseLock: () => {},
};
const port = {
  open: async () => {opened=true;}, close: async () => {closed=true;},
  readable: {getReader: () => reader},
  writable: {getWriter: () => ({write: async bytes => {
    writes.push([...bytes]);
    if (pending) { const resolve=pending; pending=null; resolve({value:replies(),done:false}); }
    else queued=replies();
  }, releaseLock: () => {}})},
};
let resolvePose;
const observed = new Promise(resolve => resolvePose=resolve);
const leader = new ReadOnlyLeader(values => resolvePose(values), () => {});
assert.equal(opened, false); assert.equal(writes.length, 0);
await leader.connect({requestPort: async () => {chosen++;return port;}});
assert.deepEqual(await observed, raw);
await leader.disconnect();
assert.equal(chosen,1); assert.equal(closed,true); assert.ok(writes.length >= 1);
assert.ok(writes.every(bytes => JSON.stringify(bytes) === JSON.stringify([...request])));
// Closing the page while the chooser is unresolved must prevent any late open.
let resolveChoice, lateOpens = 0, lateCloses = 0;
const latePort = {
  open: async () => {lateOpens++;}, close: async () => {lateCloses++;},
  readable: {getReader: () => {throw new Error('must never acquire reader');}},
};
const pendingChooser = new ReadOnlyLeader(() => assert.fail('late sample'), () => {});
const chooserResult = pendingChooser.connect({requestPort: () => new Promise(resolve => resolveChoice=resolve)});
await pendingChooser.disconnect();
resolveChoice(latePort);
await assert.rejects(chooserResult, /cancelled/);
assert.equal(lateOpens, 0); assert.equal(pendingChooser.port, null);
// Cancellation after choosing a device but during port.open must close it.
let resolveOpen;
latePort.open = () => {lateOpens++; return new Promise(resolve => resolveOpen=resolve);};
const pendingOpen = new ReadOnlyLeader(() => assert.fail('late sample'), () => {});
const openResult = pendingOpen.connect({requestPort: async () => latePort});
await new Promise(resolve => setTimeout(resolve, 0));
await pendingOpen.disconnect();
resolveOpen();
await assert.rejects(openResult, /cancelled/);
assert.equal(lateOpens, 1); assert.equal(lateCloses, 1);
assert.equal(pendingOpen.port, null); assert.equal(pendingOpen.running, false);
console.log('READ_ONLY_LEADER_PASS');
""", encoding="utf-8")
    calibration_file = tmp_path / "fixture.json"
    calibration_file.write_text(json.dumps(fixture), encoding="utf-8")
    result = subprocess.run([node, str(script), str(calibration_file)], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "READ_ONLY_LEADER_PASS" in result.stdout


def test_relay_drives_running_godot_head_and_virtual_joints(tmp_path):
    """Actual browser-format WebSocket -> UDP -> running scene, no devices."""
    godot = find_godot(os.environ.get("ROBOT_TELEOP_TEST_GODOT", ""))
    if godot is None:
        pytest.skip("Set ROBOT_TELEOP_TEST_GODOT for the executable relay/scene check")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        udp_port = reservation.getsockname()[1]
    server = DemoServer(http_port=0, udp_port=udp_port)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    environment = {
        **os.environ, "APPDATA": str(tmp_path / "appdata"),
        "XDG_DATA_HOME": str(tmp_path / "data"),
    }
    environment.pop("ROBOT_TELEOP_RUNTIME_CONFIG", None)
    command = [
        str(godot_console_executable(godot)), "--headless", "--path", str(ROOT),
        "--log-file", str(tmp_path / "relay-probe.log"),
        "--script", "res://tests/godot/alignment_demo_relay_probe.gd", "--",
        f"--simulation-input-port={udp_port}",
    ]
    process = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
    output = []
    lines = queue.Queue()

    def collect():
        for line in process.stdout:
            output.append(line)
            lines.put(line)

    collector = threading.Thread(target=collect, daemon=True)
    collector.start()

    def wait_for(marker):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                line = lines.get(timeout=0.2)
            except queue.Empty:
                if process.poll() is not None:
                    break
                continue
            if marker in line:
                return line
        pytest.fail(f"Missing {marker}\n{''.join(output)}")

    client = None
    try:
        wait_for("ALIGNMENT_RELAY_READY")
        client, response = open_ws(server.server_port)
        assert b"101" in response.split(b"\r\n")[0]
        first = packet()
        first["head"].update(x=0, y=0, z=60)
        first["arm"]["normalized"][-1] = 75
        client.sendall(masked_frame(json.dumps(first).encode()))
        wait_for("ALIGNMENT_RELAY_STAGE1")
        second = packet()
        second["seq"] = 2
        second["head"].update(x=10, y=5, z=65)
        second["arm"]["normalized"] = [-30, 120, 42, 88, 15, 20]
        client.sendall(masked_frame(json.dumps(second).encode()))
        wait_for("ALIGNMENT_RELAY_STAGE2")
        stale = packet()
        stale["seq"] = 3
        stale["timestamp_ms"] -= 5000
        client.sendall(masked_frame(json.dumps(stale).encode()))
        result_line = wait_for("ALIGNMENT_RELAY_PROBE ")
        result = json.loads(result_line.split("ALIGNMENT_RELAY_PROBE ", 1)[1])
        assert result["passed"], result
        assert result["final"]["accepted_packets"] == 2
        assert result["final"]["gripper"] == 20
        assert process.wait(timeout=5) == 0, "".join(output)
        assert "SCRIPT ERROR" not in "".join(output), "".join(output)
    finally:
        if client:
            client.close()
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        server.shutdown()
        server.server_close()
        worker.join(timeout=3)
        collector.join(timeout=3)
