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
from tools import serve_alignment_demo
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


def open_ws(port, origin=None, client_id=None, opened_ms=None):
    client = socket.create_connection(("127.0.0.1", port), timeout=3)
    key = base64.b64encode(b"simulation-tests").decode()
    origin = origin or f"http://127.0.0.1:{port}"
    target = "/input" + (f"?client={client_id}" if client_id else "")
    if opened_ms is not None:
        target += ("&" if "?" in target else "?") + f"opened={opened_ms}"
    request = (
        f"GET {target} HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nOrigin: {origin}\r\n"
        f"Upgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: 13\r\n"
        f"Sec-WebSocket-Key: {key}\r\n\r\n"
    )
    client.sendall(request.encode())
    response = b""
    while b"\r\n\r\n" not in response:
        response += client.recv(1)
    return client, response


def server_frame(client):
    def exact(size):
        result = b""
        while len(result) < size:
            chunk = client.recv(size - len(result))
            assert chunk, "Socket closed before complete server frame"
            result += chunk
        return result

    first, second = exact(2)
    assert not second & 0x80
    size = second & 127
    if size == 126:
        size = struct.unpack("!H", exact(2))[0]
    return first & 15, exact(size)


@pytest.fixture
def local_relay():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(2)
        server = DemoServer(http_port=0, udp_port=receiver.getsockname()[1])
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            yield server, receiver
        finally:
            server.shutdown()
            server.server_close()
            worker.join(timeout=3)


def test_newest_tab_takes_over_and_old_reconnect_cannot_steal_input(local_relay):
    server, receiver = local_relay
    old, _ = open_ws(server.server_port, client_id="old-page")
    new = retry = None
    try:
        assert json.loads(server_frame(old)[1])["state"] == "owner"
        old.sendall(masked_frame(json.dumps(packet()).encode()))
        original = json.loads(receiver.recv(4096))
        new, response = open_ws(server.server_port, client_id="new-page")
        assert b"101" in response
        assert json.loads(server_frame(new)[1])["state"] == "owner"
        assert json.loads(server_frame(old)[1])["state"] == "superseded"
        opcode, close = server_frame(old)
        assert opcode == 8 and struct.unpack("!H", close[:2])[0] == 4001
        paused = json.loads(receiver.recv(4096))
        assert paused["source"] == original["source"]
        assert not paused["head"]["active"] and not paused["arm"]["active"]
        new.sendall(masked_frame(json.dumps(packet()).encode()))
        accepted = json.loads(receiver.recv(4096))
        assert accepted["source"] != original["source"]
        assert accepted["arm"]["active"]
        # A delayed network retry from the old page remains permanently revoked.
        retry, response = open_ws(server.server_port, client_id="old-page")
        assert b"101" in response
        assert json.loads(server_frame(retry)[1])["state"] == "superseded"
        # Old handler teardown/rejected reconnect must not pause the new owner.
        receiver.settimeout(0.2)
        with pytest.raises(TimeoutError):
            receiver.recv(4096)
        receiver.settimeout(2)
        next_packet = packet()
        next_packet["seq"] = 2
        new.sendall(masked_frame(json.dumps(next_packet).encode()))
        assert json.loads(receiver.recv(4096))["arm"]["active"]
    finally:
        for client in (old, new, retry):
            if client:
                client.close()


def test_idle_owner_survives_old_three_second_timeout_and_rejects_stale_input(local_relay):
    server, receiver = local_relay
    client, _ = open_ws(server.server_port, client_id="idle-page")
    with client:
        assert json.loads(server_frame(client)[1])["state"] == "owner"
        time.sleep(3.15)
        stale = packet()
        stale["timestamp_ms"] -= 5000
        client.sendall(masked_frame(json.dumps(stale).encode()))
        assert "error" in json.loads(server_frame(client)[1])
        receiver.settimeout(0.1)
        with pytest.raises(TimeoutError):
            receiver.recv(4096)
        receiver.settimeout(2)
        client.sendall(masked_frame(json.dumps(packet()).encode()))
        assert json.loads(receiver.recv(4096))["arm"]["active"]


def test_disconnected_old_identity_stays_revoked_after_new_tab(local_relay):
    server, receiver = local_relay
    old, _ = open_ws(server.server_port, client_id="lost-page")
    assert json.loads(server_frame(old)[1])["state"] == "owner"
    old.sendall(masked_frame(json.dumps(packet()).encode()))
    receiver.recv(4096)
    old.close()
    assert not json.loads(receiver.recv(4096))["arm"]["active"]
    new, _ = open_ws(server.server_port, client_id="latest-page")
    with new:
        assert json.loads(server_frame(new)[1])["state"] == "owner"
        late, _ = open_ws(server.server_port, client_id="lost-page")
        with late:
            assert json.loads(server_frame(late)[1])["state"] == "superseded"


def test_older_page_arriving_late_after_relay_restart_does_not_take_over(local_relay):
    server, _ = local_relay
    newer, _ = open_ws(server.server_port, client_id="newer-page", opened_ms=2000)
    with newer:
        assert json.loads(server_frame(newer)[1])["state"] == "owner"
        delayed, _ = open_ws(server.server_port, client_id="older-page", opened_ms=1000)
        with delayed:
            assert json.loads(server_frame(delayed)[1])["state"] == "superseded"


def test_preview_exposes_only_fixed_fresh_synthetic_output(local_relay, tmp_path, monkeypatch):
    server, _ = local_relay
    monkeypatch.setattr(serve_alignment_demo, "PREVIEW_DIRECTORY", tmp_path)
    state_file = tmp_path / "preview-state.json"
    state = {"running": True, "timestamp_ms": int(time.time() * 1000),
             "render_mode": "solid", "private_path": "must not be exposed"}
    state_file.write_text(json.dumps(state), encoding="utf-8")
    (tmp_path / "preview.jpg").write_bytes(b"\xff\xd8fake-synthetic-preview\xff\xd9")
    with urlopen(f"http://127.0.0.1:{server.server_port}/preview-state.json") as response:
        visible = json.load(response)
        assert visible["running"] and visible["render_mode"] == "solid"
        assert "private_path" not in visible
    with urlopen(f"http://127.0.0.1:{server.server_port}/preview.jpg?frame=1") as response:
        assert response.headers["Content-Type"] == "image/jpeg"
        assert response.read().startswith(b"\xff\xd8")
    state["timestamp_ms"] -= 5000
    state_file.write_text(json.dumps(state), encoding="utf-8")
    for path in ("preview-state.json", "preview.jpg"):
        with pytest.raises(Exception) as failure:
            urlopen(f"http://127.0.0.1:{server.server_port}/{path}")
        assert "503" in str(failure.value)


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
                    "version": 1, "controller_revision": 2, "udp_port": receiver.getsockname()[1],
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


def test_browser_relay_takeover_cleanup_and_bounded_reconnect(tmp_path):
    node = shutil.which("node")
    if not node:
        pytest.skip("Node is needed for the isolated controller lifecycle test")
    shutil.copyfile(ROOT / "examples/alignment_demo/web/simulation_controller.js", tmp_path / "controller.mjs")
    script = tmp_path / "relay-lifecycle.mjs"
    script.write_text("""
import assert from 'node:assert/strict';
import {RelayConnection, ReadOnlyLeader, sceneInputReady} from './controller.mjs';
assert.equal(sceneInputReady({running:true,input_bound:false,input_port:14861},14861),false);
assert.equal(sceneInputReady({running:true,input_bound:true,input_port:14862},14861),false);
assert.equal(sceneInputReady({running:true,input_bound:true,input_port:14861},14861),true);
class FakeSocket {
  constructor() { this.readyState=0; this.bufferedAmount=0; this.listeners={}; this.sent=[]; }
  addEventListener(name, callback) { this.listeners[name]=callback; }
  emit(name, event={}) { this.listeners[name]?.(event); }
  owner() { this.readyState=1; this.emit('message', {data:JSON.stringify({type:'controller_status', state:'owner'})}); }
  send(data) { this.sent.push(data); }
  close() { this.readyState=3; this.emit('close', {code:1000}); }
  lose(code=1006) { this.readyState=3; this.emit('close', {code}); }
}
let serialCloses=0, cameraStops=0, runningCamera=true, manual=true, releaseCount=0;
const leader = new ReadOnlyLeader(() => {}, () => {});
// These represent already user-enabled devices; reconnect never creates them.
leader.port={close:async () => {serialCloses++;}}; leader.running=true;
const sockets=[], states=[], timers=new Map(), delays=[];
let nextTimer=0;
const relay = new RelayConnection({url:'ws://localhost/input?client=fake-page',
  createSocket:() => {const socket=new FakeSocket(); sockets.push(socket); return socket;},
  onState:state => states.push(state),
  releaseInputs:() => {
    releaseCount++; manual=false;
    if (runningCamera) {runningCamera=false;cameraStops++;}
    void leader.disconnect();
  },
  setTimer:(callback, delay) => {const id=nextTimer++;timers.set(id,callback);delays.push(delay);return id;},
  clearTimer:id => timers.delete(id),
});
relay.connect(); sockets[0].owner(); assert.ok(relay.connected);
assert.ok(relay.send({type:'fake'}));
sockets[0].lose(); await leader.stopping;
assert.equal(runningCamera,false); assert.equal(manual,false); assert.equal(leader.running,false);
assert.equal(serialCloses,1); assert.equal(cameraStops,1);
assert.deepEqual(delays,[250]);
// Visibility/focus can accelerate recovery, without restarting any device.
relay.recover(); assert.equal(sockets.length,2); assert.equal(timers.size,0);
sockets[1].owner(); assert.ok(relay.connected);
assert.equal(leader.port,null); assert.equal(runningCamera,false); assert.equal(manual,false);
// Supersession is terminal, even if close/status/focus arrive out of order.
sockets[1].emit('message',{data:JSON.stringify({type:'controller_status',state:'superseded'})});
assert.equal(relay.state,'superseded'); assert.equal(relay.send({}),false);
relay.recover(); relay.connect(); sockets[1].lose();
sockets[1].emit('message',{data:JSON.stringify({type:'controller_status',state:'owner'})});
assert.equal(sockets.length,2); assert.equal(timers.size,0); assert.equal(relay.state,'superseded');
assert.equal(releaseCount,2);
// Retry delays grow but never exceed five seconds; stop cancels them.
const retrySockets=[], retryTimers=new Map(), retryDelays=[];
let retryId=0;
const retry = new RelayConnection({url:'ws://localhost/input?client=recover-page',
  createSocket:()=>{const socket=new FakeSocket();retrySockets.push(socket);return socket;},
  onState:()=>{},releaseInputs:()=>{},
  setTimer:(callback,delay)=>{const id=retryId++;retryTimers.set(id,callback);retryDelays.push(delay);return id;},
  clearTimer:id=>retryTimers.delete(id),
});
retry.connect();
for(let index=0;index<9;index++) {
  retrySockets.at(-1).lose();
  const [id, callback]=retryTimers.entries().next().value;retryTimers.delete(id);callback();
}
assert.deepEqual(retryDelays.slice(0,6),[250,500,1000,2000,4000,5000]);
assert.ok(retryDelays.every(delay=>delay<=5000));
retrySockets.at(-1).lose(4001); retry.recover();
assert.equal(retry.state,'superseded'); assert.equal(retryTimers.size,0);
retry.stop(); assert.equal(retry.state,'stopped');
console.log('RELAY_LIFECYCLE_PASS');
""", encoding="utf-8")
    result = subprocess.run([node, str(script)], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "RELAY_LIFECYCLE_PASS" in result.stdout


def test_example_webcam_cancels_late_permission_and_pending_model(tmp_path):
    node = shutil.which("node")
    if not node:
        pytest.skip("Node is needed for the example webcam cancellation test")
    script = tmp_path / "camera-cancellation.mjs"
    script.write_text("""
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
const source = readFileSync(process.argv[2], 'utf8');
function deferred() {let resolve,reject; const promise=new Promise((done,fail)=>{resolve=done;reject=fail;});return {promise,resolve,reject};}
function setup({enumerate=async()=>[], camera, model=async()=>({}), fakeModule=null}) {
  const video={srcObject:null,plays:0,play:async()=>{video.plays++;},pause:()=>{}};
  const context=vm.createContext({
    window:{cancelAnimationFrame:()=>{}}, console:{warn:()=>{}}, DOMException, Date,
    navigator:{mediaDevices:{getUserMedia:camera}}, video, enumerate, model, fakeModule, loops:0,
  });
  vm.runInContext(source, context);
  vm.runInContext(`
    state.video=video;
    state.webglInfo={webgl1:true};
    placeVideoElement=()=>video;
    listVideoDevices=()=>enumerate();
    state.mediapipeModule=fakeModule;
    if (!fakeModule) initLandmarker=()=>model();
    loop=()=>{loops++;};
    updatePreview=()=>{}; updateFaceOverlay=()=>{};
  `, context);
  return {context,video,start:()=>context.window.startGodotWebcamTracker({backend:'mediapipe',remoteEnabled:false,preview:false}),stop:()=>context.window.stopGodotWebcamTracker()};
}
// Ownership can disappear while device enumeration is unresolved.
const devices=deferred();let requests=0;
const enumerating=setup({enumerate:()=>devices.promise,camera:()=>{requests++;assert.fail('camera opened after cancel');}});
const enumerateRejected=assert.rejects(enumerating.start(),/cancelled/);
enumerating.stop();devices.resolve([]);await enumerateRejected;
assert.equal(requests,0);
// A late permission response must stop every returned track before attachment,
// video.play(), or MediaPipe initialization, even if the model would hang.
const permission=deferred(), requested=deferred();let stopped=0, modelCalls=0;
const stream={getTracks:()=>[{stop:()=>{stopped++;}},{stop:()=>{stopped++;}}]};
const pendingPermission=setup({camera:()=>{requested.resolve();return permission.promise;},model:()=>{modelCalls++;return new Promise(()=>{});}});
const permissionRejected=assert.rejects(pendingPermission.start(),/cancelled/);
await requested.promise;pendingPermission.stop();permission.resolve(stream);await permissionRejected;
assert.equal(stopped,2);assert.equal(pendingPermission.video.srcObject,null);
assert.equal(pendingPermission.video.plays,0);assert.equal(modelCalls,0);
// If model loading is already pending, stop releases the assigned stream now;
// later model completion cannot set running=true or start the tracking loop.
const modelReady=deferred(), modelBegan=deferred();stopped=0;
const pendingModel=setup({camera:async()=>stream,model:()=>{modelBegan.resolve();return modelReady.promise;}});
const modelRejected=assert.rejects(pendingModel.start(),/cancelled/);
await modelBegan.promise;assert.equal(pendingModel.video.srcObject,stream);
pendingModel.stop();assert.equal(stopped,2);assert.equal(pendingModel.video.srcObject,null);
modelReady.resolve({});await modelRejected;
assert.equal(pendingModel.context.loops,0);
assert.equal(vm.runInContext('state.running',pendingModel.context),false);
// The unchanged successful path still starts after explicit permission.
const healthy=setup({camera:async()=>stream});await healthy.start();
assert.equal(healthy.context.loops,1);assert.equal(healthy.video.srcObject,stream);healthy.stop();
// A cancelled permission request completing after a new explicit start must
// stop only its own late tracks, never the new session's stream.
const late=deferred(), lateRequested=deferred();let cameraCalls=0, oldStops=0, newStops=0;
const oldStream={getTracks:()=>[{stop:()=>{oldStops++;}}]};
const newStream={getTracks:()=>[{stop:()=>{newStops++;}}]};
const overlapping=setup({camera:()=>++cameraCalls===1 ? (lateRequested.resolve(),late.promise) : Promise.resolve(newStream)});
const lateRejected=assert.rejects(overlapping.start(),/cancelled/);
await lateRequested.promise;overlapping.stop();await overlapping.start();
late.resolve(oldStream);await lateRejected;
assert.equal(oldStops,1);assert.equal(newStops,0);assert.equal(overlapping.video.srcObject,newStream);
assert.equal(vm.runInContext('state.running',overlapping.context),true);overlapping.stop();
// Exercise the real model initializer too: an old delayed model closes only
// itself, without clearing the newer model or starting an obsolete loop.
const oldModelReady=deferred(), oldModelBegan=deferred();let modelCreates=0, oldModelClosed=0, newModelClosed=0;
const oldModel={close:()=>{oldModelClosed++;}}, newModel={close:()=>{newModelClosed++;}};
const modelOverlap=setup({camera:async()=>newStream,fakeModule:{
  FilesetResolver:{forVisionTasks:async()=>({})},
  FaceLandmarker:{createFromOptions:()=>++modelCreates===1 ? (oldModelBegan.resolve(),oldModelReady.promise) : Promise.resolve(newModel)},
}});
const oldModelRejected=assert.rejects(modelOverlap.start(),/cancelled/);
await oldModelBegan.promise;modelOverlap.stop();await modelOverlap.start();
oldModelReady.resolve(oldModel);await oldModelRejected;
assert.equal(oldModelClosed,1);assert.equal(newModelClosed,0);
assert.equal(vm.runInContext('state.landmarker',modelOverlap.context),newModel);
assert.equal(vm.runInContext('state.running',modelOverlap.context),true);
assert.equal(modelOverlap.context.loops,1);modelOverlap.stop();
// Run the actual controller Stop/Start handlers, substituting only the module
// import boundary. Stale resolve/reject callbacks must not stop a newer camera.
const controller=readFileSync(process.argv[3],'utf8');
const handlersSource=controller.slice(controller.indexOf('  async function stopWebcam()'),controller.indexOf('  byId("head-stop").addEventListener'))
  .replace('await import("/example_webcam_tracker_bridge.js");','await loadTracker();');
for (const rejects of [false,true]) {
  const first=deferred(), second=deferred(), firstBegan=deferred(), secondBegan=deferred();
  let calls=0, stops=0;const handlers={}, nodes={};
  const byId=id=>nodes[id]??=( {value:'',textContent:'',replaceChildren:()=>{},add:()=>{},addEventListener:(_name,fn)=>{handlers[id]=fn;}} );
  const owner=vm.createContext({
    byId, headGeneration:0, webcamEnabled:false, webcamPending:false,pendingAction:null,
    relay:{connected:true},sceneConnected:true,refreshControls:()=>{},loadTracker:async()=>{},Option:function(){},
    window:{startGodotWebcamTracker:()=>++calls===1 ? (firstBegan.resolve(),first.promise) : (secondBegan.resolve(),second.promise),
      stopGodotWebcamTracker:()=>{stops++;},listGodotWebcamTrackerDevices:async()=>[]},
  });
  vm.runInContext(handlersSource,owner);
  const oldStart=handlers['head-start']();await firstBegan.promise;await owner.stopWebcam();
  const newStart=handlers['head-start']();await secondBegan.promise;
  if (rejects) first.reject(new Error('cancelled'));else first.resolve(true);
  await oldStart;assert.equal(owner.webcamPending,true);assert.equal(stops,1);
  second.resolve(true);await newStart;
  assert.equal(owner.webcamEnabled,true);assert.equal(owner.webcamPending,false);assert.equal(stops,1);
}
console.log('EXAMPLE_CAMERA_CANCEL_PASS');
""", encoding="utf-8")
    result = subprocess.run([
        node, str(script), str(ROOT / "examples/alignment_demo/web/example_webcam_tracker_bridge.js"),
        str(ROOT / "examples/alignment_demo/web/simulation_controller.js"),
    ], capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "EXAMPLE_CAMERA_CANCEL_PASS" in result.stdout


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
