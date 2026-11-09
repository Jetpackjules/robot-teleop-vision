"""Explicit calibration uses the robot socket, with acknowledgement and no replay."""
import json
import threading
from types import SimpleNamespace

import pytest
from test_remote_server_security import bare_handler
from test_so101_leader_coordinates import run_node


def request_handler(*, fresh=True, calibration="idle", paused=False, owner="page"):
    handler = bare_handler()
    server = handler.server
    server.robot_controller_lock = threading.Lock()
    server.robot_controller_id = owner
    server.trial_control_lock = threading.Lock()
    server.episode_recorder = SimpleNamespace(lock=threading.Lock(), state="paused" if paused else "idle")
    server.latest_robot_status = lambda *_: {"follower_connected": fresh, "measured_feedback_fresh": fresh,
                                            "robot_calibration": {"state": calibration}}
    sent, replies = [], []
    server.forward_tracking = lambda payload: sent.append(json.loads(payload))
    handler.send_robot_status = lambda owner, reply=None: replies.append(reply)
    return handler, sent, replies


def send(handler, settings):
    handler.handle_robot_view_settings({"type": "robot_view_settings", "request_id": "request-1", "settings": settings}, "page")


def test_calibration_delivery_is_independent_of_head_tracking():
    handler, sent, replies = request_handler()
    send(handler, {"calibrate_robot_position": True})
    assert sent == [{"type": "view_settings", "calibrate_robot_position": True}]
    assert replies == [{"request_id": "request-1", "ok": True}]


@pytest.mark.parametrize("options,error", [
    ({"fresh": False}, "feedback is offline"), ({"calibration": "offline"}, "Godot calibration is offline"),
    ({"calibration": ""}, "Godot calibration is offline"), ({"calibration": "solving"}, "already running"),
    ({"paused": True}, "Trial is paused"), ({"owner": "other"}, "no longer controls"),
])
def test_unready_or_superseded_requests_never_reach_godot(options, error):
    handler, sent, replies = request_handler(**options)
    send(handler, {"calibrate_robot_position": True})
    assert not sent
    assert replies[0]["ok"] is False and error in replies[0]["error"]


def test_cancel_is_deliverable_during_a_paused_trial_and_stale_feedback():
    handler, sent, replies = request_handler(paused=True, fresh=False, calibration="capturing")
    send(handler, {"cancel_robot_position_calibration": True})
    assert sent[0]["cancel_robot_position_calibration"] is True and replies[0]["ok"] is True


def test_invalid_module_settings_receive_a_rejection():
    handler, sent, replies = request_handler()
    send(handler, {"robot_visual_calibration_action": {"operation": "unknown", "request_id": "action-1"}})
    assert not sent and replies[0]["ok"] is False


def test_browser_waits_for_ack_and_never_replays_after_disconnect(tmp_path):
    output = run_node(tmp_path, r'''
const timers=new Map();let tid=0;
globalThis.window={so101ArmController:{},location:{protocol:'https:',host:'example.test'},addEventListener(){},dispatchEvent(){},
 setTimeout(fn){let id=++tid;timers.set(id,fn);return id;},clearTimeout(id){timers.delete(id);},setInterval(){return 0;},clearInterval(){}};
Object.defineProperty(globalThis,'navigator',{value:{},configurable:true});globalThis.document={getElementById(){return null;},addEventListener(){}};
class Socket extends EventTarget {
 static CONNECTING=0;static OPEN=1;
 constructor(){super();this.readyState=0;this.bufferedAmount=0;this.sent=[];}
 send(data){this.sent.push(JSON.parse(data));}
 close(){this.readyState=3;this.dispatchEvent(new Event('close'));}
}
globalThis.WebSocket=Socket;
globalThis.CloseEvent=class extends Event{};
const {So101ArmController}=await import('./controller.mjs');
const c=new So101ArmController();c.emitStatus=()=>{};
let socket=c.socket;socket.readyState=1;socket.dispatchEvent(new Event('open'));
let delivered=c.requestViewSettings({calibrate_robot_position:true});
let request=socket.sent.at(-1);
if(request.type!=='robot_view_settings')throw Error('settings must use robot control, not tracking');
let settled=false;delivered.then(()=>{settled=true});await Promise.resolve();
if(settled)throw Error('sending alone is not an acknowledgement');
socket.dispatchEvent(new MessageEvent('message',{data:JSON.stringify({type:'arm_status',view_settings_reply:{request_id:request.request_id,ok:true}})}));
if(await delivered!==true)throw Error('valid acknowledgement failed');
let rejected=c.requestViewSettings({refine_robot_joint_alignment:true});
request=socket.sent.at(-1);
socket.dispatchEvent(new MessageEvent('message',{data:JSON.stringify({type:'arm_status',view_settings_reply:{request_id:request.request_id,ok:false,error:'Robot offline'}})}));
try{await rejected;throw Error('must reject')}catch(e){if(!e.message.includes('Robot offline'))throw e;}
let lost=c.requestViewSettings({calibrate_robot_position:true});socket.close();
try{await lost;throw Error('must reject')}catch(e){if(!e.message.includes('not retried'))throw e;}
if(c.viewSettingsPending.size!==0)throw Error('disconnected actions must not be queued');
let retried=socket.sent.filter(p=>p.type==='robot_view_settings');
if(retried.length!==3)throw Error('an action was replayed');
console.log(JSON.stringify({passed:true}));
''', {}, production=True)
    assert output["passed"] is True
