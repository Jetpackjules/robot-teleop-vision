import hashlib
import json
import time
from pathlib import Path
import pytest
from robot_teleop.camera_alignment import CameraAlignment

@pytest.fixture
def alignment(tmp_path):
    root = tmp_path / 'project'
    user = tmp_path / 'userdata'
    root.mkdir(); user.mkdir()
    (user / 'camera_alignment_registry.json').write_text('{"camera": 123}')
    (user / 'so101_robot_registration.json').write_text('{"arm": 456}')
    profile = root / 'robot_modules/so101/local/so101_arm_pair.json'
    profile.parent.mkdir(parents=True); profile.write_text('{"motor": 789}')
    service = CameraAlignment(root, user)
    (user / 'camera_alignment_web_status.json').write_text(json.dumps({'project_root': str(root), 'updated': time.time(), 'busy': False, 'message': 'Ready'}))
    return service

def test_start_backs_up_before_single_command_and_never_changes_originals(alignment):
    originals = {p: p.read_bytes() for p in alignment.userdata.glob('*.json')}
    sent = []
    def send(payload):
        assert (Path(alignment.last_backup) / 'manifest.json').exists()
        sent.append(json.loads(payload))
    result = alignment.start('markerless', 'idle', {'state': 'hold', 'follower_connected': True}, send)
    assert result['busy']
    assert len(sent) == 1 and sent[0]['type'] == 'camera_alignment'
    assert sent[0]['mode'] == 'markerless'
    manifest = json.loads((Path(result['backup']) / 'manifest.json').read_text())
    assert any('so101_arm_pair.json' in p for p in manifest)
    for source, record in manifest.items():
        assert hashlib.sha256((Path(result['backup']) / record['file']).read_bytes()).hexdigest() == record['sha256']
    assert all(p.read_bytes() == data for p, data in originals.items())
    with pytest.raises(ValueError, match='already running'):
        alignment.start('refine', 'idle', {}, send)
    assert len(sent) == 1

@pytest.mark.parametrize('mode,state,robot', [('bad','idle',{}), ('refine','recording',{}), ('refine','paused',{}), ('refine','idle',{'armed':True}), ('refine','idle',{'rest_return_active':True}), ('refine','idle',{'follower_connected':True,'state':'active'})])
def test_rejections_do_not_send_or_create_backup(alignment, mode, state, robot):
    with pytest.raises(ValueError):alignment.start(mode,state,robot,lambda _:pytest.fail('must not send'))
    assert alignment.last_backup is None

def test_stale_or_foreign_runtime_cannot_start(alignment):
    path = alignment.userdata / 'camera_alignment_web_status.json'
    for value in [{'project_root': str(alignment.root), 'updated': time.time()-10}, {'project_root':'/other/project','updated':time.time()}]:
        path.write_text(json.dumps(value))
        with pytest.raises(ValueError, match='offline'):alignment.start('refine','idle',{},lambda _:pytest.fail('must not send'))

def test_backup_failure_blocks_start(alignment, monkeypatch):
    def fail():raise OSError('disk full')
    monkeypatch.setattr(alignment,'backup',fail)
    with pytest.raises(OSError):alignment.start('refine','idle',{},lambda _:pytest.fail('must not send'))

def test_acknowledgement_and_timeout_never_replay_commands(alignment):
    sent=[]; alignment.start('refine','idle',{},sent.append)
    packet=json.loads(sent[0])
    path=alignment.userdata / 'camera_alignment_web_status.json'
    path.write_text(json.dumps({'project_root':str(alignment.root),'updated':time.time(),'request_id':packet['request_id'],'busy':False,'message':'Previous alignment retained'}))
    assert alignment.status()['message']=='Previous alignment retained'
    assert alignment.pending is None
    alignment.start('refine','idle',{},sent.append)
    alignment.pending['sent']-=7
    assert 'did not acknowledge' in alignment.status()['message']
    assert len(sent)==2


def test_aruco_mm_is_forwarded_exactly_and_invalid_sizes_do_not_start(alignment):
    for size in [None, True, '150', 0, 19, 1001, float('nan')]:
        with pytest.raises(ValueError):alignment.start('aruco','idle',{},lambda _:pytest.fail('must not send'),size)
    sent=[]
    alignment.start('aruco','idle',{},sent.append,123.4,7)
    packet=json.loads(sent[0])
    assert packet['mode']=='aruco' and packet['marker_size_mm']==123.4 and packet['marker_id']==7


def test_charuco_specs_are_validated_and_forwarded(alignment):
    board = {'squares_x':10,'squares_y':16,'square_mm':30,'marker_mm':22,'dictionary':1}
    for changes in [{'squares_x':True},{'squares_y':2},{'square_mm':float('nan')},{'marker_mm':30},{'dictionary':0},{'dictionary':'1'}]:
        with pytest.raises(ValueError):alignment.start('charuco','idle',{},lambda _:pytest.fail('must not send'),board={**board,**changes})
    sent=[];alignment.start('charuco','idle',{},sent.append,board=board)
    assert json.loads(sent[0])['charuco_board']==board


def test_board_tabletop_flag_is_explicit_and_validated(alignment):
    sent=[]
    with pytest.raises(ValueError):alignment.start('aruco','idle',{},sent.append,127,49,board_is_tabletop='yes')
    alignment.start('aruco','idle',{},sent.append,127,49,board_is_tabletop=False)
    assert json.loads(sent[0])['board_is_tabletop'] is False


def test_board_level_updates_shared_table_once_and_preserves_other_settings(tmp_path):
    import threading
    from tools.lan_remote_view_server import LanRemoteServer
    server=LanRemoteServer.__new__(LanRemoteServer)
    server.site_settings_lock=threading.Lock()
    server.site_settings_path=tmp_path/'settings.json'
    server.save_site_settings({'modeled_workspace_center':[.2,.7,.3],'robot_overlay_claw_opacity':.5})
    status={'online':True,'world_level':{'source':'calibration_board_charuco','token':123,'table_y':0}}
    server.sync_board_table_level(status)
    saved=server.load_site_settings()
    assert saved['modeled_workspace_center']==[.2,0,.3]
    assert saved['robot_overlay_claw_opacity']==.5
    server.save_site_settings({'modeled_workspace_center':[.2,.1,.3],'robot_overlay_claw_opacity':.5})
    server.sync_board_table_level(status)
    assert server.load_site_settings()['modeled_workspace_center']==[.2,.1,.3]
    server.sync_board_table_level({'online':False,'world_level':{'source':'calibration_board_aruco','token':124}})
    assert server.load_site_settings()['modeled_workspace_center']==[.2,.1,.3]
    status['world_level']['token']=125
    server.sync_board_table_level(status)
    assert server.load_site_settings()['modeled_workspace_center']==[.2,0,.3]
