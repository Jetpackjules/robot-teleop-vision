import base64,hashlib,hmac,json
import pytest
from tools import webrtc_ice


def test_unconfigured_gateway_keeps_existing_stun(monkeypatch):
    monkeypatch.delenv('ROBOT_TELEOP_ICE_CONFIG',raising=False)
    assert webrtc_ice.ice_servers()==webrtc_ice.DEFAULT_SERVERS


def test_turn_rest_auth_mints_credentials_without_exposing_secret(tmp_path,monkeypatch):
    p=tmp_path/'ice.json';p.write_text(json.dumps({'iceServers':[{'urls':'turn:relay.example:3478','sharedSecret':'test-secret'}]}))
    monkeypatch.setenv('ROBOT_TELEOP_ICE_CONFIG',str(p));monkeypatch.setattr(webrtc_ice.time,'time',lambda:1000)
    data=webrtc_ice.ice_servers();entry=data[0]
    assert entry['username']=='4600:teleop'
    assert entry['credential']==base64.b64encode(hmac.new(b'test-secret',b'4600:teleop',hashlib.sha1).digest()).decode()
    assert 'sharedSecret'not in json.dumps(data) and 'test-secret'not in json.dumps(data)


def test_private_static_credentials_are_normalized(tmp_path,monkeypatch):
    p=tmp_path/'ice.json';p.write_text(json.dumps({'iceServers':[{'urls':['turns:relay.example:443?transport=tcp'],'username':'user','credential':'test-password','private-note':'ignore'}]}));monkeypatch.setenv('ROBOT_TELEOP_ICE_CONFIG',str(p))
    assert webrtc_ice.ice_servers()==[{'urls':['turns:relay.example:443?transport=tcp'],'username':'user','credential':'test-password'}]


def test_cloudflare_key_stays_on_server_and_credentials_are_cached(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from contextlib import nullcontext
    p=tmp_path/'ice.json';p.write_text(json.dumps({'cloudflareTurn':{'keyId':'test-key','apiToken':'private-api-token'}}));monkeypatch.setenv('ROBOT_TELEOP_ICE_CONFIG',str(p))
    calls=[]
    def fetch(request,**_):
        calls.append(request)
        assert request.get_header('Authorization')=='Bearer private-api-token'
        assert request.full_url.endswith('/test-key/credentials/generate-ice-servers')
        return nullcontext(SimpleNamespace(read=lambda _:json.dumps({'iceServers':[{'urls':'turn:relay.example:3478','username':'short-lived','credential':'temporary'}]}).encode()))
    monkeypatch.setattr(webrtc_ice.urllib.request,'urlopen',fetch);webrtc_ice._cache.clear()
    first=webrtc_ice.ice_servers();second=webrtc_ice.ice_servers()
    assert first==second and len(calls)==1
    assert 'private-api-token'not in json.dumps(first)


@pytest.mark.parametrize('config',[[],{'iceServers':'wrong'},{'iceServers':[{'urls':'https://relay.example'}]},{'iceServers':[{'urls':'turn:x','sharedSecret':True}]}])
def test_bad_operator_configuration_rejected(config,tmp_path,monkeypatch):
    p=tmp_path/'ice.json';p.write_text(json.dumps(config));monkeypatch.setenv('ROBOT_TELEOP_ICE_CONFIG',str(p))
    with pytest.raises(ValueError):webrtc_ice.ice_servers()
