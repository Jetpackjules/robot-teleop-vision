"""Operator-owned ICE settings; expose credentials, never REST auth secrets."""
import base64
import hashlib
import hmac
import json
import os
import time
import re
import threading
import urllib.request
from pathlib import Path

DEFAULT_SERVERS=[{'urls':'stun:stun.l.google.com:19302'}]
_cache={}
_lock=threading.Lock()


def cloudflare_servers(config):
    key=config.get('keyId','');token=config.get('apiToken','')
    if not isinstance(key,str)or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',key)or not isinstance(token,str)or not token:
        raise ValueError('Invalid Cloudflare TURN configuration')
    ttl=max(3600,min(86400,int(config.get('ttl',86400))))
    identity=hashlib.sha256(json.dumps([key,token,ttl]).encode()).hexdigest()
    with _lock:
        if _cache.get('identity')==identity and _cache['until']>time.time():return _cache['servers']
        request=urllib.request.Request('https://rtc.live.cloudflare.com/v1/turn/keys/'+key+'/credentials/generate-ice-servers',
            data=json.dumps({'ttl':ttl}).encode(),headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
        with urllib.request.urlopen(request,timeout=5)as response:body=response.read(65537)
        if len(body)>65536:raise ValueError('TURN response too large')
        servers=json.loads(body).get('iceServers')
        if not isinstance(servers,list):raise ValueError('Invalid TURN response')
        _cache.update(identity=identity,until=time.time()+min(ttl-300,3600),servers=servers)
        return servers


def ice_servers():
    path=os.environ.get('ROBOT_TELEOP_ICE_CONFIG','')
    if not path:return DEFAULT_SERVERS
    data=Path(path).read_bytes()
    if len(data)>65536:raise ValueError('ICE configuration too large')
    config=json.loads(data)
    if isinstance(config,dict)and config.get('cloudflareTurn'):
        config={'iceServers':cloudflare_servers(config['cloudflareTurn'])}
    if not isinstance(config,dict)or not isinstance(config.get('iceServers',[]),list)or len(config.get('iceServers',[]))>8:
        raise ValueError('Invalid ICE configuration')
    servers=[]
    for item in config.get('iceServers',[]):
        if not isinstance(item,dict):raise ValueError('Invalid ICE server')
        urls=item.get('urls');urls=[urls]if isinstance(urls,str)else urls
        if not isinstance(urls,list)or not urls or not all(isinstance(u,str)and u.startswith(('stun:','stuns:','turn:','turns:'))for u in urls):
            raise ValueError('Invalid ICE URLs')
        entry={'urls':urls}
        if item.get('sharedSecret'):
            if not isinstance(item['sharedSecret'],str):raise ValueError('Invalid TURN secret')
            username=str(int(time.time())+3600)+':teleop'
            entry.update(username=username,credential=base64.b64encode(hmac.new(item['sharedSecret'].encode(),username.encode(),hashlib.sha1).digest()).decode())
        else:
            for key in ['username','credential']:
                if key in item:entry[key]=str(item[key])
        servers.append(entry)
    return servers or DEFAULT_SERVERS
