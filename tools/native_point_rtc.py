"""Native point preview only; capture and recording remain independent."""
import asyncio
import json
import struct
import time
from aiortc.exceptions import InvalidStateError

from tools.native_sparse_stream import NativeSparseSession, discover_native_cameras

CHUNK_SIZE = 16384
HEADER = 20
MAX_PACKET = 512*1024


def packet_chunks(packet, sequence, camera):
    if not 0 < len(packet) <= MAX_PACKET or not 0 <= camera < 16:
        raise ValueError('invalid native point packet')
    size = CHUNK_SIZE-HEADER
    count = (len(packet)+size-1)//size
    for index in range(count):
        yield struct.pack('<4sIIHHI', b'NPC1', sequence & 0xffffffff,
                          len(packet), index, count, camera)+packet[index*size:(index+1)*size]


async def publish_native_points(channel, pc, server, client, fps, budget, robot_path):
    loop = asyncio.get_running_loop()
    cameras = await loop.run_in_executor(None, discover_native_cameras, server.stream_host, server.capture_port)
    if not cameras:
        channel.close()
        return
    slots = {camera['id']:index for index,(camera,_) in enumerate(cameras)}

    def settings():
        with server.stream_crop_lock:
            current = server.stream_crop_clients.get(client)
            value = dict(current[0]) if current else dict(point_preview_enabled=True, point_preview_budget=budget)
        return {**value, '_sparse_predictor':'delta_v1'}

    def robot():
        try:
            value = json.loads(robot_path.read_text())
            if value.get('project_root','').rstrip('/') == str(server.root.parent).rstrip('/') and 0 <= time.time()-float(value.get('updated',0)) < 2:
                return value.get('robot', {'available':False})
        except (OSError, ValueError, TypeError):pass
        return {'available':False}

    primary = min(cameras,key=lambda item:item[0]['priority'])[0]
    viewer = dict(transform=primary['_viewer'],fov=60,near=.05,far=100)
    session = NativeSparseSession(cameras,settings,robot,fps,viewer)
    sequence = 0
    session.start()
    try:
        while channel.readyState == 'open' and pc.connectionState not in ('failed','closed'):
            packet = await loop.run_in_executor(None,session.take,.1)
            if session.stop_event.is_set():break
            if packet is None or channel.bufferedAmount:
                continue  # Never queue another capture behind an unsent one.
            packet = server.robot_joint_history.attach(packet,wait_for_bracket_ms=0)
            metadata = json.loads(packet[9:9+struct.unpack_from('<I',packet,5)[0]])
            camera = slots[metadata['cameras'][0]['id']]
            sequence += 1
            for chunk in packet_chunks(packet,sequence,camera):channel.send(chunk)
            await asyncio.sleep(0)
    except (ConnectionError,OSError,ValueError,KeyError,InvalidStateError):
        pass
    finally:
        await loop.run_in_executor(None,session.close)
        if channel.readyState != 'closed':channel.close()
