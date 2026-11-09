import asyncio,json,struct,threading,zlib
from pathlib import Path
from types import SimpleNamespace
from aiortc import RTCPeerConnection,RTCConfiguration
from tools.native_point_rtc import packet_chunks,publish_native_points


def test_chunks_are_bounded_and_lossless():
 for size in [1,180000,360000]:
  packet=bytes(i%251 for i in range(size));chunks=list(packet_chunks(packet,42,2))
  assert all(len(c)<=16384 for c in chunks)
  assert b"".join(c[20:]for c in chunks)==packet
  assert all(struct.unpack("<4sIIHHI",c[:20])[:3]==(b"NPC1",42,size) for c in chunks)


def test_live_native_camera_points_over_unordered_loopback():
 async def run():
  root=Path(__file__).resolve().parents[1]
  # Integration is explicit; ordinary unit runs never require live cameras.
  import os
  if os.environ.get("TELEOP_LIVE_NATIVE_RTC")!="1":
   import pytest;pytest.skip("explicit live capture test")
  root=Path(os.environ["TELEOP_TEST_ROOT"])
  results=[]
  for budget in [20000,40000]:
   tx=RTCPeerConnection(RTCConfiguration(iceServers=[]));rx=RTCPeerConnection(RTCConfiguration(iceServers=[]))
   channel=tx.createDataChannel("rgbd-stream",ordered=False,maxRetransmits=0)
   partial={};received={};expected=set();ready=asyncio.Event()
   @rx.on("datachannel")
   def on_channel(c):
    @c.on("message")
    def receive(data):
     magic,sequence,size,index,count,slot=struct.unpack("<4sIIHHI",data[:20]);assert magic==b"NPC1"
     item=partial.setdefault(sequence,{})
     item[index]=data[20:]
     if len(item)!=count:return
     packet=b"".join(item[i]for i in range(count));assert len(packet)==size
     assert packet in expected,"SCTP must preserve the exact source packet"
     end=9+struct.unpack_from("<I",packet,5)[0];meta=json.loads(packet[9:end])["cameras"][0]
     assert meta["preview_native"] and meta["preview_budget"]==budget and meta["preview_count"]==budget
     assert meta["preview_predictor"]=="delta_v1" and len(zlib.decompress(packet[end:]))==budget*9
     received[slot]=True
     if len(received)==3:ready.set()
   await tx.setLocalDescription(await tx.createOffer());await rx.setRemoteDescription(tx.localDescription)
   await rx.setLocalDescription(await rx.createAnswer());await tx.setRemoteDescription(rx.localDescription)
   for _ in range(200):
    if channel.readyState=="open":break
    await asyncio.sleep(.025)
   assert channel.readyState=="open"
   def attach(packet,**_):expected.add(packet);return packet
   server=SimpleNamespace(stream_host="127.0.0.1",capture_port=8781,root=root/"web",stream_crop_lock=threading.Lock(),stream_crop_clients={},robot_joint_history=SimpleNamespace(attach=attach))
   task=asyncio.create_task(publish_native_points(channel,tx,server,"test",15,budget,root/".teleop/nonexistent-robot-file.json"))
   try:
    await asyncio.wait_for(ready.wait(),15)
    results.append({"budget":budget,"cameras":len(received),"exact_packets":len(expected)})
   finally:
    task.cancel()
    try:await task
    except asyncio.CancelledError:pass
    await tx.close();await rx.close()
  print(json.dumps(results))
 asyncio.run(run())
