"""Exercise the actual POSIX native snapshot protocol without camera hardware."""
import os
import re
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import time
import uuid

import pytest

ROOT=Path(__file__).resolve().parents[1]

@pytest.mark.skipif(sys.platform!="linux",reason="Linux foreign-process lease check")
def test_capture_control_accepts_foreign_process_and_expires_lease(tmp_path):
    godot=os.environ.get("ROBOT_TELEOP_TEST_GODOT","")
    if not godot: pytest.skip("Godot is needed")
    env={**os.environ,"ROBOT_TELEOP_CAPTURE_ONLY":"1",
        "ROBOT_TELEOP_CAPTURE_SHM_PREFIX":"teleop_test_"+uuid.uuid4().hex,
        "XDG_DATA_HOME":str(tmp_path),"ROBOT_TELEOP_MODULE":"disabled"}
    result=subprocess.run([godot,"--headless","--path",str(ROOT),"--script",
        "res://tests/godot/capture_control_probe.gd","--",str(os.getpid())],
        env=env,capture_output=True,text=True,timeout=20)
    assert result.returncode==0,result.stdout+result.stderr
    assert "CAPTURE_CONTROL foreign_process_and_lease_expiry=passed" in result.stdout,result.stdout+result.stderr

@pytest.mark.skipif(sys.platform!="linux",reason="POSIX camera capture integration")
def test_native_shared_capture_preserves_frames_and_rejects_torn_writes(tmp_path):
    godot=os.environ.get("ROBOT_TELEOP_TEST_GODOT","")
    library=Path(os.environ.get("ROBOT_TELEOP_TEST_NATIVE_LIBRARY",str(ROOT/"native/realsense_shared_memory/bin/librealsense_shared_memory_color_authority.so")))
    if not godot or not library.is_file():
        pytest.skip("A built native extension and Godot are needed")
    from multiprocessing import shared_memory
    name="teleop_test_"+uuid.uuid4().hex
    source=shared_memory.SharedMemory(name=name,create=True,size=128+16*16*8)
    try:
        header=source.buf
        header[:8]=b"RSPG01\x00\x00"
        struct.pack_into("<QQIIIIffffIIdd",header,8,2,1,16,16,1,1,20,21,8,8,1024,1024,60,time.time()*1000)
        for y in range(16):
            for x in range(16):
                index=y*16+x
                struct.pack_into("<f",header,128+index*4,1+index*.001)
                struct.pack_into("BBBB",header,128+1024+index*4,x,y,x+y,255)
        struct.pack_into("<f",header,128+(8*16+8)*4,0.0)
        del header
        (tmp_path/"project.godot").write_text('config_version=5\n[application]\nconfig/name="Shared capture test"\n')
        (tmp_path/"native.gdextension").write_text('[configuration]\nentry_symbol="realsense_shared_memory_library_init"\ncompatibility_minimum="4.6"\n[libraries]\nlinux.x86_64="'+library.as_posix()+'"\n')
        (tmp_path/".godot").mkdir()
        (tmp_path/".godot/extension_list.cfg").write_text("res://native.gdextension\n")
        shutil.copy2(ROOT/"tests/godot/shared_capture_probe.gd",tmp_path/"probe.gd")
        result=subprocess.run([godot,"--headless","--path",str(tmp_path),"--script","res://probe.gd","--",name,name+"_out"],
            capture_output=True,text=True,timeout=25)
        assert result.returncode==0,result.stdout+result.stderr
        assert re.search(r"SHARED_CAPTURE checks=\d+ failures=0",result.stdout)
        assert not Path('/dev/shm',name+'_out').exists(),"publisher removes its mapping on normal shutdown"
    finally:
        source.close()
        source.unlink()
