"""Compile the production depth encoder and verify both lossless backends."""
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


@pytest.mark.parametrize("fast", [False, True])
def test_native_depth_compression_is_exact(tmp_path, fast):
    compiler = shutil.which("g++")
    if sys.platform != "linux" or not compiler:
        pytest.skip("Native zlib/libdeflate check requires the Linux compiler")
    if fast and (not shutil.which("pkg-config") or subprocess.run(["pkg-config", "--exists", "libdeflate"]).returncode):
        pytest.skip("Optional libdeflate development package is unavailable")
    root = Path(__file__).resolve().parents[1]
    source = (root / "native/realsense_shared_memory/src/realsense_shared_memory_point_cloud.cpp").read_text()
    encoder = source[source.index("static PackedByteArray deflate_depth_predictive_fast("):
                     source.index("class RgbdDepthCompressionPool {")]
    harness = r'''
#include <cstdint>
#include <vector>
#include <memory>
#include <cassert>
#include <zlib.h>
#include "rgbd_pixel_pack.h"
#ifdef REALSENSE_RGBD_LIBDEFLATE
#include <libdeflate.h>
#endif
class PackedByteArray {
 std::vector<uint8_t> data;
public:
 bool is_empty() const {return data.empty();}
 int64_t size() const {return data.size();}
 void resize(int64_t n) {data.resize(n);}
 void clear() {data.clear();}
 uint8_t *ptrw() {return data.data();}
 const uint8_t *ptr() const {return data.data();}
};
'''
    harness += encoder + r'''
int main() {
 for (int width : {1, 17, 848}) {
  const int count=width*480;
  PackedByteArray input;input.resize(count*2);
  std::vector<uint16_t> expected(count);
  for(int i=0;i<count;i++) {
   expected[i]=(i%17==0)?0:uint16_t(i*137);
   input.ptrw()[i*2]=expected[i]&255;input.ptrw()[i*2+1]=expected[i]>>8;
  }
  for(int repeat=0;repeat<3;repeat++) {
   auto compressed=deflate_depth_predictive_fast(input,width);
   assert(!compressed.is_empty());std::vector<uint8_t> predicted(count*2);
   uLongf size=predicted.size();
   assert(uncompress(predicted.data(),&size,compressed.ptr(),compressed.size())==Z_OK);
   assert(size==predicted.size());std::vector<uint16_t> actual(count);
   for(int i=0;i<count;i++) {
    uint16_t z=uint16_t(predicted[i])|(uint16_t(predicted[i+count])<<8);
    int delta=int(z>>1)^-int(z&1);
    actual[i]=uint16_t((i>=width?actual[i-width]:0)+delta);
   }
   assert(actual==expected);
  }
 }
 assert(deflate_depth_predictive_fast(PackedByteArray(),848).is_empty());
}
'''
    cpp = tmp_path / "depth.cpp"
    cpp.write_text(harness)
    executable = tmp_path / "depth"
    command = [compiler, "-std=c++17", "-O2", "-I",str(root / "native/realsense_shared_memory/src"),str(cpp), "-lz", "-o", str(executable)]
    if fast:
        command += ["-DREALSENSE_RGBD_LIBDEFLATE", "-ldeflate"]
    subprocess.run(command, check=True, capture_output=True, text=True, timeout=60)
    subprocess.run([str(executable)], check=True, timeout=20)
