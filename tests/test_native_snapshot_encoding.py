"""Compare the production native-BGR encoder with the previous RGBA encoder."""
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


def test_native_snapshots_preserve_jpeg_bytes_and_ownership(tmp_path):
    compiler = shutil.which("g++")
    if sys.platform != "linux" or not compiler:
        pytest.skip("Production libjpeg parity requires the Linux compiler")
    root = Path(__file__).resolve().parents[1]
    source = (root / "native/realsense_shared_memory/src/realsense_shared_memory_point_cloud.cpp").read_text()
    encoder = source[source.index("struct RgbdJpegError {"):source.index("static PackedByteArray deflate_depth_predictive_fast(")]
    harness = r'''
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <csetjmp>
#include <cstdlib>
#include <iostream>
#include <vector>
#include "native_color_snapshot.h"
#include "rgbd_pixel_pack.h"
extern "C" {
#include <jpeglib.h>
}
struct Vector3 {float x=1,y=1,z=1;};
struct Image {enum Format {FORMAT_RGB8,FORMAT_RGBA8};};
class PackedByteArray {
 std::vector<uint8_t> bytes;
public:
 bool is_empty() const {return bytes.empty();}
 int64_t size() const {return bytes.size();}
 void resize(int64_t n) {bytes.resize(n);}
 uint8_t *ptrw() {return bytes.data();}
 const uint8_t *ptr() const {return bytes.data();}
 bool operator==(const PackedByteArray &other) const {return bytes==other.bytes;}
};
'''
    harness += encoder + r'''
int main() {
 int checks=0;
 for(auto size : {std::pair<int,int>{1,1},{2,3},{17,19},{848,480},{1920,1080}}) {
  const int w=size.first,h=size.second;
  for(int bpp : {3,4}) for(int padding : {0,17}) {
   auto storage=std::make_shared<std::vector<uint8_t>>((w*bpp+padding)*h);
   NativeColorSnapshot snapshot{storage,w,h,bpp,w*bpp+padding};assert(snapshot.valid());
   PackedByteArray rgba;rgba.resize(w*h*4);
   uint32_t random=0x1234567;
   for(int y=0;y<h;y++) for(int x=0;x<w;x++) {
    auto *bgr=storage->data()+y*snapshot.pitch+x*bpp;
    for(int c=0;c<bpp;c++){random=random*1664525+1013904223;bgr[c]=random>>24;}
    auto *p=rgba.ptrw()+(y*w+x)*4;
    p[0]=bgr[2];p[1]=bgr[1];p[2]=bgr[0];p[3]=255;
   }
   const auto before=*storage;
   for(int q : {1,78,95,100}) {
    const auto legacy=encode_resampled_rgb_jpeg_fast(rgba,w,h,4,0,0,w,h,w,h,q,false,Vector3{},Vector3{});
    const auto native=encode_native_bgr_jpeg_fast(snapshot,q);
    assert(!native.is_empty());assert(native==legacy);checks++;
   }
   assert(*storage==before);
   // A queued snapshot must retain bytes after capture advances or closes.
   std::weak_ptr<const std::vector<uint8_t>> lifetime=snapshot.pixels;
   storage.reset();assert(snapshot.valid());assert(*snapshot.pixels==before);
   snapshot=NativeColorSnapshot();assert(lifetime.expired());checks++;
  }
 }
 assert(!NativeColorSnapshot{}.valid());
 auto bytes=std::make_shared<std::vector<uint8_t>>(10);
 assert(!(NativeColorSnapshot{bytes,100,100,3,300}).valid());
 std::cout<<"PASS "<<checks<<" exact JPEG/ownership checks (including native Quality)\n";
}
'''
    cpp = tmp_path / "snapshot.cpp"
    cpp.write_text(harness)
    executable = tmp_path / "snapshot"
    subprocess.run([compiler, "-std=c++17", "-O2", "-I", str(root / "native/realsense_shared_memory/src"),
                    str(cpp), "-ljpeg", "-o", str(executable)], check=True, capture_output=True, text=True, timeout=60)
    result = subprocess.run([str(executable)], check=True, capture_output=True, text=True, timeout=60)
    assert "PASS 100 exact JPEG/ownership checks" in result.stdout
