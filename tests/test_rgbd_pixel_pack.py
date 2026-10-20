"""Bit-for-bit SIMD equivalence, including padded rows and depth edge cases."""
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.mark.parametrize("sanitize", [False, True])
def test_pixel_pack_preserves_every_value(tmp_path, sanitize):
    compiler = shutil.which("g++") or shutil.which("clang++")
    if compiler is None:
        pytest.skip("C++ compiler required; exercised with the lab native build")
    root = Path(__file__).resolve().parents[1]
    source = tmp_path / "pixels.cpp"
    source.write_text(r'''
#include "rgbd_pixel_pack.h"
#include <cassert>
#include <memory>
#include <vector>
#include <limits>
#include <random>
int main() {
 std::mt19937 random(19);
 for(int count=1;count<1031;count++) for(bool mask:{false,true}) {
  // Allocate exactly the row length: ASan catches a vector read past its end.
  std::unique_ptr<uint8_t[]> source(new uint8_t[count*3]);
  std::vector<uint8_t> actual(count*4+2,71),expected(actual);
  for(int i=0;i<count*3;i++) source[i]=(i%11<3)?0:random();
  for(int i=0;i<count;i++) {
   expected[1+i*4]=source[i*3+2];expected[1+i*4+1]=source[i*3+1];expected[1+i*4+2]=source[i*3];
   expected[1+i*4+3]=!mask || (source[i*3]|source[i*3+1]|source[i*3+2]) ? 255:0;
  }
  rgbd_pixels::bgr_to_rgba(source.get(),actual.data()+1,count,mask);
  assert(actual==expected);
 }
 std::vector<uint16_t> raw(65539);
 for(size_t i=0;i<raw.size();i++) raw[i]=uint16_t(i);
 std::vector<float> actual(raw.size()),expected(raw.size());
 for(float units:{.0001f,.001f,.002f}) for(float minimum:{0.f,.2f}) {
  int valid=0;
  for(size_t i=0;i<raw.size();i++) {expected[i]=float(raw[i])*units;valid+=(expected[i]>=minimum && expected[i]<=1.5f);}
  assert(rgbd_pixels::z16_to_rf(raw.data(),actual.data(),raw.size(),units,minimum,1.5f)==valid);
  assert(std::memcmp(actual.data(),expected.data(),actual.size()*4)==0);
 }
 for(int width:{1,7,17,848}) for(int rows:{1,3,77}) {
  const int count=width*rows-rows%2;
  if(count==0)continue;
  std::unique_ptr<uint8_t[]> source(new uint8_t[count*2]);
  std::vector<uint8_t> predicted(count*2+2,73),golden(predicted);
  for(int i=0;i<count*2;i++)source[i]=random();
  for(int i=0;i<count;i++) {
   const uint16_t current=uint16_t(source[i*2])|(uint16_t(source[i*2+1])<<8);
   const uint16_t previous=i<width?0:uint16_t(source[(i-width)*2])|(uint16_t(source[(i-width)*2+1])<<8);
   const int16_t delta=static_cast<int16_t>(uint16_t(current-previous));
   const uint16_t zigzag=delta>=0?uint16_t(uint32_t(delta)*2U):uint16_t(uint32_t(-int32_t(delta))*2U-1U);
   golden[1+i]=uint8_t(zigzag);golden[1+count+i]=uint8_t(zigzag>>8);
  }
  rgbd_pixels::predict_vertical(source.get(),predicted.data()+1,count,width);
  assert(predicted==golden);
 }
 std::vector<float> depths(1000003);
 for(float &value:depths) {uint32_t bits=random();std::memcpy(&value,&bits,4);}
 for(int i=0;i<65536;i++) {
  depths[i]=float(i)*.001f;
  depths[65536+i]=std::nextafter(float(i)*.001f+.0005f,0.f);
  depths[131072+i]=std::nextafter(float(i)*.001f+.0005f,std::numeric_limits<float>::infinity());
 }
 for(float maximum:{1.5f,4.5f,100.f,std::numeric_limits<float>::infinity()}) {
  std::vector<uint8_t> packed(depths.size()*2+2,93),golden(packed);
  for(size_t i=0;i<depths.size();i++) {
   float value=depths[i];uint16_t mm=std::isfinite(value)&&value>0 && value<=maximum
    ?uint16_t(std::min(65535.,std::round(double(value)*1000.))):0;
   golden[1+i*2]=uint8_t(mm);golden[2+i*2]=uint8_t(mm>>8);
  }
  rgbd_pixels::rf_to_mm(depths.data(),packed.data()+1,depths.size(),maximum);
  assert(packed==golden);
 }
}
''')
    executable = tmp_path / "pixels"
    command = [compiler, "-std=c++17", "-O2", "-I", str(root / "native/realsense_shared_memory/src"),
               str(source), "-o", str(executable)]
    if sanitize:
        command += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    subprocess.run(command, check=True, capture_output=True, text=True, timeout=45)
    subprocess.run([str(executable)], check=True, capture_output=True, text=True, timeout=30)
