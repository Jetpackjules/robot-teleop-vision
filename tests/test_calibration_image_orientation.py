"""Image rotation must never rotate depth lookup or the calibrated camera axes."""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_inverse_image_rotation_preserves_original_pixels(tmp_path):
    compiler = shutil.which("g++") or shutil.which("clang++")
    if compiler is None:
        pytest.skip("A C++ compiler is required")
    source = tmp_path / "orientation.cpp"
    source.write_text(r'''
#include "calibration_image_orientation.h"
#include <cassert>
#include <cmath>
void check(int rotation,double x,double y,double ox,double oy,int w=4,int h=3) {
 auto p=calibration_original_pixel(x,y,w,h,rotation);
 assert(std::abs(p[0]-ox)<1e-9 && std::abs(p[1]-oy)<1e-9);
}
int main() {
 check(0,0,0,0,0);check(0,3,2,3,2);
 check(90,0,0,3,0);check(90,2,3,0,2);check(90,1,2,1,1);
 check(180,0,0,3,2);check(180,3,2,0,0);
 check(270,0,0,0,2);check(270,2,3,3,0);check(270,1,2,2,1);
 check(270,200.25,820.75,820.75,518.75,1280,720);
 // Dense round trip through a separately defined forward image rotation.
 for(int r:{0,90,180,270}) for(int y=0;y<720;y+=17) for(int x=0;x<1280;x+=13) {
   double u=x+.25,v=y+.5;
   double a=u,b=v;
   if(r==90){a=v;b=1279-u;}else if(r==180){a=1279-u;b=719-v;}
   else if(r==270){a=719-v;b=u;}
   check(r,a,b,u,v,1280,720);
 }
}
''')
    executable = tmp_path / "orientation.exe"
    subprocess.run([compiler, "-std=c++17", "-I", str(ROOT / "native/realsense_shared_memory/src"),
                    str(source), "-o", str(executable)], check=True, capture_output=True, timeout=30)
    subprocess.run([str(executable)], check=True, timeout=10)
