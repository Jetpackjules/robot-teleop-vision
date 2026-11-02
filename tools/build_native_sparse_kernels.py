"""Build optional point-buffer kernels without starting cameras or services."""
from pathlib import Path
import os
import shlex
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tools.native_sparse_kernels import SOURCE,library_path

if __name__=='__main__':
    output=library_path()
    output.parent.mkdir(parents=True,exist_ok=True)
    temporary=output.with_suffix('.building.so')
    subprocess.run([*shlex.split(os.environ.get('CXX','g++')),'-O3','-std=c++17',
        '-shared','-fPIC','-ffp-contract=off',str(SOURCE),'-o',str(temporary)],check=True)
    temporary.replace(output)
    print(output)
