"""Generate public synthetic packets and actual SO-101 FK, without devices."""
from pathlib import Path
import argparse
import base64
import json
import runpy
import struct
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robot_modules.so101.tools.so101_kinematics import rendered_link_transforms
from tools.workspace_stream_crop import WorkspaceStreamCropper


def generate():
    fixture=runpy.run_path(str(ROOT/'tests/test_workspace_stream_crop.py'))
    source=fixture['packet'](True,True,2)
    settings=fixture['SETTINGS']
    cropped=WorkspaceStreamCropper().crop(source,settings)
    statistics=fixture['split'](cropped)[0]['stream_crop']
    base=np.eye(4)
    base[:3,:3]=[[0,-1,0],[0,0,1],[-1,0,0]]
    base[:3,3]=[0,0,-1]
    poses=[]
    for i in range(16):
        phase=i/15*2*np.pi
        normalized=[-40.4296875+25*np.sin(phase),65+15*np.sin(phase),-35+20*np.cos(phase),70+15*np.sin(phase+.5),30*np.sin(phase),20+20*(1+np.cos(phase))]
        def dictionary(matrix):return {'basis_x':matrix[:3,0].tolist(),'basis_y':matrix[:3,1].tolist(),
            'basis_z':matrix[:3,2].tolist(),'origin':matrix[:3,3].tolist()}
        links={name:dictionary(base@matrix) for name,matrix in rendered_link_transforms(normalized).items()}
        links['base_link']=dictionary(base)
        poses.append({'joints':normalized,'links':links})
    return {'settings':settings,'source':base64.b64encode(source).decode(),'cropped':base64.b64encode(cropped).decode(),
            'stream':statistics,'poses':poses,'synthetic':True}


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    data=generate();args.output.write_text(json.dumps(data,separators=(',',':')))
    print(json.dumps({'fixture':str(args.output),'source_bytes':len(base64.b64decode(data['source'])),
                      'cropped_bytes':len(base64.b64decode(data['cropped'])),'joint_poses':len(data['poses'])}))
