"""Bake link-local signed-distance fields from the same CAD assets as the overlay."""
from pathlib import Path
import hashlib,json
import numpy as np
import trimesh
import open3d as o3d
ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'robot_modules/so101/assets'
N=96
fields=[];links=[]
for name in ['gripper_link','moving_jaw_so101_v1_link']:
    path=ASSETS/(name+'.glb')
    scene=trimesh.load(path,force='scene',process=False)
    mesh=scene.to_geometry() if hasattr(scene,'to_geometry') else scene.dump(concatenate=True)
    # GLB primitives split vertices at material/normal seams. Welding restores
    # the closed CAD topology without changing the rendered surface.
    mesh.merge_vertices()
    mesh.remove_unreferenced_vertices()
    if not mesh.is_watertight or not mesh.is_winding_consistent:
        raise ValueError(f'{name} must be a closed, consistently wound mesh')
    low,high=mesh.bounds+np.array([[-.055]*3,[.055]*3])
    grid=np.stack(np.meshgrid(*[np.linspace(low[i],high[i],N) for i in range(3)],indexing='ij'),axis=-1)
    ray=o3d.t.geometry.RaycastingScene()
    ray.add_triangles(o3d.core.Tensor(np.asarray(mesh.vertices,np.float32)),o3d.core.Tensor(np.asarray(mesh.faces,np.uint32)))
    distances=ray.compute_signed_distance(o3d.core.Tensor(grid.reshape(-1,3).astype(np.float32)),nsamples=3).numpy().reshape(N,N,N)
    fields.append(distances.transpose(2,1,0))
    links.append({'name':name,'min':low.tolist(),'max':high.tolist(),'model_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'watertight':bool(mesh.is_watertight)})
    print(name,'bounds',mesh.bounds.tolist(),'watertight',mesh.is_watertight)
np.stack(fields,axis=-1).astype('<f2').tofile(ASSETS/'claw-mask-sdf.bin')
(ASSETS/'claw-mask-sdf.json').write_text(json.dumps({'size':N,'format':'rg16f','links':links},indent=2))
print('CLAW SDF BAKED')
