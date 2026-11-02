import {countWorkspacePoints} from './workspace_coverage.js';
const cameras=new Map();
self.onmessage=({data})=>{
  try {
    const started=performance.now();
    for(const camera of data.updates)cameras.set(camera.key,camera);
    for(const key of cameras.keys())if(!data.keys.includes(key))cameras.delete(key);
    const result=countWorkspacePoints(data.keys.map(key=>cameras.get(key)),data);
    self.postMessage({id:data.id,...result,workerMs:performance.now()-started});
  } catch(error){self.postMessage({id:data.id,error:String(error?.message||error)});}
};
