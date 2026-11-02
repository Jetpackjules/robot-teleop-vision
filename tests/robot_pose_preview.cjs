const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
 const {RobotPosePreview}=await import('data:text/javascript;base64,'+Buffer.from(fs.readFileSync(path.join(__dirname,'../web/robot_pose_preview.js'),'utf8')).toString('base64'));
 const dict=(x,y,z,angle=0,scale=1)=>({basis_x:[Math.cos(angle)*scale,Math.sin(angle)*scale,0],basis_y:[-Math.sin(angle)*scale,Math.cos(angle)*scale,0],basis_z:[0,0,scale],origin:[x,y,z]});
 const packet=(stamp,angle=0,x=0)=>({available:true,pose_unix_ms:stamp,parents:{arm:'base',tip:'arm'},links:{base:dict(x,0,0),arm:dict(x,0,0,angle),tip:dict(x+Math.cos(angle),Math.sin(angle),0,angle)}});
 const p=new RobotPosePreview(70);p.accept(packet(100),0);p.setManual(true);p.accept(packet(200,Math.PI/2),100);
 const angles=new Set();
 for(let t=100;t<=170;t+=1000/60){
  const pose=p.sample(t),a=pose.arm.origin,b=pose.tip.origin;
  assert(Math.abs(Math.hypot(...a.map((v,i)=>v-b[i]))-1)<1e-9,'interpolation keeps the arm connected and rigid');
  angles.add(Math.atan2(pose.arm.basis_x[1],pose.arm.basis_x[0]).toFixed(6));
 }
 assert(angles.size>=4,'sparse updates produce intermediate display frames');
 assert.deepEqual(p.sample(171),packet(200,Math.PI/2).links,'finish at exact authoritative transforms');
 assert.equal(p.accept(packet(150,-Math.PI),172),false,'late RGB-D poses cannot rewind control-channel updates');
 assert.equal(p.accept({...packet(0),pose_unix_ms:undefined},172),false,'legacy packets cannot rewind a manual edit');
 p.setBaseTarget({origin:[.1,.2,-.3],yaw:30},'base');
 const moved=p.sample(173);assert.deepEqual(moved.base.origin,[.1,.2,-.3],'base preview updates immediately without any server frame');
 assert(Math.abs(Math.hypot(...moved.tip.origin.map((v,i)=>v-moved.arm.origin[i]))-1)<1e-9);
 p.accept(packet(300,Math.PI/2,.04),180);assert(p.sample(185).base.origin.every((v,i)=>Math.abs(v-[.1,.2,-.3][i])<1e-9),'server base preview is not double-applied');
 p.setBaseTarget(null);p.setManual(false);p.accept(packet(400,0,.5),200);
 assert.deepEqual(p.sample(200),packet(400,0,.5).links,'ordinary measured motion gets no interpolation delay');
 let fractional=0;
 p.reset();p.accept(packet(1000),0);p.setManual(true);
 for(let tick=1;tick<=60;tick++){
  const now=tick*1000/60;if(tick%6===0)p.accept(packet(1000+now,tick/60),now);
  const pose=p.sample(now),angle=Math.atan2(pose.arm.basis_x[1],pose.arm.basis_x[0]);
  if(tick%6!==0&&angle>0)fractional++;
 }
 assert(fractional>=35,'10 Hz source poses animate through most 60 Hz display frames');
 const scale=new RobotPosePreview();scale.accept({links:{root:dict(1,2,3,.7,2)}},0);scale.setManual(true);scale.accept({links:{root:dict(2,4,6,-.5,2)}},10);
 const mid=scale.sample(40).root;assert(Math.abs(Math.hypot(...mid.basis_x)-2)<1e-9,'interpolation preserves model scale');
 console.log('Robot pose preview checks passed: rigid hierarchy, 60 Hz sampling of 10 Hz poses, timestamps, immediate base preview, no double application, and normal motion timing.');
})().catch(e=>{console.error(e);process.exitCode=1;});
