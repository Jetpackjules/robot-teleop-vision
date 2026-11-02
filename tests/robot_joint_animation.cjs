const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
  const source=fs.readFileSync(path.join(__dirname,'../web/robot_joint_animation.js'),'utf8');
  const {measuredJointTransforms,MeasuredJointAnimation}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
  const cases=JSON.parse(fs.readFileSync(process.argv[2]||path.join(__dirname,'fixtures/so101_measured_fk.json'),'utf8'));
  let largest=0,checks=0;
  for(const item of cases){
    const model=item.robot.kinematics,actual=measuredJointTransforms(model,item.normalized);
    assert.deepEqual(Object.keys(actual).sort(),Object.keys(item.robot.links).sort());
    for(const [name,transform] of Object.entries(item.robot.links)) for(const key of ['basis_x','basis_y','basis_z','origin']) for(let i=0;i<3;i++){
      const error=Math.abs(actual[name][key][i]-transform[key][i]);largest=Math.max(largest,error);checks++;
      assert(error<2e-6,`${name}.${key}[${i}] differs from Godot by ${error}`);
    }
    const modelBefore=JSON.stringify(model),valuesBefore=JSON.stringify(item.normalized);
    measuredJointTransforms(model,item.normalized);
    assert.equal(JSON.stringify(model),modelBefore);assert.equal(JSON.stringify(item.normalized),valuesBefore);
  }
  const item=cases[0],robot={kinematics:item.robot.kinematics,measured_joint_history:[
    {unix_ms:1000,normalized:[0,0,0,0,170,0]}, {unix_ms:1100,normalized:[10,20,30,40,-170,100]}]};
  const animation=new MeasuredJointAnimation();assert(animation.accept(robot));
  animation.sample(1050);assert.equal(animation.status.mode,'interpolated');assert.equal(animation.status.errorMs,0);
  const midpoint=measuredJointTransforms(robot.kinematics,[5,10,15,20,180,50]);
  assert.deepEqual(animation.sample(1050),midpoint,'interpolate measured joints at the camera instant, including wrap-around and jaw curve');
  assert.deepEqual(animation.sample(2000),measuredJointTransforms(robot.kinematics,robot.measured_joint_history[1].normalized),'never extrapolate beyond feedback');
  assert.equal(animation.status.synchronized,false);assert.equal(animation.status.mode,'held_latest');
  animation.accept({...robot,measured_joint_history:[{unix_ms:900,normalized:[1,1,1,1,1,1]}]});
  assert.equal(animation.samples.at(-1).unix_ms,1100,'late packet cannot erase recent history');
  animation.model={...animation.model,manual_active:true};assert.equal(animation.sample(1050),null,'manual alignment keeps its existing preview path');
  animation.reset();assert.equal(animation.sample(1050),null);
  console.log(JSON.stringify({checks,godotCases:cases.length,maxTransformError:largest,timingAndFallbackChecks:'passed'}));
})().catch(error=>{console.error(error);process.exit(1);});
