const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
(async()=>{
 const source=fs.readFileSync(path.join(__dirname,'../web/point_detail_settings.js'),'utf8');
 const {resolvePointDetail}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
 assert.equal(resolvePointDetail({point_preview_detail:40000},null),'40000','a new origin loads the host choice');
 for(const n of [0,-2,-1,20000,40000]){
  assert.equal(resolvePointDetail({point_preview_detail:n},null),String(n));
  assert.equal(resolvePointDetail({point_preview_detail:40000},String(n)),String(n),'explicit last browser choice is retained');
 }
 assert.equal(resolvePointDetail({point_preview_detail:40000},'0',false),'40000','Reload defaults explicitly restores the host choice');
 assert.equal(resolvePointDetail({},'broken'),'-1');
 console.log('PASS host point detail across origins, all five choices, browser preference and explicit default reload');
})().catch(e=>{console.error(e);process.exitCode=1;});
