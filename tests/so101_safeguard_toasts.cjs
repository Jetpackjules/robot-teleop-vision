const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
(async()=>{
 const source=fs.readFileSync(path.join(__dirname,'../robot_modules/so101/web/safeguard_toasts.js'),'utf8');
 const {ArmNoticeDetector,ArmToastStack}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
 const base={state:'armed',follower_connected:true,message:'Keyboard control is armed.',ik_guard_trip_count:5,
  following_error_trip_count:3,gripper_contact_trip_count:2,watchdog_hold_count:4};
 const wrap=status=>({serverConnected:true,status});
 const d=new ArmNoticeDetector();assert.deepEqual(d.update(wrap(base)),[],'no replay of historical counters');
 assert.deepEqual(d.update(wrap({...base})),[]);
 const floor={...base,ik_guard_trip_count:6,ik_clearance_limited:true};
 let events=d.update(wrap(floor));assert.equal(events.length,1);assert.equal(events[0].title,'Claw/base clearance limit');assert.equal(events[0].type,'safeguard');
 assert.deepEqual(d.update(wrap(floor)),[],'no spam while limit stays active');
 const following={...floor,following_error_trip_count:5};events=d.update(wrap(following));assert.equal(events[0].id,'following');assert.equal(events[0].count,2);
 const watchdog={...following,state:'hold',watchdog_hold_count:5,watchdog_holding:true,message:'command watchdog expired'};
 events=d.update(wrap(watchdog));assert.equal(events[0].type,'connection');assert.equal(events[0].id,'watchdog');assert.deepEqual(d.update(wrap(watchdog)),[]);
 const legacy=new ArmNoticeDetector();legacy.update(wrap(base));assert.equal(legacy.update(wrap({...base,state:'hold',message:'received command is older than watchdog limit'})).length,1);
 assert.equal(legacy.update(wrap({...base,state:'hold',message:'received command is older than watchdog limit'})).length,0);
 const redOnly=new ArmNoticeDetector({connectionNotices:false});redOnly.update(wrap(base));assert.deepEqual(redOnly.update(wrap({...base,state:'hold',watchdog_hold_count:5,watchdog_holding:true,message:'command watchdog expired'})),[]);
 const reboot={...base,ik_guard_trip_count:0,following_error_trip_count:0,gripper_contact_trip_count:0,watchdog_hold_count:0};assert.deepEqual(d.update(wrap(reboot)),[]);
 assert.equal(d.update(wrap({...reboot,gripper_contact_trip_count:1}))[0].id,'gripper');
 assert.equal(d.update(wrap({...reboot,leader_limited_joints:[2]}))[0].id,'joint-limit');
 assert.equal(d.update(wrap({...reboot,state:'hold',message:'Motion stopped because measured servo telemetry became stale.'}))[0].id,'servo-feedback');
 assert.equal(d.update(wrap({...reboot,state:'fault',hardware_fault:'Motor write failed'}))[0].id,'arm-fault');
 assert.equal(d.update({serverConnected:false,status:reboot})[0].id,'connection');
 const initialActive=new ArmNoticeDetector();assert.equal(initialActive.update(wrap(floor))[0].id,'clearance');
 class Element {
  constructor(){this.children=[];this.dataset={};this.attributes={};this.classes=new Set();this.classList={add:x=>this.classes.add(x),remove:x=>this.classes.delete(x)};}
  append(...items){for(const item of items){item.parent=this;this.children.push(item);}}
  setAttribute(k,v){this.attributes[k]=v;}
  remove(){if(this.parent)this.parent.children=this.parent.children.filter(x=>x!==this);}
  set innerHTML(_){throw Error('HTML injection');}
  focus(){throw Error('Notices must not steal keyboard focus');}
 }
 let now=0,next=0;const pending=new Map(),timers={setTimeout(fn,delay){const id=++next;pending.set(id,{fn,at:now+delay});return id;},clearTimeout(id){pending.delete(id);}};
 const advance=ms=>{const end=now+ms;while(true){const entry=[...pending].sort((a,b)=>a[1].at-b[1].at)[0];if(!entry||entry[1].at>end)break;now=entry[1].at;pending.delete(entry[0]);entry[1].fn();}now=end;};
 const doc={head:new Element(),body:new Element(),createElement:()=>new Element()};
 const stack=new ArmToastStack({document:doc,timers,maxVisible:3});assert.equal(stack.root.attributes['aria-live'],'polite');
 const notice={id:'floor',type:'safeguard',title:'Floor <script>',detail:'Unsafe step blocked.',count:1};
 stack.show(notice);advance(4000);stack.show({...notice,count:2});assert.equal(stack.items.size,1);assert.equal(stack.items.get('floor').count.textContent,'×3');
 assert.equal(stack.items.get('floor').label.textContent,'Floor <script>');advance(1000);assert.equal(stack.items.size,1,'repeat resets expiry');
 advance(4000);assert.ok(stack.items.get('floor').element.classes.has('leaving'));advance(180);assert.equal(stack.items.size,0);
 for(let i=0;i<5;i++)stack.show({...notice,id:String(i)});assert.equal(stack.items.size,3);assert.equal(stack.root.children.length,3);
 stack.dispose();stack.show(notice);stack.update(wrap(floor));assert.equal(pending.size,0);assert.equal(doc.body.children.length,0);assert.equal(doc.head.children.length,0);
 const muted=new ArmToastStack({document:doc,timers,enabled:false});
 assert.equal(muted.root.hidden,true);assert.equal(muted.root.attributes['aria-live'],'off');
 muted.update(wrap(base));muted.update(wrap(floor));muted.show(notice);assert.equal(muted.items.size,0);assert.equal(pending.size,0);
 muted.setEnabled(true);muted.update(wrap(floor));assert.equal(muted.items.size,0,'muted warnings must not replay on re-enable');
 assert.equal(muted.root.hidden,false);assert.equal(muted.root.attributes['aria-live'],'polite');
 muted.update(wrap({...floor,ik_guard_trip_count:7}));assert.equal(muted.items.size,1,'new safeguards show after re-enable');
 muted.setEnabled(false);assert.equal(muted.items.size,0);assert.equal(muted.root.children.length,0);assert.equal(pending.size,0,'muting clears visible notices and timers immediately');
 muted.update({serverConnected:false,status:floor});muted.setEnabled(true);muted.update({serverConnected:false,status:floor});assert.equal(muted.items.size,0,'muted connection notices must not replay');
 muted.update(wrap(floor));muted.update({serverConnected:false,status:floor});assert.equal(muted.items.size,1,'new connection notices resume');
 muted.dispose();assert.equal(pending.size,0);assert.equal(doc.body.children.length,0);
 const storage=new Map(),controls=new Map();
 global.localStorage={getItem:key=>storage.get(key)??null,setItem:(key,value)=>storage.set(key,value)};
 class Control extends EventTarget {blur(){this.blurred=true;}}
 const control=id=>{if(!controls.has(id))controls.set(id,new Control());return controls.get(id);};
 global.document={getElementById:control,querySelectorAll:()=>[]};global.window={addEventListener(){}};
 const moduleSource=fs.readFileSync(path.join(__dirname,'../robot_modules/so101/web/module.js'),'utf8');
 const {createRobotModule}=await import('data:text/javascript;base64,'+Buffer.from(moduleSource).toString('base64'));
 const module=createRobotModule({});assert.equal(module.notificationsEnabled,true);
 const enabled=[];module.armNotices={setEnabled:value=>enabled.push(value)};
 module.bind({sendViewSettings(){throw Error('Notification toggle must not change robot/view settings');},updateStatus(){}});
 const toggle=control('so101-notifications-enabled');toggle.checked=false;toggle.dispatchEvent(new Event('change'));
 assert.deepEqual(enabled,[false]);assert.equal(toggle.blurred,true);assert.equal(createRobotModule({}).notificationsEnabled,false,'muted preference survives a reload');
 toggle.checked=true;toggle.dispatchEvent(new Event('change'));assert.deepEqual(enabled,[false,true]);assert.equal(createRobotModule({}).notificationsEnabled,true);
 global.localStorage={getItem(){throw Error('Storage blocked');},setItem(){throw Error('Storage blocked');}};
 assert.equal(createRobotModule({}).notificationsEnabled,true,'storage failures retain the default');
 toggle.checked=false;toggle.dispatchEvent(new Event('change'));assert.equal(module.notificationsEnabled,false,'storage failures do not break the toggle');
 console.log('PASS safeguard/input classification, muted baseline, immediate clearing, re-enable, bounded stacking, fade and cleanup');
})().catch(e=>{console.error(e);process.exitCode=1;});
