import {vadd,vsub,vmul,vdot,vcross,vunit,transform3,multiply3,rotation3,rotationDegrees,
  orientedBox,rayPlane,rayAxis,projectWorld,screenRay,workspaceEditBox,workspaceEditSettings,resizeWorkspaceBox} from './workspace_transform.js?v=20261021_direct_workspace';
import {WorkspaceCoverage,coverageText} from './workspace_coverage.js';

const NS='http://www.w3.org/2000/svg',COLORS=['#ff9b87','#aee2a1','#95caff'];
const svg=(tag,attributes={})=>{const node=document.createElementNS(NS,tag);for(const [k,v]of Object.entries(attributes))node.setAttribute(k,String(v));return node;};
const clone=value=>JSON.parse(JSON.stringify(value));

// Only the handles intercept pointers. Ordinary orbit/pan/zoom stays available
// everywhere else, including while the editor is open.
export class WorkspaceEditor {
  constructor({getSettings,onChange,onFinish,onFrame,onWheel,getCoverage=null,geometry=null,onRequestFinish=null}) {
    Object.assign(this,{getSettings,onChange,onFinish,onFrame,onWheel,getCoverage,geometry,onRequestFinish});this.mode='move';this.active=false;
    this.root=document.createElement('div');this.root.id='workspace-editor';this.root.hidden=true;
    this.root.innerHTML=`<div class="workspace-edit-toolbar" role="toolbar" aria-label="3D workspace editor">
      <strong id="workspace-edit-title">Edit table</strong><button type="button" data-action="frame">Fit in View</button>
      <div class="workspace-edit-modes" role="group" aria-label="Transform tool">
        <button type="button" data-tool="move" aria-pressed="true">Move <kbd>W</kbd></button>
        <button type="button" data-tool="resize" aria-pressed="false">Resize <kbd>R</kbd></button>
        <button type="button" data-tool="rotate" aria-pressed="false">Rotate <kbd>E</kbd></button>
      </div><span class="workspace-edit-actions"><button type="button" data-action="reset" hidden>Reset</button><button type="button" data-action="cancel">Cancel</button><button type="button" data-action="done" class="primary">Done</button></span>
      <p id="workspace-edit-help"></p><output id="workspace-edit-readout" aria-live="off"></output>
      <output id="workspace-edit-coverage" aria-live="off" hidden title="Measured camera points inside the table region, before robot masking and meshing. Includes camera overlaps; does not measure screen coverage or occlusion."></output>
    </div>`;
    this.svg=svg('svg',{'aria-label':'3D transform handles'});this.root.prepend(this.svg);
    const definitions=svg('defs'),clip=svg('clipPath',{id:'workspace-editor-visible-area'});this.clip=svg('path');clip.append(this.clip);definitions.append(clip);this.svg.append(definitions);
    this.lines=svg('g',{'class':'workspace-edit-wire'});this.handles=svg('g');this.svg.append(this.lines,this.handles);
    this.lines.setAttribute('clip-path','url(#workspace-editor-visible-area)');this.handles.setAttribute('clip-path','url(#workspace-editor-visible-area)');
    document.body.append(this.root);
    this.root.querySelectorAll('[data-tool]').forEach(button=>button.addEventListener('click',()=>this.setMode(button.dataset.tool)));
    this.root.querySelector('[data-action=done]').onclick=()=>this.finish(true);
    this.root.querySelector('[data-action=cancel]').onclick=()=>this.finish(false);
    this.root.querySelector('[data-action=reset]').onclick=()=>Promise.resolve(this.geometry?.onReset?.()).catch(error=>{this.root.querySelector('#workspace-edit-help').textContent=error.message||String(error);});
    this.root.querySelector('[data-action=frame]').onclick=()=>this.onFrame?.(this.editBox());
    this.svg.addEventListener('wheel',event=>this.onWheel?.(event),{passive:false});
    this.svg.addEventListener('pointermove',event=>this.pointerMove(event));
    this.svg.addEventListener('pointerup',event=>this.pointerUp(event));
    this.svg.addEventListener('pointercancel',event=>this.pointerUp(event,true));
    this.svg.addEventListener('lostpointercapture',()=>{if(this.drag)this.endDrag(true);});
    this.key=event=>this.keyDown(event);window.addEventListener('keydown',this.key,true);
  }
  editBox(){return this.geometry?.getBox?.()||workspaceEditBox(this.getSettings(),this.target);}
  editSettings(box){return this.geometry?.toSettings?.(box)||workspaceEditSettings(box,this.target);}
  begin(target) {
    if(this.active)this.finish(true);
    this.target=target;this.before=clone(this.editSettings(this.editBox()));this.active=true;this.root.hidden=false;this.lastKey='';
    const coverage=this.root.querySelector('#workspace-edit-coverage');coverage.hidden=target!=='table'||!this.getCoverage;
    this.coverage?.close();this.coverage=null;
    if(!coverage.hidden){
      coverage.textContent='Point cloud kept: waiting for full camera frames…';coverage.dataset.state='waiting';
      this.coverage=new WorkspaceCoverage({onResult:result=>{
        if(!this.active||this.target!=='table')return;
        coverage.textContent=coverageText(result);coverage.dataset.state=result.state;
        if(result.state==='ready'){coverage.dataset.percent=String(result.percent??'');coverage.dataset.total=String(result.total);coverage.dataset.kept=String(result.kept);}
        else {delete coverage.dataset.percent;delete coverage.dataset.total;delete coverage.dataset.kept;}
      }});
    }
    this.root.querySelector('#workspace-edit-title').textContent=this.geometry?.title||'Edit table';
    this.root.querySelector('[data-tool=resize]').hidden=this.geometry?.planar===true;
    this.root.querySelector('[data-tool=rotate]').firstChild.textContent=this.geometry?.planar?'Yaw ':'Rotate ';
    this.root.querySelector('[data-action=done]').textContent=this.geometry?.saveLabel||'Done';
    this.root.querySelector('[data-action=reset]').hidden=!this.geometry?.onReset;
    this.setMode('move');this.root.querySelector('[data-tool=move]').focus({preventScroll:true});
  }
  setMode(mode) {
    if(this.finishing||(mode==='resize'&&this.geometry?.planar))return;
    if(this.drag)this.endDrag(false);
    this.mode=mode;this.lastKey='';
    this.root.querySelectorAll('[data-tool]').forEach(button=>button.setAttribute('aria-pressed',String(button.dataset.tool===mode)));
    this.root.querySelector('#workspace-edit-help').textContent=this.geometry?.planar
      ?(mode==='move'?(this.geometry.verticalTranslation?'Drag the center across the table. Use Y to raise or lower the base.':'Drag the center across the table, or use an axis. Shift + arrow keys makes fine adjustments.')
        :this.geometry.verticalTranslation?'Drag the ring to turn the base. Tilt stays fixed.':'Drag the ring to turn the base. Height and tilt stay fixed.')
      :mode==='move'
      ?'Drag the center to move freely, or a colored axis for one direction. Drag the scene to orbit.'
      :mode==='resize'?'Drag a face to resize one dimension; drag a corner to resize several. The opposite side stays fixed.'
        :this.target==='table'?'Drag the ring to turn the table. It stays level.':'Drag a colored ring to rotate around that axis.';
  }
  finish(commit) {
    if(!this.active||this.finishing)return;
    if(this.drag)this.endDrag(!commit);
    if(this.onRequestFinish){
      this.finishing=true;
      this.root.querySelectorAll('button').forEach(button=>button.disabled=true);
      return Promise.resolve().then(()=>this.onRequestFinish(commit)).then(ok=>{
        if(ok!==false)this.close(commit);
      }).catch(error=>{this.root.querySelector('#workspace-edit-help').textContent=error.message||String(error);}).finally(()=>{
        this.finishing=false;this.root.querySelectorAll('button').forEach(button=>button.disabled=false);
      });
    }
    if(!commit)this.onChange(clone(this.before),'cancel');
    else this.onChange(this.editSettings(this.editBox()),'commit');
    this.close(commit);
  }
  close(commit){
    this.coverage?.close();this.coverage=null;
    this.active=false;this.root.hidden=true;this.lines.replaceChildren();this.handles.replaceChildren();this.onFinish?.(commit,this.target);
  }
  update(view) {
    if(!this.active)return;
    if(this.coverage)this.coverage.update(this.getCoverage());
    this.view=view;const box=this.editBox();this.box=box;
    // Do not let a projected handle intercept the controller or Hold button.
    const panels=Array.from(document.querySelectorAll('.topbar .control-stack,.topbar .status-chip'),el=>el.getBoundingClientRect()).filter(r=>r.width&&r.height);
    let areas=[[0,0,view.width,view.height]];
    for(const panel of panels) {
      const x0=panel.left-(view.left||0)-4,y0=panel.top-(view.top||0)-4,x1=x0+panel.width+8,y1=y0+panel.height+8;
      areas=areas.flatMap(([l,t,r,b])=>{
        const a=Math.max(l,x0),c=Math.min(r,x1),d=Math.max(t,y0),e=Math.min(b,y1);
        if(a>=c||d>=e)return [[l,t,r,b]];
        return [[l,t,r,d],[l,e,r,b],[l,d,a,e],[c,d,r,e]].filter(([l,t,r,b])=>r>l&&b>t);
      });
    }
    this.clip.setAttribute('d',areas.map(([l,t,r,b])=>`M${l},${t}H${r}V${b}H${l}Z`).join(''));
    const key=JSON.stringify([view.viewProjection,view.width,view.height,box.center,box.size,box.rotation,this.mode,areas]);
    if(key===this.lastKey)return;this.lastKey=key;
    this.svg.setAttribute('viewBox',`0 0 ${view.width} ${view.height}`);
    this.svg.style.left=`${view.left||0}px`;this.svg.style.top=`${view.top||0}px`;
    this.svg.style.width=`${view.width}px`;this.svg.style.height=`${view.height}px`;
    this.lines.replaceChildren();
    const projected=box.corners.map(p=>projectWorld(p,view));
    for(let i=0;i<8;i++)for(let axis=0;axis<3;axis++) {
      const j=i^(1<<axis);if(j<i||!projected[i]||!projected[j])continue;
      this.lines.append(svg('line',{x1:projected[i][0],y1:projected[i][1],x2:projected[j][0],y2:projected[j][1]}));
    }
    this.root.querySelector('#workspace-edit-readout').textContent=
      this.geometry?.readout?.(box)||`${box.size.map(v=>v.toFixed(3)).join(' × ')} m · ${this.target==='table'?box.rotation[1].toFixed(1)+'°':box.rotation.map(v=>v.toFixed(1)+'°').join(' / ')}`;
    // Keep the captured handle alive throughout a drag. Its coordinates still
    // follow the updated box through the wireframe; rebuild after release.
    if(this.drag)return;
    this.handles.replaceChildren();
    const center=projectWorld(box.center,view);if(!center)return;
    if(this.mode==='move') {
      this.addPoint(center,{kind:'plane'},this.geometry?.planar?'Move across table':'Move in view plane','#ffffff',9,'rect');
      const distance=Math.hypot(...vsub(box.center,Array.from(view.viewer.slice(12,15))));
      const length=Math.max(.08,distance*Math.tan(view.fov*Math.PI/360)*.3);
      for(let axis=0;axis<3;axis++) {
        if(this.geometry?.planar&&!this.geometry.verticalTranslation&&axis===1)continue;
        const direction=[0,0,0];direction[axis]=1;
        const end=projectWorld(vadd(box.center,vmul(direction,length)),view);
        if(!end||Math.hypot(end[0]-center[0],end[1]-center[1])<22)continue;
        const line=svg('line',{x1:center[0],y1:center[1],x2:end[0],y2:end[1],stroke:COLORS[axis],'stroke-width':3});
        this.lines.append(line);
        const fraction=20/Math.hypot(end[0]-center[0],end[1]-center[1]);
        const hit=svg('line',{x1:center[0]+(end[0]-center[0])*fraction,y1:center[1]+(end[1]-center[1])*fraction,x2:end[0],y2:end[1],stroke:'transparent','stroke-width':16,'class':'workspace-handle workspace-stroke-hit'});
        this.bindHandle(hit,{kind:'axis',axis,direction});this.handles.append(hit);
        this.addPoint(end,{kind:'axis',axis,direction},`Move ${'XYZ'[axis]}`,COLORS[axis],7);
        const label=svg('text',{x:end[0]+12,y:end[1]+4,fill:COLORS[axis]});label.textContent='XYZ'[axis];this.lines.append(label);
      }
    } else if(this.mode==='resize') {
      for(let axis=0;axis<3;axis++)for(const sign of [-1,1]) {
        const local=[0,0,0];local[axis]=sign*box.half[axis];const p=projectWorld(vadd(box.center,transform3(box.matrix,local)),view);
        if(p){const signs=[0,0,0];signs[axis]=sign;this.addPoint(p,{kind:'face',axis,signs,direction:box.matrix.slice(axis*3,axis*3+3)},`Resize ${'XYZ'[axis]} ${sign>0?'positive':'negative'} face`,COLORS[axis],7,'rect');}
      }
      projected.forEach((p,i)=>{if(p)this.addPoint(p,{kind:'corner',signs:[0,1,2].map(a=>(i&(1<<a))?1:-1)},`Resize corner ${i+1}`,'#ffffff',5,'rect');});
    } else {
      const radius=Math.max(...box.half)*1.16;
      for(const axis of (this.target==='table'||this.geometry?.planar)?[1]:[0,1,2]) {
        const points=[];
        for(let i=0;i<=96;i++) {const p=[0,0,0],angle=i*Math.PI/48;p[(axis+1)%3]=Math.cos(angle)*radius;p[(axis+2)%3]=Math.sin(angle)*radius;points.push(projectWorld(vadd(box.center,p),view));}
        if(points.some(p=>!p))continue;
        const d=points.map((p,i)=>`${i?'L':'M'}${p[0]},${p[1]}`).join(' ');
        const path=svg('path',{d,fill:'none',stroke:COLORS[axis],'stroke-width':3,'class':'workspace-ring'});
        const hit=svg('path',{d,fill:'none',stroke:'transparent','stroke-width':22,'class':'workspace-handle workspace-stroke-hit',tabindex:0,role:'slider','aria-label':`Rotate ${'XYZ'[axis]}`});
        this.bindHandle(hit,{kind:'rotate',axis});this.handles.append(path,hit);
      }
    }
  }
  addPoint(p,handle,label,color,size,shape='circle') {
    const g=svg('g',{'class':'workspace-handle',tabindex:0,role:'slider','aria-label':label});
    g.append(svg('circle',{cx:p[0],cy:p[1],r:15,fill:'transparent'}));
    g.append(svg(shape,shape==='circle'?{cx:p[0],cy:p[1],r:size,fill:color,stroke:'#10202a','stroke-width':2}
      :{x:p[0]-size,y:p[1]-size,width:size*2,height:size*2,rx:2,fill:color,stroke:'#10202a','stroke-width':2}));
    this.bindHandle(g,handle);this.handles.append(g);
  }
  bindHandle(node,handle) {
    const axis=handle.axis??0,value=handle.kind==='rotate'?this.box.rotation[axis]:this.mode==='resize'?this.box.size[axis]:this.box.center[axis];
    node.setAttribute('aria-valuenow',String(value));node.setAttribute('aria-valuemin',handle.kind==='rotate'?'-360':this.mode==='resize'?'0.02':'-20');node.setAttribute('aria-valuemax',handle.kind==='rotate'?'360':'20');
    node.setAttribute('aria-valuetext',`${value.toFixed(handle.kind==='rotate'?1:3)} ${handle.kind==='rotate'?'degrees':'metres'}`);

    node.addEventListener('pointerdown',event=>{
      if(event.button!==0||!this.active||this.finishing)return;
      const view=this.view,ray=screenRay(event.clientX-(view.left||0),event.clientY-(view.top||0),view),box=this.box;
      const normal=handle.kind==='rotate'?[0,1,2].map(a=>a===handle.axis?1:0):this.geometry?.planar?[0,1,0]:vunit(Array.from(view.viewer.slice(8,11)));
      const startPlane=rayPlane(ray,box.center,normal),startAxis=handle.direction?rayAxis(ray,box.center,handle.direction):null;
      if((['plane','corner','rotate'].includes(handle.kind)&&!startPlane)||(['axis','face'].includes(handle.kind)&&startAxis===null))return;
      this.drag={handle,start:box,view: {...view},normal,startPlane,startAxis,id:event.pointerId,node,before:clone(this.editSettings(box))};
      node.setPointerCapture(event.pointerId);node.classList.add('is-dragging');event.preventDefault();event.stopPropagation();
    });
    node.addEventListener('keydown',event=>{
      if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(event.key))return;
      if(this.finishing)return;
      const sign=['ArrowRight','ArrowUp'].includes(event.key)?1:-1,box=this.box,axis=handle.axis??(this.geometry?.planar&&['ArrowUp','ArrowDown'].includes(event.key)?2:0);
      const step=sign*(event.shiftKey?.001:.01);let next;
      if(handle.kind==='rotate'){const angle=[0,0,0];angle[axis]=sign*(event.shiftKey?.1:1);next=orientedBox(box.center,box.size,(this.target==='table'||this.geometry?.planar)?[0,((box.rotation[1]+sign*(event.shiftKey?.1:1)+540)%360)-180,0]:rotationDegrees(multiply3(rotation3(angle),box.matrix)));}
      else if(this.mode==='resize'){const delta=[0,0,0];delta[axis]=step;next=resizeWorkspaceBox(box,handle.signs,delta,this.target);}
      else {const delta=[0,0,0];delta[axis]=step;next=orientedBox(vadd(box.center,delta),box.size,box.rotation);}
      this.onChange(this.editSettings(next),'commit');event.preventDefault();event.stopPropagation();
    });
  }
  pointerMove(event) {
    const d=this.drag;if(!d||event.pointerId!==d.id||this.finishing)return;
    const ray=screenRay(event.clientX-(d.view.left||0),event.clientY-(d.view.top||0),d.view),h=d.handle;let next;
    if(h.kind==='axis'||h.kind==='face') {
      const value=rayAxis(ray,d.start.center,h.direction);if(value===null)return;
      const delta=value-d.startAxis;
      if(h.kind==='axis')next=orientedBox(vadd(d.start.center,vmul(h.direction,delta)),d.start.size,d.start.rotation);
      else {const local=[0,0,0];local[h.axis]=delta;next=resizeWorkspaceBox(d.start,h.signs,local,this.target);}
    } else {
      const point=rayPlane(ray,d.start.center,d.normal);if(!point)return;
      if(h.kind==='rotate') {
        const a=vunit(vsub(d.startPlane,d.start.center)),b=vunit(vsub(point,d.start.center));
        const angle=Math.atan2(vdot(d.normal,vcross(a,b)),vdot(a,b))*180/Math.PI,rotation=[0,0,0];rotation[h.axis]=angle;
        next=orientedBox(d.start.center,d.start.size,(this.target==='table'||this.geometry?.planar)?[0,((d.start.rotation[1]+angle+540)%360)-180,0]:rotationDegrees(multiply3(rotation3(rotation),d.start.matrix)));
      } else if(h.kind==='corner')next=resizeWorkspaceBox(d.start,h.signs,transform3(d.start.inverse,vsub(point,d.startPlane)),this.target);
      else next=orientedBox(vadd(d.start.center,vsub(point,d.startPlane)),d.start.size,d.start.rotation);
    }
    if(next.center.some(v=>Math.abs(v)>19)||next.rotation.some(v=>!Number.isFinite(v)))return;
    this.onChange(this.editSettings(next),'preview');event.preventDefault();
  }
  pointerUp(event,cancel=false){if(this.drag&&event.pointerId===this.drag.id){this.endDrag(cancel);event.preventDefault();}}
  endDrag(cancel) {
    const d=this.drag;if(!d)return;this.drag=null;this.lastKey='';
    if(d.node.hasPointerCapture(d.id))d.node.releasePointerCapture(d.id);
    this.onChange(cancel?d.before:this.editSettings(this.editBox()),cancel?'cancel-drag':'commit');
  }
  keyDown(event) {
    if(!this.active||/INPUT|TEXTAREA|SELECT/.test(event.target?.tagName)||event.ctrlKey||event.metaKey||event.altKey)return;
    if(event.key==='Escape'){this.finish(false);event.preventDefault();event.stopImmediatePropagation();}
    else if(['w','e','r'].includes(event.key.toLowerCase())){this.setMode({w:'move',e:'rotate',r:'resize'}[event.key.toLowerCase()]);event.preventDefault();event.stopImmediatePropagation();}
    else if(['a','s','d','f','i','k','j','l','q'].includes(event.key.toLowerCase())){event.preventDefault();event.stopImmediatePropagation();}
    else if(event.key==='Enter'&&event.target===document.body){this.finish(true);event.preventDefault();}
  }
  dispose(){this.finish(false);window.removeEventListener('keydown',this.key,true);this.root.remove();}
}
