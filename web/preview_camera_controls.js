const KEY='robotTeleopPreviewExcludedCameras';
export class PreviewCameraControls {
  constructor(root,onChange){
    this.root=root;this.onChange=onChange;this.excluded=[];
    try{const ids=JSON.parse(localStorage.getItem(KEY)||'[]');if(Array.isArray(ids))this.excluded=ids.filter(id=>typeof id==='string'&&/^[A-Za-z0-9_-]{1,64}$/.test(id)).slice(0,8);}catch{}
    this.load();
  }
  async load(){
    try{
      const r=await fetch('/api/v1/cameras',{cache:'no-store'});if(!r.ok)throw Error();
      const {cameras}=await r.json();if(!cameras?.length)throw Error();
      if(cameras.every(c=>this.excluded.includes(c.id)))this.excluded=this.excluded.filter(id=>id!==cameras[0].id);
      this.root.replaceChildren();
      for(const c of cameras){
        const label=document.createElement('label');label.className='toggle-row';
        const text=document.createElement('span');text.textContent=`${c.name||c.model||'Camera'} · ${c.id.slice(-4)}`;
        const input=document.createElement('input');input.type='checkbox';input.checked=!this.excluded.includes(c.id);input.setAttribute('aria-label',`Include ${c.name||c.id} in live 3D`);
        input.addEventListener('change',()=>{
          if(!input.checked&&this.root.querySelectorAll('input:checked').length===0){input.checked=true;return;}
          this.excluded=input.checked?this.excluded.filter(id=>id!==c.id):[...new Set([...this.excluded,c.id])];
          localStorage.setItem(KEY,JSON.stringify(this.excluded));this.onChange();
        });label.append(text,input);this.root.append(label);
      }
      this.onChange();
    }catch{this.root.textContent='Waiting for camera list…';this.timer=setTimeout(()=>this.load(),3000);}
  }
  close(){clearTimeout(this.timer);}
}
