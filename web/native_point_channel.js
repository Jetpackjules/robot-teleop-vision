// Bounded reassembly of independent captures. Missing chunks never block the
// next capture; late/obsolete frames are discarded separately per camera.
export class NativePointAssembler {
  constructor(){this.pending=new Map();this.latest=new Map();}
  accept(buffer,now=performance.now()){
    if(!(buffer instanceof ArrayBuffer)||buffer.byteLength<20||buffer.byteLength>16384)return null;
    const v=new DataView(buffer);
    if(v.getUint32(0,true)!==0x3143504e)return null;
    const id=v.getUint32(4,true),size=v.getUint32(8,true),part=v.getUint16(12,true),count=v.getUint16(14,true),camera=v.getUint32(16,true);
    const stride=16364;
    if(!size||size>512*1024||camera>=16||count!==Math.ceil(size/stride)||part>=count||buffer.byteLength-20!==Math.min(stride,size-part*stride))return null;
    const last=this.latest.get(camera);
    if(last!==undefined&&(((id-last)>>>0)>=0x80000000||last===id))return null;
    for(const [key,entry]of this.pending)if(now-entry.at>250)this.pending.delete(key);
    let entry=this.pending.get(id);
    if(!entry){
      if(this.pending.size>=8)this.pending.delete(this.pending.keys().next().value);
      entry={size,count,camera,at:now,bytes:new Uint8Array(size),seen:new Set()};this.pending.set(id,entry);
    }
    if(entry.size!==size||entry.camera!==camera||entry.seen.has(part))return null;
    entry.bytes.set(new Uint8Array(buffer,20),part*stride);entry.seen.add(part);
    if(entry.seen.size!==count)return null;
    this.pending.delete(id);this.latest.set(camera,id);
    for(const [key,other]of this.pending)if(other.camera===camera&&((id-key)>>>0)<0x80000000)this.pending.delete(key);
    return entry.bytes.buffer;
  }
}
