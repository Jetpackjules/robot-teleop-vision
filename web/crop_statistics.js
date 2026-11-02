// Transport, retained samples and draw submission have different denominators.
// None of these counters measures the fraction of the screen that is visible.
const count = value => typeof value === 'number' && Number.isFinite(value) && value >= 0;
const integer = value => Math.round(value).toLocaleString('en-US');
function bytes(value) {
  if(value < 1024)return `${integer(value)} B`;
  if(value < 1024*1024)return `${(value/1024).toFixed(1)} KiB`;
  return `${(value/(1024*1024)).toFixed(2)} MiB`;
}
function retained(kept,source,noun) {
  if(!count(kept)||!count(source))return 'Not reported by the host';
  if(source===0)return `No ${noun} in the source packet`;
  return `${(100*kept/source).toFixed(1)}% kept · ${integer(kept)} of ${integer(source)}`;
}

export function cropStatistics(status,{is3D=true}={}) {
  if(!is3D)return {message:'Table statistics apply to 3D views. Select a 3D view to measure cropping.',rows:[]};
  const network=status?.streamCrop,geometry=status?.workspaceCrop,rows=[];
  if(network?.applied&&count(network.input_bytes)&&network.input_bytes>0&&count(network.output_bytes)) {
    const fraction=network.output_bytes/network.input_bytes,saved=100*(1-fraction);
    rows.push(['Network bytes',`${(100*fraction).toFixed(1)}% sent · ${Math.abs(saved).toFixed(1)}% ${saved>=0?'saved':'more bytes'} (${bytes(network.output_bytes)} of ${bytes(network.input_bytes)})`]);
    rows.push(['Depth samples',retained(network.retained_samples,network.source_samples,'valid depth samples')]);
    rows.push(['RGB pixel area',retained(network.color_pixels,network.source_color_pixels,'RGB pixels')]);
  } else {
    rows.push(['Server crop','Not active in the latest packet; cropping is applied on the laptop']);
  }
  rows.push(['3D grid submission',count(geometry?.sourceCells)&&geometry.sourceCells>0&&count(geometry?.submittedCells)
    ? `${(100*geometry.submittedCells/geometry.sourceCells).toFixed(1)}% submitted · ${integer(geometry.submittedCells)} of ${integer(geometry.sourceCells)} received grid cells`
    : 'Waiting for a rendered 3D frame']);
  return {message:'',rows};
}

export function renderCropStatistics(element,status,options) {
  if(!element)return;
  const model=cropStatistics(status,options),signature=JSON.stringify(model);
  // Status is polled frequently. Do not replace or announce identical values.
  if(element.dataset.cropStatistics===signature&&element.querySelector('.crop-statistics-list, [data-crop-message]'))return;
  element.dataset.cropStatistics=signature;
  const document=element.ownerDocument,children=[];
  const current=element.querySelector('.crop-statistics-list');
  if(model.rows.length&&current&&current.querySelectorAll('dt').length===model.rows.length
      &&[...current.querySelectorAll('dt')].every((term,index)=>term.textContent===model.rows[index][0])) {
    current.querySelectorAll('dd').forEach((description,index)=>{description.textContent=model.rows[index][1];});
    const message=element.querySelector('[data-crop-message]');
    message.textContent=model.message;message.hidden=!model.message;
    return;
  }
  const message=document.createElement('p');message.dataset.cropMessage='';message.textContent=model.message;message.hidden=!model.message;children.push(message);
  if(model.rows.length){
    const list=document.createElement('dl');list.className='crop-statistics-list';
    for(const [label,value]of model.rows){
      const term=document.createElement('dt'),description=document.createElement('dd');
      term.textContent=label;description.textContent=value;list.append(term,description);
    }
    children.push(list);
    const details=document.createElement('details'),summary=document.createElement('summary'),help=document.createElement('p');
    summary.textContent='What these numbers measure';
    help.textContent='Server figures describe the latest packet; grid submission covers all loaded cameras after the server crop. RGB area counts pixels, not compressed bytes. Each grid cell can form two triangles; invalid or masked triangles may still be discarded. Color-visibility and fusion passes add work beyond this count. These percentages do not measure visible screen area.';
    details.append(summary,help);children.push(details);
  }
  element.replaceChildren(...children);
}
