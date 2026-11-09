// Optional recovery of archived visual adjustments. Motor calibration is never sent.
export function applyCorrection(registration, group, enabled) {
  const next = structuredClone(registration);
  for (const [key, value] of Object.entries(enabled ? group.old : group.neutral)) {
    if (Number.isInteger(group.index)) next[key][group.index] = value[group.index];
    else next[key] = structuredClone(value);
  }
  return next;
}

export function correctionEnabled(registration, group) {
  const same = (a,b) => typeof a === 'number' && typeof b === 'number' ? Math.abs(a-b)<1e-5
    : Array.isArray(a) && Array.isArray(b) ? a.length===b.length && a.every((v,i)=>same(v,b[i])) : a===b;
  return Object.entries(group.old).every(([key, value]) => {
    const actual = Number.isInteger(group.index) ? registration[key]?.[group.index] : registration[key];
    const expected = Number.isInteger(group.index) ? value[group.index] : value;
    return same(actual,expected);
  });
}

export async function mountLegacyCorrections(owner, anchor) {
  const response = await fetch(new URL('./legacy_visual_corrections.local.json', import.meta.url), {cache:'no-store'});
  if (response.status === 404) return; // No archived adjustments on this host.
  if (!response.ok) throw new Error('Could not load archived visual corrections.');
  const archive = await response.json();
  if (archive.version !== 1 || !Array.isArray(archive.groups)) throw new Error('Unsupported visual correction archive.');
  const details = document.createElement('details');
  details.className = 'advanced-disclosure';
  const summary = document.createElement('summary'); summary.textContent = 'Previous arm visual corrections'; details.append(summary);
  const content = document.createElement('div'); content.className = 'disclosure-body'; details.append(content);
  const help = document.createElement('p'); help.className = 'helper-text';
  help.textContent = 'Restore saved adjustments individually. Each change saves on the host for everyone. Base placement moves the overlay only; with it off, the model uses the scene origin. The unfitted claw uses a linear model reference.';
  content.append(help);
  const status = document.createElement('p'); status.className = 'helper-text'; status.setAttribute('role','status');
  const controls = archive.groups.map(group => {
    const label = document.createElement('label'); label.className = 'toggle-row';
    const text = document.createElement('span'); text.textContent = group.label;
    const input = document.createElement('input'); input.type = 'checkbox'; input.disabled = true; input.dataset.legacyCorrection = group.id;
    label.append(text,input); content.append(label); return {group,input};
  });
  content.append(status); anchor.after(details);
  let busy = false;
  const read = async () => {
    const reply = await owner.calibrationAction('export');
    if (!reply.ok || !reply.file) throw new Error(reply.message || 'Visual calibration unavailable.');
    return reply.file;
  };
  const reflect = file => controls.forEach(({group,input}) => { input.checked = correctionEnabled(file.registration,group); });
  details.addEventListener('toggle', async () => {
    if (!details.open || busy) return;
    busy = true; controls.forEach(({input})=>input.disabled=true); status.textContent='Reading saved corrections…';
    try { reflect(await read()); status.textContent='Unchecked corrections are off. Original values remain archived.'; }
    catch (error) { status.textContent=error.message; }
    finally { busy=false; controls.forEach(({input})=>input.disabled=false); }
  });
  for (const {group,input} of controls) input.addEventListener('change', async () => {
    if (busy) return;
    busy=true; const enabled=input.checked; controls.forEach(({input})=>input.disabled=true); status.textContent='Saving visual correction…';
    try {
      // Read immediately before changing one group: preserve all other current tuning.
      const file=await read();
      file.registration=applyCorrection(file.registration,group,enabled);
	  if (group.id==='base') file.base_reference_only=!enabled;
      const reply=await owner.calibrationAction('import',{file,restore_base:group.id==='base'});
      if (!reply.ok) throw new Error(reply.message || 'Could not save the correction.');
      reflect(file); status.textContent='Saved on the host. No motor settings changed.';
    } catch(error) { input.checked=!enabled; status.textContent=error.message; }
    finally { busy=false; controls.forEach(({input})=>input.disabled=false); }
  });
}
