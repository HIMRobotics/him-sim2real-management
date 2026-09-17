'use strict';
const $ = id => document.getElementById(id);
const key = new URLSearchParams(location.hash.slice(1)).get('key');
if (key) { sessionStorage.setItem('g1-key', key); history.replaceState(null, '', '/'); }
const token = sessionStorage.getItem('g1-key') || '';
let state = null, selected = localStorage.getItem('g1-selected') || '', initialized = false;
let busy = false, downloading = null, uploadBusy = false, pollBusy = false, offline = false;
let viewMode=sessionStorage.getItem('g1-view')||'local';
const labels = {starting:'STARTING',recording:'RECORDING',stopping:'SAVING',stale:'STATE STREAM STALE',saved:'SAVED',error:'NEEDS ATTENTION',incomplete:'INCOMPLETE'};
function visible(id, yes) { $(id).classList.toggle('hidden', !yes); }
function error(message) { $('error').textContent = message; visible('error', !!message); }
function toast(message) { $('toast').textContent = message; visible('toast', !!message); }
function config() { return {interface:$('interface').value,domain:Number($('domain').value),python:$('python').value}; }
async function api(path, body) {
  const response = await fetch(path, {method:body === undefined ? 'GET' : 'POST', headers:{'X-G1-Token':token,...(body===undefined?{}:{'Content-Type':'application/json'})}, body:body===undefined?undefined:JSON.stringify(body)});
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || 'The operation failed. Your saved runs are retained.');
  return data;
}
async function action(fn) {
  if (busy) return;
  busy = true; error(''); toast(''); render();
  try { await fn(); await refresh(); } catch(e) { error(e.message); }
  finally { busy = false; render(); }
}
function settingsChanged() {
  if (state) { state.connection_valid = false; state.connection = {state:'unchecked',message:'Settings changed. Check this connection before recording.'}; }
  render();
}
['interface','domain','python'].forEach(id => $(id).addEventListener('input',settingsChanged));
function node(tag, text, cls) { const n=document.createElement(tag); n.textContent=text; if(cls)n.className=cls; return n; }
function selectedRun() { return state?.runs.find(r=>r.id===selected); }
function render() {
  if (!state) return;
  renderRemote();
  const jobWorking=state.job?.state==='working', locked=busy||jobWorking||uploadBusy||offline;
  visible('demo',state.demo); $('mode').textContent=state.demo?'DEMO · NO ROBOT':'LOCAL · READ ONLY';
  $('version').textContent='v'+state.version; $('disk').textContent=state.free_gib+' GB free';
  $('connection').textContent=state.connection.message;
  $('check').disabled=locked||!!state.active;
  $('check').textContent=state.connection.state==='checking'?'Checking…':'Check connection';
  visible('check-details',!!state.connection.details); $('check-log').textContent=state.connection.details||'';
  const connectionMatches = state.connection_valid && config().interface===(state.settings.interface||'auto') && config().domain===(state.settings.domain??0) && (config().python||state.python)===(state.settings.python||state.python);
  $('start').disabled=locked||!!state.active||!connectionMatches;
  $('start-help').textContent=state.active?'Recording is running. Stop it before starting another run.':connectionMatches?'Connected. Start recording, wait for the green indicator, then launch your policy.':'Check the connection first. It is valid for five minutes.';
  ['interface','domain','python'].forEach(id=>$(id).disabled=locked||!!state.active);
  const run = selectedRun() || (state.active===selected ? {id:selected,name:$('name').value,policy:$('policy').value,phase:'starting',labels:[],evidence:[],state_samples:0,command_samples:0,warnings:[]} : null);
  visible('empty',!run); visible('run-view',!!run);
  if (run) {
    $('run-title').textContent=run.name; $('run-policy').textContent=run.policy||'';
    $('phase').textContent=labels[run.phase]; $('phase').className='badge'+(run.phase==='recording'?' live':['error','stale','incomplete'].includes(run.phase)?' bad':'');
    $('state-count').textContent=run.state_samples.toLocaleString(); $('command-count').textContent=run.command_samples.toLocaleString();
    $('freshness').textContent=run.phase==='saved'?'Saved':run.phase==='recording'?'Live':run.phase==='stale'?'Stale':'—';
    $('freshness-label').textContent=run.phase==='saved'?'on this computer':'state stream';
    const live=['starting','recording','stopping','stale'].includes(run.phase);
    $('run-guidance').textContent={starting:'Waiting for actual robot data. Do not launch the policy yet.',recording:state.demo?'Synthetic recording is running. Try stopping, labeling and downloading it.':'Recording. Launch the policy in its usual terminal. Stop the policy normally before stopping this recorder.',stopping:'Finishing the recording and writing the report. Please wait.',stale:'Robot state is not fresh. Check the robot and network. The app does not stop the policy.',saved:'Run saved. Add an outcome and any files, then prepare the ZIP to send to Arena.',error:'This run has a collection error. Keep it and send its data/error log too.',incomplete:'The recorder did not finalize this run. Data is preserved, but completeness is unknown.'}[run.phase];
    visible('stop',live); $('stop').disabled=busy||offline||run.phase==='stopping'; $('stop').textContent=run.phase==='stopping'?'Saving…':'Stop recording';
    const warnings=Array.from(new Set([run.error,...run.warnings].filter(Boolean)));
    visible('quality',warnings.length>0); $('quality').textContent=warnings.join(' ');
    visible('label-details',!['starting','stopping','incomplete'].includes(run.phase)); $('save-label').disabled=locked;
    $('labels').replaceChildren(...run.labels.map(l=>node('span',l.trial+' · '+l.outcome,'chip')));
    const finished=['saved','error'].includes(run.phase);
    visible('finished',finished); visible('recovery',run.phase==='incomplete');
    $('files').disabled=locked; $('recover').disabled=locked;
    $('attachments').replaceChildren(...run.evidence.map(e=>node('span',e.original_name||e.path,'chip')));
    $('export').disabled=locked; $('export').textContent=jobWorking&&state.job.name==='Preparing return file'?'Preparing & verifying…':'Prepare data for Arena';
    const result=state.job?.state==='done'?state.job.result:null;
    if(result?.download && result.run===run.id) downloading=result;
    visible('download',!!downloading&&downloading.run===run.id);
    if(downloading?.run===run.id) { $('download').href=downloading.download; $('download').download=downloading.filename; }
    visible('report',!!run.has_report); $('report').href='/report/'+run.id;
    $('run-path').textContent=run.path||'Creating run folder…';
  } else { $('phase').textContent='NO RUN SELECTED'; $('phase').className='badge'; }
  const list=$('runs');
  list.replaceChildren();
  if(!state.runs.length) list.append(node('p','No runs yet.','muted'));
  state.runs.forEach(r=>{const b=node('button','','run-row'+(r.id===selected?' selected':''));const left=node('span','');left.append(node('strong',r.name),node('small',(r.source==='synthetic_demo'?'DEMO · ':'')+(r.policy||'')+' · '+r.state_samples.toLocaleString()+' state samples'));b.append(left,node('span',labels[r.phase],'badge'));b.addEventListener('click',()=>{selected=r.id;localStorage.setItem('g1-selected',selected);$('run-log').textContent='';render();});list.append(b);});
  if(state.job?.state==='error') error(state.job.message);
}
async function refresh() {
  if(pollBusy) return;
  pollBusy=true;
  try {
    const fresh=await api('/api/state');
    if(!initialized) {
      $('interface').replaceChildren(node('option','Automatic'));
      $('interface').firstChild.value='auto';
      fresh.interfaces.forEach(name=>{const opt=node('option',name);opt.value=name;$('interface').append(opt);});
      for(const id of ['interface','domain','python','operator','robot']) if(fresh.settings[id]!==undefined) $(id).value=fresh.settings[id];
      if(fresh.remote?.config){$('ssh-destination').value=fresh.remote.config.destination;$('ssh-python').value=fresh.remote.config.python;$('ssh-port').value=fresh.remote.config.port||'';}
      initialized=true;
    }
    if(offline) error('');
    offline=false;
    state=fresh;
    if(!selected || (!state.runs.some(r=>r.id===selected)&&selected!==state.active)) selected=state.active||state.runs[0]?.id||'';
    if(state.error) error(state.error);
    render();
  } catch(e) { offline=true; error(e.message==='Failed to fetch'?'The local app is unreachable. Keep its terminal open and reopen the link printed there. The last screen may be out of date.':e.message); $('start').disabled=true; $('check').disabled=true; $('stop').disabled=true; }
  finally {pollBusy=false;}
}
$('check').onclick=()=>action(()=>api('/api/check',config()));
$('run-form').onsubmit=e=>{e.preventDefault();action(async()=>{const result=await api('/api/start',{...config(),name:$('name').value,policy:$('policy').value,operator:$('operator').value,robot:$('robot').value,notes:$('notes').value});selected=result.run;downloading=null;localStorage.setItem('g1-selected',selected);$('run-log').textContent='';});};
$('stop').onclick=()=>action(()=>api('/api/stop',{run:selected}));
$('label-form').onsubmit=e=>{e.preventDefault();action(async()=>{if(!$('operator').value.trim())throw new Error('Enter the operator name on the left before saving an outcome.');await api('/api/label',{run:selected,trial:$('trial').value,outcome:$('outcome').value,notes:$('trial-notes').value,operator:$('operator').value});downloading=null;toast('Outcome saved. Include failed and interrupted attempts too.');const n=Number($('trial').value);if(Number.isInteger(n))$('trial').value=String(n+1).padStart(3,'0');$('trial-notes').value='';});};
$('export').onclick=()=>action(async()=>{downloading=null;await api('/api/export',{run:selected});});
$('show-log').onclick=()=>action(async()=>{$('run-log').textContent=(await api('/api/log/'+selected)).text||'No recorder log is available.';});
$('recover').onclick=()=>{if(confirm('Have the recorder and every writer to this run stopped? Recovery marks it incomplete and preserves its raw data.'))action(()=>api('/api/recover',{run:selected,confirmed:true}));};
$('files').onchange=async()=>{
  const files=Array.from($('files').files), run=selected;
  if(!files.length)return;
  uploadBusy=true;error('');downloading=null;render();
  try {for(let i=0;i<files.length;i++) {const f=files[i];$('upload-status').textContent='Adding '+(i+1)+' of '+files.length+': '+f.name;const r=await fetch('/api/upload?run='+encodeURIComponent(run)+'&name='+encodeURIComponent(f.name),{method:'POST',headers:{'X-G1-Token':token,'Content-Type':'application/octet-stream'},body:f});const result=await r.json();if(!r.ok)throw new Error(result.error);}toast('Files added. Prepare a new return ZIP to include them.');}
  catch(e){error(e.message);}
  finally{uploadBusy=false;$('files').value='';$('upload-status').textContent='Choose completed files only. Files stay on the recorder computer and are included in the return ZIP.';await refresh();render();}
};

function remoteConfig(){return {destination:$('ssh-destination').value.trim(),python:$('ssh-python').value.trim()||'python3',port:$('ssh-port').value};}
function renderRemote(){
  if(state.onboard)viewMode='local';
  $('storage-help').textContent=viewMode==='remote'?'Remote runs stay on the G1 computer. Download the return ZIP to keep a copy on your laptop.':'Runs are saved on the recorder computer. Refreshing or closing this page does not stop recording.';
  visible('location-choice',!state.onboard);visible('remote-panel',viewMode==='remote');visible('local-workspace',viewMode!=='remote');visible('local-saved',viewMode!=='remote');
  $('show-local').classList.toggle('selected',viewMode==='local');$('show-remote').classList.toggle('selected',viewMode==='remote');
  const r=state.remote||{}, c=remoteConfig(), working=busy||uploadBusy||offline||state.job?.state==='working';
  const matches=r.checked&&r.config?.destination===c.destination&&r.config?.python===c.python&&(r.config?.port||'')===(c.port?Number(c.port):'');
  const disabled=working||state.demo||!!state.active;
  $('ssh-check').disabled=disabled;$('ssh-install').disabled=disabled||!matches||!!r.url;$('ssh-transfer').disabled=disabled||!matches||!$('ssh-policy-file').files.length;$('ssh-open').disabled=disabled||!matches||!r.release||!r.info?.sdk;
  $('ssh-close').disabled=working;visible('ssh-close',!!r.url);visible('ssh-link',!!r.url);
  if(r.url)$('ssh-link').href=r.url;
  $('ssh-command').textContent=/^(?:[A-Za-z0-9_][A-Za-z0-9_.-]*@)?[A-Za-z0-9][A-Za-z0-9_.-]*$/.test(c.destination)?'ssh '+(c.port&&/^\d+$/.test(c.port)?'-p '+c.port+' ':'')+c.destination:'ssh YOUR_USER@YOUR_G1_COMPUTER';
  $('ssh-status').textContent=state.demo?'SSH operations are disabled in demo mode. Reopen without --demo to use your real SSH connection.':working&&state.job?.name?state.job.name+'…':r.disconnected?'The remote recorder connection ended. Recheck SSH access and reopen it; review any unfinished run.':matches?'SSH connected to '+(r.info?.hostname||c.destination)+'. '+(r.info?.sdk?'SDK imports passed. Live robot data still needs its own check.':'SDK could not import: select its existing Python environment and check again.'):'Enter the connection they already use, then check SSH access.';
  $('ssh-install-status').textContent=r.release?'Recorder copied and verified: '+r.release.path:'';
  $('ssh-policy-status').textContent=r.policy?'Verified '+r.policy.name+'\nSaved on G1: '+r.policy.path+'\nSHA-256: '+r.policy.sha256+'\nGive this path to the policy operator; include it and the checksum in the run notes.':'Choose a downloaded policy file/archive. Transfer does not install or execute it.';
  $('ssh-details').textContent=[r.info?.detail,r.logs].filter(Boolean).join('\n');
}
$('show-local').onclick=()=>{viewMode='local';sessionStorage.setItem('g1-view',viewMode);render();};
$('show-remote').onclick=()=>{viewMode='remote';sessionStorage.setItem('g1-view',viewMode);render();};
['ssh-destination','ssh-python','ssh-port','ssh-policy-file'].forEach(id=>$(id).addEventListener('input',()=>state&&renderRemote()));
$('ssh-check').onclick=()=>action(()=>api('/api/remote',{action:'check',...remoteConfig()}));
$('ssh-install').onclick=()=>action(()=>api('/api/remote',{action:'install'}));
$('ssh-open').onclick=()=>action(()=>api('/api/remote',{action:'open'}));
$('ssh-close').onclick=()=>{if(confirm('Finish and save the run in the robot tab first. Close only the recorder connection now? This does not stop any policy.'))action(()=>api('/api/remote',{action:'close'}));};
$('ssh-transfer').onclick=async()=>{
  const file=$('ssh-policy-file').files[0];if(!file)return;
  uploadBusy=true;error('');render();
  try{const response=await fetch('/api/remote-upload?name='+encodeURIComponent(file.name),{method:'POST',headers:{'X-G1-Token':token,'Content-Type':'application/octet-stream'},body:file});const result=await response.json();if(!response.ok)throw new Error(result.error);toast('File received locally. Wait for the verified G1 path before using it.');}
  catch(e){error(e.message);}
  finally{uploadBusy=false;await refresh();render();}
};

if(!token) error('Open the full link printed in the app terminal. It gives this browser access to your local recorder.');
else {refresh();setInterval(refresh,1200);}
