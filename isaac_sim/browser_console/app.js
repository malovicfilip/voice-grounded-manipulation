'use strict';
const $ = id => document.getElementById(id);
let csrf = '', state = {}, config = {}, recording = false, requestBusy = false;
let stream, context, source, processor, muted, timer, chunks = [], samples = 0;

function statusClass(status) {
  if (['idle','awaiting_confirmation','completed'].includes(status)) return 'ok';
  if (['preparing','running','recovering','needs_human','refused','cancelled'].includes(status)) return 'warn';
  if (['stopped','faulted','error','stopping'].includes(status)) return 'bad';
  return '';
}

function chip(text) {
  const span = document.createElement('span'); span.className = 'chip'; span.textContent = text; return span;
}

function conditionLabel(condition) {
  if (condition.kind === 'object_on_target') return `complete: ${condition.object_id} on ${condition.target_id}`;
  if (condition.kind === 'object_inspected') return `complete: inspected ${condition.object_id}`;
  if (condition.kind === 'workspace_observed') return 'complete: workspace observed';
  if (condition.kind === 'named_pose_reached') return `complete: pose ${condition.pose_name}`;
  return `complete: ${condition.kind || 'condition'}`;
}

function renderReplayScene(scene) {
  const host = $('replayScene');
  host.querySelectorAll('.scene-item').forEach(node => node.remove());
  if (!scene) { host.style.display = 'none'; return false; }
  host.style.display = 'block';
  for (const target of scene.targets || []) {
    const node = document.createElement('div');
    node.className = `scene-item scene-target ${target.target_id}`;
    node.style.left = `${7 + target.u * 86}%`; node.style.top = `${10 + target.v * 80}%`;
    node.textContent = target.target_id.replace('_target',''); host.append(node);
  }
  for (const object of scene.objects || []) {
    const node = document.createElement('div');
    node.className = `scene-item ${object.object_id}` + (object.held ? ' scene-held' : '');
    node.style.left = `${7 + object.u * 86}%`; node.style.top = `${10 + object.v * 80}%`;
    node.textContent = object.object_id.replace('_cube',''); host.append(node);
  }
  return true;
}

function renderTimeline() {
  const host = $('timeline'); host.replaceChildren();
  const timeline = state.timeline || [];
  if (!timeline.length) { const empty = document.createElement('span'); empty.className='muted'; empty.textContent='No decisions yet.'; host.append(empty); return; }
  for (const event of timeline) {
    const box = document.createElement('div'); box.className = `event ${event.status || ''}`;
    const title = document.createElement('strong');
    const parts = [event.capability || event.skill, event.object_id, event.target_id ? `→ ${event.target_id}` : '', event.pose_name].filter(Boolean);
    title.textContent = parts.join(' · ') || 'step'; box.append(title);
    const info = document.createElement('small'); info.textContent = `${event.status || 'selected'}${event.reason ? ' — ' + event.reason : ''}`; box.append(info);
    host.append(box);
  }
}

function render() {
  const pending = state.status === 'awaiting_confirmation';
  const ready = csrf && !state.busy && !recording && !requestBusy;
  const preparableStates = config.control_mode === 'agent' ? ['idle','completed','refused','needs_human','cancelled','error'] : ['idle','completed','refused','cancelled','error'];
  const canPrepare = ready && preparableStates.includes(state.status);
  $('record').disabled = !canPrepare || !config.microphone_enabled;
  $('text').disabled = !canPrepare;
  $('prepare').disabled = !canPrepare;
  $('confirm').disabled = !(ready && pending && state.confirmation);
  $('cancel').disabled = !(ready && pending);
  $('stop').disabled = !csrf || state.status === 'stopping';
  $('recover').disabled = !(ready && ['stopped','faulted'].includes(state.status));
  $('confirm').textContent = config.control_mode === 'agent' ? 'Confirm mission & start agent' : 'Confirm & execute task';
  $('status').textContent = recording ? 'Recording — speak now' : (state.status || 'Connecting…');
  $('statusDot').className = `dot ${statusClass(state.status)}`;
  $('transcript').textContent = state.transcript || '—';
  $('message').textContent = state.message || '';
  $('goal').textContent = state.mission?.goal_summary || (config.control_mode === 'task' ? 'Planned task' : '—');
  $('budget').textContent = state.mission?.max_actions ? `${state.mission.max_actions} decisions maximum` : '—';

  $('scope').replaceChildren();
  if (state.mission) {
    $('scopeWrap').hidden = false;
    for (const id of state.mission.object_ids || []) $('scope').append(chip(`object: ${id}`));
    for (const id of state.mission.target_ids || []) $('scope').append(chip(`target: ${id}`));
    for (const id of state.mission.pose_names || []) $('scope').append(chip(`pose: ${id}`));
    for (const condition of state.mission.success_conditions || []) $('scope').append(chip(conditionLabel(condition)));
  } else $('scopeWrap').hidden = true;

  $('steps').replaceChildren();
  if (config.control_mode === 'task') {
    for (const step of state.steps || []) {
      const li = document.createElement('li');
      li.textContent = [step.skill?.replaceAll('_',' '), step.object_id, step.target_id ? `→ ${step.target_id}` : '', step.pose_name, step.reason].filter(Boolean).join(' · ');
      $('steps').append(li);
    }
  }
  renderTimeline();
  $('semantic').textContent = state.semantic_state ? JSON.stringify(state.semantic_state, null, 2) : 'No semantic observation yet.';
  $('details').textContent = JSON.stringify(state, null, 2);
  $('error').textContent = $('error').textContent || '';

  const replayShown = renderReplayScene(state.demo_scene);
  if (config.viewer_url) {
    $('viewer').hidden = false; $('viewerEmpty').hidden = true;
    if (!$('viewer').src) $('viewer').src = config.viewer_url;
    $('replayScene').style.display = 'none';
  } else if (replayShown) {
    $('viewer').hidden = true; $('viewerEmpty').hidden = true;
  } else {
    $('viewer').hidden = true; $('viewerEmpty').hidden = false;
  }
}

async function call(path, body = {}, audio = false) {
  const response = await fetch(path, {method:'POST', headers:{
    'Content-Type': audio ? 'audio/wav' : 'application/json', 'X-VGM-CSRF': csrf,
  }, body: audio ? body : JSON.stringify(body)});
  const value = await response.json();
  if (!response.ok) throw new Error(value.error || 'Request failed');
  state = value; render(); return value;
}

async function action(fn) {
  if (requestBusy) return;
  requestBusy = true; $('error').textContent=''; render();
  try { await fn(); } catch (error) { $('error').textContent=error.message; }
  finally { requestBusy=false; render(); }
}

async function cleanup() {
  clearTimeout(timer);
  if (stream) stream.getTracks().forEach(track => track.stop());
  if (source) source.disconnect(); if (processor) processor.disconnect(); if (muted) muted.disconnect();
  if (context && context.state !== 'closed') await context.close();
  stream=context=source=processor=muted=null;
}

function wav(rate) {
  const data=new ArrayBuffer(44+samples*2), view=new DataView(data);
  const text=(offset,value)=>[...value].forEach((c,i)=>view.setUint8(offset+i,c.charCodeAt(0)));
  text(0,'RIFF'); view.setUint32(4,36+samples*2,true); text(8,'WAVE'); text(12,'fmt ');
  view.setUint32(16,16,true); view.setUint16(20,1,true); view.setUint16(22,1,true);
  view.setUint32(24,rate,true); view.setUint32(28,rate*2,true); view.setUint16(32,2,true); view.setUint16(34,16,true);
  text(36,'data'); view.setUint32(40,samples*2,true);
  let offset=44;
  for (const chunk of chunks) for (const value of chunk) {
    const sample=Math.max(-1,Math.min(1,value)); view.setInt16(offset,Math.round(sample*(sample<0?32768:32767)),true); offset+=2;
  }
  return new Blob([data],{type:'audio/wav'});
}

$('record').onclick = async () => {
  recording=true; samples=0; chunks=[]; $('error').textContent=''; render();
  try {
    const acquired=await navigator.mediaDevices.getUserMedia({audio:true,video:false});
    if (!recording) { acquired.getTracks().forEach(track=>track.stop()); return; }
    stream=acquired; context=new AudioContext(); await context.resume(); source=context.createMediaStreamSource(stream);
    processor=context.createScriptProcessor(4096,1,1); muted=context.createGain(); muted.gain.value=0;
    processor.onaudioprocess=event=>{
      if (!recording || !context) return;
      const remaining=Math.max(0,context.sampleRate*15-samples);
      const chunk=new Float32Array(event.inputBuffer.getChannelData(0).slice(0,remaining)); chunks.push(chunk); samples+=chunk.length;
      if (samples>=context.sampleRate*15) queueMicrotask(()=>$('finish').click());
    };
    source.connect(processor); processor.connect(muted); muted.connect(context.destination);
    $('finish').disabled=false; timer=setTimeout(()=>$('finish').click(),15000);
  } catch (error) { await cleanup(); recording=false; $('error').textContent='Microphone unavailable: '+error.message; render(); }
};
$('finish').onclick = async () => {
  if (!recording || $('finish').disabled) return;
  $('finish').disabled=true; const rate=context.sampleRate; await cleanup(); const audio=wav(rate); recording=false;
  await action(()=>call('/api/audio',audio,true)); chunks=[];
};
$('prepare').onclick=()=>action(()=>call('/api/text',{transcript:$('text').value}));
$('confirm').onclick=()=>{ const confirmation=state.confirmation; action(()=>call('/api/confirm',{confirmation})); };
$('cancel').onclick=()=>action(()=>call('/api/cancel'));
$('recover').onclick=()=>{ if (window.confirm('Recovery never resumes the previous mission. Continue only if the robot is safe to recover.')) action(()=>call('/api/recover')); };
$('stop').onclick=async()=>{
  recording=false; $('finish').disabled=true;
  try { await cleanup(); chunks=[]; await call('/api/stop'); }
  catch (error) { $('error').textContent='STOP NOT CONFIRMED: '+error.message; }
  render();
};
window.addEventListener('pagehide',()=>{ if(stream) stream.getTracks().forEach(track=>track.stop()); });

async function poll() {
  try {
    if (!csrf) {
      const response=await fetch('/api/config'); if(!response.ok) throw new Error('Local bridge unavailable');
      config=await response.json(); csrf=config.csrf;
      $('session').textContent=`Session: ${config.session || 'local replay'} · dashboard bound to localhost`;
      $('modeBadge').textContent=config.control_mode==='agent'?'closed-loop agent':'planned task';
      $('backendBadge').textContent=config.backend_mode; $('connectionBadge').textContent='connected'; $('connectionBadge').className='badge live';
      $('micNote').textContent=config.microphone_enabled?'Up to 15 seconds. Audio is transcribed before any mission is proposed.':'Microphone disabled in this mode; type a command instead.';
    }
    const response=await fetch('/api/state',{headers:{'X-VGM-CSRF':csrf}});
    if(!response.ok) throw new Error('Connection lost — robot stop is not confirmed.');
    state=await response.json(); render();
  } catch (error) {
    $('error').textContent=error.message; csrf=''; $('connectionBadge').textContent='disconnected'; $('connectionBadge').className='badge'; render();
  } finally { setTimeout(poll,700); }
}
poll();
