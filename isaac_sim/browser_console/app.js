'use strict';
const $ = id => document.getElementById(id);
let csrf = '', state = {}, recording = false, requestBusy = false;
let stream, context, source, processor, muted, timer, chunks = [], samples = 0;

function render() {
  const pending = state.status === 'awaiting_confirmation';
  const ready = csrf && !state.busy && !recording && !requestBusy;
  const canPrepare = ready && ['idle', 'completed', 'refused', 'cancelled', 'error'].includes(state.status);
  $('record').disabled = !canPrepare;
  $('text').disabled = !canPrepare;
  $('prepare').disabled = !canPrepare;
  $('confirm').disabled = !(ready && pending && state.confirmation);
  $('cancel').disabled = !(ready && pending);
  $('stop').disabled = !csrf || state.status === 'stopping';
  $('recover').disabled = !(ready && ['stopped', 'faulted'].includes(state.status));
  $('status').textContent = recording ? 'Recording — speak now (maximum 15 seconds)' : (state.status || 'Connecting…');
  $('transcript').textContent = state.transcript || '—';
  $('message').textContent = state.message || '';
  $('steps').replaceChildren();
  for (const step of state.steps || []) {
    const li = document.createElement('li');
    li.textContent = [step.skill.replaceAll('_', ' '), step.object_id, step.target_id ? '→ ' + step.target_id : '', step.pose_name, step.reason].filter(Boolean).join(' · ');
    $('steps').append(li);
  }
  $('details').textContent = JSON.stringify(state, null, 2);
}

async function call(path, body = {}, audio = false) {
  const response = await fetch(path, {method: 'POST', headers: {
    'Content-Type': audio ? 'audio/wav' : 'application/json', 'X-VGM-CSRF': csrf,
  }, body: audio ? body : JSON.stringify(body)});
  const value = await response.json();
  if (!response.ok) throw new Error(value.error || 'Request failed');
  state = value; render(); return value;
}

async function action(fn) {
  if (requestBusy) return;
  requestBusy = true; $('error').textContent = ''; render();
  try { await fn(); } catch (error) { $('error').textContent = error.message; }
  finally { requestBusy = false; render(); }
}

async function cleanup() {
  clearTimeout(timer);
  if (stream) stream.getTracks().forEach(track => track.stop());
  if (source) source.disconnect();
  if (processor) processor.disconnect();
  if (muted) muted.disconnect();
  if (context && context.state !== 'closed') await context.close();
  stream = context = source = processor = muted = null;
}

function wav(rate) {
  const data = new ArrayBuffer(44 + samples * 2), view = new DataView(data);
  const text = (offset, value) => [...value].forEach((c, i) => view.setUint8(offset + i, c.charCodeAt(0)));
  text(0, 'RIFF'); view.setUint32(4, 36 + samples * 2, true); text(8, 'WAVE'); text(12, 'fmt ');
  view.setUint32(16, 16, true); view.setUint16(20, 1, true); view.setUint16(22, 1, true);
  view.setUint32(24, rate, true); view.setUint32(28, rate * 2, true);
  view.setUint16(32, 2, true); view.setUint16(34, 16, true); text(36, 'data');
  view.setUint32(40, samples * 2, true);
  let offset = 44;
  for (const chunk of chunks) for (const value of chunk) {
    const sample = Math.max(-1, Math.min(1, value));
    view.setInt16(offset, Math.round(sample * (sample < 0 ? 32768 : 32767)), true); offset += 2;
  }
  return new Blob([data], {type: 'audio/wav'});
}

$('record').onclick = async () => {
  recording = true; samples = 0; chunks = []; $('error').textContent = ''; render();
  try {
    const acquired = await navigator.mediaDevices.getUserMedia({audio: true, video: false});
    if (!recording) { acquired.getTracks().forEach(track => track.stop()); return; }
    stream = acquired;
    context = new AudioContext(); await context.resume();
    source = context.createMediaStreamSource(stream);
    processor = context.createScriptProcessor(4096, 1, 1);
    muted = context.createGain(); muted.gain.value = 0;
    processor.onaudioprocess = event => {
      if (!recording || !context) return;
      const remaining = Math.max(0, context.sampleRate * 15 - samples);
      const chunk = new Float32Array(event.inputBuffer.getChannelData(0).slice(0, remaining));
      chunks.push(chunk); samples += chunk.length;
      if (samples >= context.sampleRate * 15) queueMicrotask(() => $('finish').click());
    };
    source.connect(processor); processor.connect(muted); muted.connect(context.destination);
    $('finish').disabled = false; timer = setTimeout(() => $('finish').click(), 15000);
  } catch (error) {
    await cleanup(); recording = false; $('error').textContent = 'Microphone unavailable: ' + error.message; render();
  }
};

$('finish').onclick = async () => {
  if (!recording || $('finish').disabled) return;
  $('finish').disabled = true;
  const rate = context.sampleRate;
  await cleanup();
  const audio = wav(rate); recording = false;
  await action(() => call('/api/audio', audio, true));
  chunks = [];
};
$('prepare').onclick = () => action(() => call('/api/text', {transcript: $('text').value}));
$('confirm').onclick = () => {
  const confirmation = state.confirmation;
  action(() => call('/api/confirm', {confirmation}));
};
$('cancel').onclick = () => action(() => call('/api/cancel'));
$('recover').onclick = () => {
  if (window.confirm('Request recovery only? The robot must be stationary and empty-handed. No previous task will resume.'))
    action(() => call('/api/recover'));
};
$('stop').onclick = async () => {
  // Stop bypasses the normal requestBusy guard so it remains responsive.
  recording = false; $('finish').disabled = true;
  try { await cleanup(); chunks = []; await call('/api/stop'); }
  catch (error) { $('error').textContent = 'STOP NOT CONFIRMED: ' + error.message; }
  render();
};
window.addEventListener('pagehide', () => { if (stream) stream.getTracks().forEach(track => track.stop()); });

async function poll() {
  try {
    if (!csrf) {
      const response = await fetch('/api/config');
      if (!response.ok) throw new Error('Local bridge unavailable');
      const config = await response.json(); csrf = config.csrf;
      $('session').textContent = 'Connected to simulator session: ' + config.session;
    }
    const response = await fetch('/api/state', {headers: {'X-VGM-CSRF': csrf}});
    if (!response.ok) throw new Error('Connection lost — reload this page. Robot stop is not confirmed.');
    state = await response.json(); render();
  } catch (error) {
    $('error').textContent = error.message; csrf = ''; render();
  } finally { setTimeout(poll, 1000); }
}
poll();
