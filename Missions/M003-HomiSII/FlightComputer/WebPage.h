// Pre-launch page served at http://192.168.4.1 - polls /status twice a second.
#pragma once

const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>HOMIS-II Avionics</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;padding:12px;background:#111;color:#eee;max-width:520px}
h1{font-size:18px;margin:0 0 8px}
.state{font-size:28px;font-weight:700;padding:10px;border-radius:8px;text-align:center;background:#333}
.SAFE{background:#1f5f2a}.CALIBRATING{background:#7a6a10}.ARMED{background:#a11}
.BOOST,.COAST{background:#a11}.DESCENT,.LANDED{background:#1d4d8a}
.msg{margin:8px 0;min-height:1.2em;color:#fc6}
table{width:100%;border-collapse:collapse;margin:8px 0}
td{padding:4px 6px;border-bottom:1px solid #333}td:last-child{text-align:right;font-variant-numeric:tabular-nums}
.ok{color:#5d5}.bad{color:#f55}
button{font-size:16px;padding:12px;margin:4px 0;width:100%;border:0;border-radius:8px;background:#444;color:#fff}
button.arm{background:#b11}button.release{background:#a60}
button:disabled{opacity:.35}
input{width:70px;font-size:16px;padding:6px;background:#222;color:#eee;border:1px solid #555;border-radius:4px}
.row{display:flex;gap:8px;align-items:center;justify-content:space-between;margin:6px 0}
h2{font-size:15px;margin:16px 0 4px;color:#aaa}
</style></head><body>
<h1>HOMIS-II Avionics</h1>
<div id="state" class="state">...</div>
<div id="msg" class="msg"></div>

<h2>Checks</h2>
<table id="checks"></table>

<h2>Live</h2>
<table id="live"></table>

<h2>Arm</h2>
<button id="arm" class="arm" onclick="cmd('arm','ARM the flight computer? Hold the rocket still on the rail for 2 s.')">ARM</button>
<button id="disarm" onclick="cmd('disarm')">DISARM</button>

<h2>Servo test (SAFE only)</h2>
<button id="lock" onclick="cmd('lock')">Lock</button>
<button id="release" class="release" onclick="cmd('release','Move servo to RELEASE? The parachute will come out if it is loaded.')">Release</button>

<h2>Settings (SAFE only, saved on the board)</h2>
<div class="row">Backup deploy timer, s after launch<input id="backup" type="number" step="0.1"></div>
<div class="row">Servo lock angle, deg<input id="lockDeg" type="number" step="1"></div>
<div class="row">Servo release angle, deg<input id="releaseDeg" type="number" step="1"></div>
<button onclick="save()">Save settings</button>
<p style="color:#888;font-size:13px">Backup timer = OpenRocket apogee time + about 2 s.
Arming is refused until it is set.</p>

<script>
let s={},filled=false;
const $=id=>document.getElementById(id);
function ok(b){return b?'<span class="ok">OK</span>':'<span class="bad">FAIL</span>'}
function rows(r){return r.map(x=>'<tr><td>'+x[0]+'</td><td>'+x[1]+'</td></tr>').join('')}
async function poll(){
  try{
    s=await (await fetch('/status')).json();
    $('state').textContent=s.state+(s.state=='DESCENT'?' ('+s.deployReason+')':'');
    $('state').className='state '+s.state;
    $('msg').textContent=(s.restored?'[resumed after reset] ':'')+s.msg;
    const still=s.g>0.9&&s.g<1.1;
    $('checks').innerHTML=rows([
      ['MPU6050 accel/gyro',ok(s.mpuOk&&s.mpuLive)+' ('+s.mpuErrs+' errors)'],
      ['Accel reads 1 g at rest',ok(still)+' '+s.g.toFixed(3)+' g'],
      ['BMP180 barometer',ok(s.bmpOk&&s.bmpLive)+' ('+s.bmpErrs+' errors)'],
      ['SD card',ok(s.sdOk)+' '+s.run],
      ['Camera',ok(s.camOk)+' '+s.photos+' photos'],
      ['Servo',s.servoReleased?'<span class="bad">RELEASED</span>':'<span class="ok">LOCKED</span>'],
      ['Backup timer',s.backupS>0?'<span class="ok">'+s.backupS.toFixed(1)+' s</span>':'<span class="bad">not set</span>'],
      ['Log drops',s.logDrops==0?'<span class="ok">0</span>':'<span class="bad">'+s.logDrops+'</span>']]);
    $('live').innerHTML=rows([
      ['Altitude (filtered)',s.alt.toFixed(2)+' m'],
      ['Altitude (baro)',s.hBaro.toFixed(2)+' m'],
      ['Vertical velocity',s.vel.toFixed(2)+' m/s'],
      ['Vertical accel',s.aVert.toFixed(2)+' m/s²'],
      ['Gyro rate',s.gyroDps.toFixed(1)+' °/s'],
      ['Pressure / temp',s.pressPa.toFixed(0)+' Pa / '+s.tempC.toFixed(1)+' °C'],
      ['Altitude above sea level',s.absAlt.toFixed(0)+' m'],
      ['Max altitude',s.maxAlt.toFixed(1)+' m'],
      ['Flight time',s.t.toFixed(2)+' s'],
      ['Uptime',s.uptime+' s']]);
    const safe=s.state=='SAFE';
    $('arm').disabled=!safe;
    $('disarm').disabled=!['CALIBRATING','ARMED','LANDED'].includes(s.state);
    $('lock').disabled=$('release').disabled=!safe;
    if(!filled){$('backup').value=s.backupS||'';$('lockDeg').value=s.lockDeg;$('releaseDeg').value=s.releaseDeg;filled=true}
  }catch(e){$('msg').textContent='No connection to the board'}
}
async function cmd(c,confirmText){
  if(confirmText&&!confirm(confirmText))return;
  await fetch('/cmd?c='+c,{method:'POST'});
  setTimeout(poll,150);
}
async function save(){
  const p=new URLSearchParams();
  [['backup','backup'],['lock','lockDeg'],['release','releaseDeg']].forEach(([k,id])=>{if($(id).value!=='')p.set(k,$(id).value)});
  const r=await fetch('/config?'+p,{method:'POST'});
  alert(await r.text());
  filled=false;
}
poll();setInterval(poll,500);
</script></body></html>)HTML";
