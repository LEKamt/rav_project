#ifndef PAGINA_H
#define PAGINA_H

#include <Arduino.h>

const char PAGINA[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>F450 - Control Compensado</title>
<style>
 body{font-family:system-ui,sans-serif;margin:0;padding:12px;background:#121212;color:#eee;text-align:center}
 h1{font-size:20px;margin:4px 0 10px;color:#4fc3f7}
 h2{font-size:15px;margin:14px 0 6px;color:#aaa;text-align:left}
 #enlace{padding:8px;border-radius:8px;font-weight:bold;margin-bottom:10px}
 .ok{background:#1b5e20}.mal{background:#b71c1c}
 .datos{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;margin-bottom:8px}
 .dato{background:#1e1e1e;border-radius:8px;padding:8px;border:1px solid #333}.dato b{display:block;font-size:22px;color:#fff}
 button{font-size:20px;padding:14px;border:0;border-radius:10px;color:#fff;width:100%;margin:5px 0;cursor:pointer}
 .armar{background:#2e7d32}.parar{background:#c62828;font-size:26px;padding:20px}
 .fila{display:grid;grid-template-columns:1fr 1fr;gap:8px}
 .btn-azul{background:#1565c0}.btn-naranja{background:#ef6c00}
 canvas{width:100%;height:180px;background:#000;border-radius:8px;display:block}
 .leyenda{font-size:13px;text-align:left;margin:4px 0}
 .r{color:#ff5252}.p{color:#40c4ff}
 .gan{display:grid;grid-template-columns:90px 1fr;gap:6px;align-items:center;text-align:left}
 .gan input{font-size:18px;padding:8px;border-radius:8px;border:1px solid #444;background:#222;color:#fff;width:100%;box-sizing:border-box}
 .badge{display:inline-block;padding:2px 6px;border-radius:4px;font-size:12px;background:#333}
</style></head><body>
<h1>F450 : Control Compensado (tabla M2)</h1>
<div id="enlace" class="mal">Conectando...</div>

<div class="datos">
 <div class="dato">Roll<b id="roll">-</b></div>
 <div class="dato">Pitch<b id="pitch">-</b></div>
 <div class="dato">Acelerador<b id="thr">-</b></div>
</div>

<div id="msg" style="color:#ffb74d;min-height:20px;font-weight:bold"></div>

<div class="datos">
 <div class="dato">M1 [us]<b id="m1">-</b><span id="f1" class="badge">- g</span></div>
 <div class="dato">M2 [us]<b id="m2">-</b><span id="f2" class="badge">- g</span></div>
 <div class="dato">Lazo Hz<b id="hz">-</b></div>
 <div class="dato">M4 [us]<b id="m4">-</b><span id="f4" class="badge">- g</span></div>
 <div class="dato">M3 [us]<b id="m3">-</b><span id="f3" class="badge">- g</span></div>
 <div class="dato">ToF / US<b id="dist">-</b></div>
</div>

<canvas id="graf" width="600" height="180"></canvas>
<div class="leyenda"><span class="r">&#9632; Roll</span> &nbsp; <span class="p">&#9632; Pitch</span> &plusmn;30&deg;</div>

<div class="fila">
 <button class="armar" onclick="cmd('a')">ARMAR</button>
 <button class="btn-azul" onclick="cmd('up')">Acel. +5%</button>
</div>
<div class="fila">
 <button class="btn-azul" onclick="cmd('down')">Acel. -5%</button>
 <button class="btn-naranja" onclick="cmd('tare')">Calibrar IMU</button>
</div>
<button class="parar" onclick="cmd('s')">&#9632; PARAR MOTORES</button>
<div class="fila" style="margin-top:6px">
 <button class="btn-azul" style="font-size:16px;padding:10px" onclick="cmd('inv')">Invertir Signo Roll</button>
 <button class="btn-azul" style="font-size:16px;padding:10px" onclick="cmd('comp')">Compensaci&oacute;n ON / OFF</button>
</div>

<h2>Sinton&iacute;a PID</h2>
<div class="gan">
 <span>Kp Roll</span><input id="kpr" type="number" step="0.001" value="0.008">
 <span>Kd Roll</span><input id="kdr" type="number" step="0.0001" value="0.0015">
 <span>Kp Pitch</span><input id="kpp" type="number" step="0.001" value="0.008">
 <span>Kd Pitch</span><input id="kdp" type="number" step="0.0001" value="0.0015">
 <span>Kp Yaw</span><input id="kpy" type="number" step="0.0005" value="0.0015">
</div>
<button class="btn-naranja" onclick="enviarPid()" style="margin-top:8px">Aplicar Ganancias</button>

<h2>Compensaci&oacute;n de M2 (tabla de balanza)</h2>
<div class="datos" style="grid-template-columns:1fr 1fr">
 <div class="dato">Estado<b id="comp">-</b></div>
 <div class="dato">Tabla hasta<b id="techo">-</b></div>
</div>
<div style="font-size:13px;color:#aaa;text-align:left">La tabla se edita en el .ino (TAB_EMPUJE). Por Serial: <code>tabla</code>, <code>raw</code>, <code>nom</code>.</div>

<script>
let cv = document.getElementById('graf'), ctx = cv.getContext('2d');
let histR = [], histP = [];
let pidCargado = false;
function cmd(c){ fetch('/cmd?c=' + c).then(r=>r.text()).then(t=>{ document.getElementById('msg').innerText = t; }); }
function enviarPid(){
  let p = new URLSearchParams({
    kpr: document.getElementById('kpr').value,
    kdr: document.getElementById('kdr').value,
    kpp: document.getElementById('kpp').value,
    kdp: document.getElementById('kdp').value,
    kpy: document.getElementById('kpy').value
  });
  fetch('/pid?' + p.toString()).then(r=>r.text()).then(t=>{ document.getElementById('msg').innerText = t; });
}
function loop(){
  fetch('/estado').then(r=>r.json()).then(d=>{
    document.getElementById('enlace').className = 'ok';
    document.getElementById('enlace').innerText = d.activo ? 'EN VUELO / ACTIVO' : 'MOTORES EN REPOSO';
    document.getElementById('roll').innerText = d.roll.toFixed(1) + '°';
    document.getElementById('pitch').innerText = d.pitch.toFixed(1) + '°';
    document.getElementById('thr').innerText = (d.thr * 100).toFixed(0) + '%';
    document.getElementById('m1').innerText = d.m[0];
    document.getElementById('m2').innerText = d.m[1];
    document.getElementById('m3').innerText = d.m[2];
    document.getElementById('m4').innerText = d.m[3];
    document.getElementById('f1').innerText = d.f[0].toFixed(0) + ' g';
    document.getElementById('f2').innerText = d.f[1].toFixed(0) + ' g';
    document.getElementById('f3').innerText = d.f[2].toFixed(0) + ' g';
    document.getElementById('f4').innerText = d.f[3].toFixed(0) + ' g';
    document.getElementById('hz').innerText = d.hz.toFixed(0);
    document.getElementById('dist').innerText = d.tof + ' mm';
    if(d.pid && !pidCargado){
      document.getElementById('kpr').value = d.pid[0];
      document.getElementById('kdr').value = d.pid[1];
      document.getElementById('kpp').value = d.pid[2];
      document.getElementById('kdp').value = d.pid[3];
      document.getElementById('kpy').value = d.pid[4];
      pidCargado = true;
    }
    document.getElementById('comp').innerText = d.comp ? (d.sat ? 'ACTIVA (sobre tabla)' : 'ACTIVA') : 'APAGADA';
    document.getElementById('techo').innerText = d.techo + ' us';
    
    histR.push(d.roll); histP.push(d.pitch);
    if(histR.length > cv.width) { histR.shift(); histP.shift(); }
    dibujar();
  }).catch(()=>{
    document.getElementById('enlace').className = 'mal';
    document.getElementById('enlace').innerText = 'Sin conexion WiFi';
  });
}
function dibujar(){
  ctx.fillStyle = '#000'; ctx.fillRect(0,0,cv.width,cv.height);
  ctx.strokeStyle = '#333'; ctx.beginPath(); ctx.moveTo(0, cv.height/2); ctx.lineTo(cv.width, cv.height/2); ctx.stroke();
  function trazar(arr, color){
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
    for(let i=0; i<arr.length; i++){
      let y = cv.height/2 - (arr[i] / 30.0) * (cv.height/2);
      if(i===0) ctx.moveTo(i, y); else ctx.lineTo(i, y);
    }
    ctx.stroke();
  }
  trazar(histR, '#ff5252'); trazar(histP, '#40c4ff');
}
setInterval(loop, 100);
</script>
</body></html>
)HTML";

#endif
