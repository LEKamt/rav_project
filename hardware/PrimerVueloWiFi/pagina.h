#ifndef PAGINA_H
#define PAGINA_H

// [WiFi] Página de control. Va en una cabecera porque el conversor de .ino a C++ de
// Arduino toma las funciones JavaScript como funciones de C++ y no compila.

#include <Arduino.h>

const char PAGINA[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>F450</title>
<style>
 body{font-family:system-ui,sans-serif;margin:0;padding:12px;background:#111;color:#eee;text-align:center}
 h1{font-size:20px;margin:4px 0 10px}
 #enlace{padding:8px;border-radius:8px;font-weight:bold;margin-bottom:10px}
 .ok{background:#1b5e20}.mal{background:#b71c1c}
 .datos{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;margin-bottom:12px}
 .dato{background:#222;border-radius:8px;padding:8px}.dato b{display:block;font-size:24px}
 button{font-size:22px;padding:16px;border:0;border-radius:10px;color:#fff;width:100%;margin:5px 0}
 .armar{background:#2e7d32}.parar{background:#c62828;font-size:28px;padding:22px}
 .fila{display:grid;grid-template-columns:1fr 1fr;gap:8px}.fila button{background:#1565c0}
 input[type=range]{width:100%;height:40px;margin-top:8px}
</style></head><body>
<h1>F450 - RAV</h1>
<div id="enlace" class="mal">Conectando...</div>
<div class="datos">
 <div class="dato">Roll<b id="roll">-</b></div>
 <div class="dato">Pitch<b id="pitch">-</b></div>
 <div class="dato">Acel. %<b id="thr">-</b></div>
</div>
<div id="estado">-</div>
<button class="armar" onclick="cmd('a')">ARMAR (15 %)</button>
<div class="fila"><button onclick="cmd('-')">- 5 %</button><button onclick="cmd('+')">+ 5 %</button></div>
<input type="range" id="barra" min="0" max="80" step="1" value="0"
 oninput="document.getElementById('obj').textContent=this.value" onchange="cmd('t',this.value/100)">
<div>Acelerador objetivo: <span id="obj">0</span> %</div>
<button class="parar" onclick="cmd('s')">PARAR</button>
<script>
function cmd(c,v){fetch('/cmd?c='+c+(v!==undefined?'&v='+v:'')).catch(()=>{});}
function leer(){
 fetch('/estado').then(r=>r.json()).then(d=>{
  roll.textContent=d.roll.toFixed(1); pitch.textContent=d.pitch.toFixed(1);
  thr.textContent=(d.thr*100).toFixed(0);
  estado.textContent=d.bajada?'BAJADA AUTOMATICA':(d.activo?'VUELO ACTIVO':'DETENIDO');
  if(!document.activeElement||document.activeElement.id!=='barra'){barra.value=Math.round(d.obj*100);obj.textContent=barra.value;}
  enlace.textContent='Enlace OK'; enlace.className='ok';
 }).catch(()=>{enlace.textContent='SIN ENLACE'; enlace.className='mal';});
}
setInterval(leer,200);
</script></body></html>)HTML";

#endif
