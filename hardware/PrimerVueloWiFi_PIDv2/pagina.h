#ifndef PAGINA_H
#define PAGINA_H

// [WiFi] Página de control + sintonía PID. Va en una cabecera porque el conversor de
// .ino a C++ de Arduino toma las funciones JavaScript como funciones de C++ y no compila.

#include <Arduino.h>

const char PAGINA[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>F450</title>
<style>
 body{font-family:system-ui,sans-serif;margin:0;padding:12px;background:#111;color:#eee;text-align:center}
 h1{font-size:20px;margin:4px 0 10px}
 h2{font-size:15px;margin:14px 0 6px;color:#aaa;text-align:left}
 #enlace{padding:8px;border-radius:8px;font-weight:bold;margin-bottom:10px}
 .ok{background:#1b5e20}.mal{background:#b71c1c}
 .datos{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;margin-bottom:8px}
 .dato{background:#222;border-radius:8px;padding:8px}.dato b{display:block;font-size:24px}
 button{font-size:22px;padding:16px;border:0;border-radius:10px;color:#fff;width:100%;margin:5px 0}
 .armar{background:#2e7d32}.parar{background:#c62828;font-size:28px;padding:22px}
 .fila{display:grid;grid-template-columns:1fr 1fr;gap:8px}.fila button{background:#1565c0}
 input[type=range]{width:100%;height:40px;margin-top:8px}
 canvas{width:100%;height:180px;background:#000;border-radius:8px;display:block}
 .leyenda{font-size:13px;text-align:left;margin:4px 0}
 .r{color:#ff5252}.p{color:#40c4ff}
 .gan{display:grid;grid-template-columns:70px 1fr;gap:6px;align-items:center;text-align:left}
 .gan input{font-size:20px;padding:8px;border-radius:8px;border:0;width:100%;box-sizing:border-box}
 .gan small{color:#888}
 .aplicar{background:#ef6c00;font-size:20px;padding:14px}
 #activas{font-size:13px;color:#aaa;text-align:left;margin-top:4px}
</style></head><body>
<h1>F450 - RAV</h1>
<div id="enlace" class="mal">Conectando...</div>

<div class="datos">
 <div class="dato">Roll<b id="roll">-</b></div>
 <div class="dato">Pitch<b id="pitch">-</b></div>
 <div class="dato">Acel. %<b id="thr">-</b></div>
</div>
<div id="estado">-</div>
<div id="msg" style="color:#ffb74d;min-height:18px"></div>
<div class="datos">
 <div class="dato">M1<b id="m1">-</b></div><div class="dato">M2<b id="m2">-</b></div>
 <div class="dato">Lazo Hz<b id="hz">-</b></div>
 <div class="dato">M4<b id="m4">-</b></div><div class="dato">M3<b id="m3">-</b></div>
 <div class="dato">Corte<b id="corte" style="font-size:13px">-</b></div>
</div>

<canvas id="graf" width="600" height="180"></canvas>
<div class="leyenda"><span class="r">&#9632; Roll</span> &nbsp; <span class="p">&#9632; Pitch</span>
 &nbsp; ultimos 10 s, escala &plusmn;<span id="esc">45</span>&deg;
 <button id="pausa" style="width:auto;font-size:14px;padding:6px 10px;margin:0 0 0 8px;background:#444" onclick="pausar()">Pausar</button>
 <button style="width:auto;font-size:14px;padding:6px 10px;margin:0 0 0 4px;background:#444" onclick="escala()">Escala</button></div>

<h2>Ganancias PID</h2>
<div class="gan">
 <span>Kp roll</span><input id="kpr" type="number" step="0.0005" inputmode="decimal">
 <span>Kd roll</span><input id="kdr" type="number" step="0.00005" inputmode="decimal">
 <span>Ki roll</span><input id="kir" type="number" step="0.0005" inputmode="decimal">
 <span>Kp pitch</span><input id="kpp" type="number" step="0.0005" inputmode="decimal">
 <span>Kd pitch</span><input id="kdp" type="number" step="0.00005" inputmode="decimal">
 <span>Ki pitch</span><input id="kip" type="number" step="0.0005" inputmode="decimal">
 <span>Kp yaw<br><small>por &deg;/s</small></span><input id="kpy" type="number" step="0.0005" inputmode="decimal">
 <span>Banda<br><small>muerta &deg;</small></span><input id="db" type="number" step="0.1" inputmode="decimal">
</div>
<button class="aplicar" onclick="aplicar()">APLICAR GANANCIAS</button>
<div id="activas">Activas en el dron: -</div>

<h2>Control</h2>
<button class="armar" onclick="cmd('a')">ARMAR (15 %) - solo si esta detenido</button>
<div class="fila"><button onclick="cmd('down')">- 5 %</button><button onclick="cmd('up')">+ 5 %</button></div>
<input type="range" id="barra" min="0" max="80" step="1" value="0"
 oninput="document.getElementById('obj').textContent=this.value" onchange="cmd('t',this.value/100)">
<div>Acelerador objetivo: <span id="obj">0</span> %</div>
<button class="parar" onclick="cmd('s')">PARAR</button>

<script>
var R=[],P=[],seq=0,primera=true,pausado=false,esc=45,NMAX=500;
function cmd(c,v){fetch('/cmd?c='+encodeURIComponent(c)+(v!==undefined?'&v='+encodeURIComponent(v):'')).then(function(r){return r.text();}).then(function(t){if(t!=='ok'){msg.textContent=t;setTimeout(function(){msg.textContent='';},3000);}}).catch(function(){});}
function aplicar(){
 var q='/pid?kpr='+kpr.value+'&kdr='+kdr.value+'&kir='+kir.value+'&kpp='+kpp.value+'&kdp='+kdp.value+'&kip='+kip.value+'&kpy='+kpy.value+'&db='+db.value;
 fetch(q).then(function(){primera=true;}).catch(function(){});
}
function pausar(){pausado=!pausado;pausa.textContent=pausado?'Seguir':'Pausar';}
function escala(){esc=(esc==45)?30:(esc==30?15:(esc==15?5:45));document.getElementById('esc').textContent=esc;dibujar();}
function trazo(ctx,a,w,h,color){
 ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();
 for(var i=0;i<a.length;i++){
  var x=w-(a.length-i)*(w/NMAX), y=h/2-(a[i]/esc)*(h/2);
  if(i==0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
 }
 ctx.stroke();
}
function dibujar(){
 var c=graf,ctx=c.getContext('2d'),w=c.width,h=c.height;
 ctx.clearRect(0,0,w,h);
 ctx.strokeStyle='#333';ctx.lineWidth=1;ctx.beginPath();
 ctx.moveTo(0,h/2);ctx.lineTo(w,h/2);
 ctx.moveTo(0,h/4);ctx.lineTo(w,h/4);ctx.moveTo(0,3*h/4);ctx.lineTo(w,3*h/4);ctx.stroke();
 ctx.fillStyle='#666';ctx.font='12px sans-serif';
 ctx.fillText('+'+esc,4,12);ctx.fillText('-'+esc,4,h-4);
 trazo(ctx,R,w,h,'#ff5252');trazo(ctx,P,w,h,'#40c4ff');
}
function leer(){
 fetch('/estado?s='+seq).then(function(r){return r.json();}).then(function(d){
  roll.textContent=d.roll.toFixed(1); pitch.textContent=d.pitch.toFixed(1);
  thr.textContent=(d.thr*100).toFixed(0);
  estado.textContent=d.bajada?'BAJADA AUTOMATICA':(d.activo?'VUELO ACTIVO':'DETENIDO');
  if(!document.activeElement||document.activeElement.id!=='barra'){barra.value=Math.round(d.obj*100);obj.textContent=barra.value;}
  m1.textContent=d.m[0];m2.textContent=d.m[1];m3.textContent=d.m[2];m4.textContent=d.m[3];
  hz.textContent=d.hz;corte.textContent=d.corte||'-';
  if(d.seq<seq){R=[];P=[];}
  seq=d.seq;
  if(!pausado){
   for(var i=0;i<d.r.length;i++){R.push(d.r[i]/100);P.push(d.p[i]/100);}
   while(R.length>NMAX){R.shift();P.shift();}
   dibujar();
  }
  activas.textContent='Activas en el dron: Kp_r='+d.k[0]+' Kd_r='+d.k[1]+' Ki_r='+d.k[2]+' Kp_p='+d.k[3]+' Kd_p='+d.k[4]+' Ki_p='+d.k[5]+' Kp_y='+d.k[7]+' db='+d.k[6];
  if(primera){kpr.value=d.k[0];kdr.value=d.k[1];kir.value=d.k[2];kpp.value=d.k[3];kdp.value=d.k[4];kip.value=d.k[5];db.value=d.k[6];kpy.value=d.k[7];primera=false;}
  enlace.textContent='Enlace OK'; enlace.className='ok';
 }).catch(function(){enlace.textContent='SIN ENLACE'; enlace.className='mal';});
}
setInterval(leer,200);
</script></body></html>)HTML";

#endif