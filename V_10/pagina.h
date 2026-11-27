// =============================================================================
//  pagina.h  -  Interfaz web  (V_4, rehecha)
//
//  QUE CAMBIO Y POR QUE
//  --------------------
//  La version anterior tenia seis pestanas y unas cuarenta casillas numericas
//  con un boton APLICAR comun. Dos problemas reales de uso:
//
//  1. "No se ve que cambios se hacen". Escribias un numero, pulsabas APLICAR y
//     la unica senal era una linea gris en la cabecera. Si el firmware
//     recortaba el valor por estar fuera de limite, no te enterabas.
//  2. "Es muy compleja". Para subir una ganancia habia que encontrarla entre
//     nueve casillas iguales, sin saber cual tocar primero.
//
//  Ahora:
//  - TRES pestanas: VOLAR (lo unico que hace falta en el aire), AJUSTAR y
//    DATOS. La de VOLAR no tiene ni un numero que haya que escribir.
//  - Las ganancias son DESLIZADORES que se aplican SOLOS al moverlos. No hay
//    boton APLICAR para ellas: mueves y el dron ya lo tiene.
//  - Cada deslizador muestra al lado el valor que el DRON confirma: se pone
//    ambar mientras va en camino y verde cuando el dron lo devuelve igual. Si
//    el firmware lo recorta por limite, lo dice con el valor real.
//  - Un aviso flotante abajo confirma cada cambio con su nombre y su valor.
//  - El boton GUARDAR se marca con un punto cuando hay cambios sin guardar.
//  - Los deslizadores se generan desde una tabla en JavaScript (GRUPOS), asi
//    que anadir un parametro nuevo es una linea, no un bloque de HTML.
//  - Los grupos estan NUMERADOS en el orden en que se sintonizan.
//
//  Lo que NO se aplica solo: el modelo fisico y el recorrido de los ESC. Esos
//  siguen con boton y solo funcionan desarmado, porque cambian el significado
//  del mando que llega a los motores.
// =============================================================================
#ifndef PAGINA_H
#define PAGINA_H

#include <Arduino.h>

const char PAGINA[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>F450-RAV</title>
<style>
*{box-sizing:border-box}
body{font-family:system-ui,-apple-system,sans-serif;margin:0;background:#0d1117;color:#e6edf3;font-size:15px}
header{position:sticky;top:0;background:#161b22;border-bottom:1px solid #30363d;padding:8px 10px;z-index:9}
.fila{display:flex;gap:6px;align-items:center;flex-wrap:wrap}
h1{font-size:15px;margin:0;flex:1;letter-spacing:.5px}
.pill{padding:3px 9px;border-radius:999px;font-size:11px;font-weight:700;white-space:nowrap}
.ok{background:#17502a;color:#7ee2a8}.mal{background:#5c1a1a;color:#ffb0b0}
.aviso{background:#5a4512;color:#ffd87a}.neutro{background:#21262d;color:#9aa4b2}
#msg{font-size:12px;color:#9aa4b2;margin-top:5px;min-height:15px}
nav{display:flex;background:#161b22;border-bottom:1px solid #30363d}
nav button{flex:1;background:none;border:0;color:#8b949e;padding:12px 6px;font-size:14px;font-weight:700;border-bottom:3px solid transparent}
nav button.act{color:#58a6ff;border-bottom-color:#58a6ff}
main{padding:10px;padding-bottom:95px}
.tab{display:none}.tab.act{display:block}
.tarj{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:10px;margin-bottom:10px}
.tarj h2{font-size:12px;margin:0 0 8px;color:#8b949e;text-transform:uppercase;letter-spacing:1px}
.tarj h2.plegable{cursor:pointer;display:flex;justify-content:space-between;align-items:center;margin:0}
.g2{display:grid;grid-template-columns:repeat(2,1fr);gap:6px}
.g3{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}
.g4{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}
.dato{background:#0d1117;border:1px solid #21262d;border-radius:8px;padding:8px;text-align:center}
.dato span{display:block;font-size:11px;color:#8b949e}
.dato b{display:block;font-size:22px;font-variant-numeric:tabular-nums}
.dato.sm b{font-size:15px}
button.b{width:100%;border:0;border-radius:9px;color:#fff;padding:13px;font-size:15px;font-weight:700}
.varmar{background:#1f7a37}.vparar{background:#b32626;font-size:19px}
.vsec{background:#21262d;color:#c9d1d9}.vaz{background:#1a5fb4}
input[type=range]{width:100%;height:30px;accent-color:#58a6ff;margin:0}
input[type=number]{width:100%;background:#0d1117;border:1px solid #30363d;color:#e6edf3;border-radius:6px;padding:7px;font-size:14px;font-variant-numeric:tabular-nums}
select{width:100%;background:#0d1117;border:1px solid #30363d;color:#e6edf3;border-radius:6px;padding:8px;font-size:14px}
label{font-size:11px;color:#8b949e;display:block;margin-bottom:2px}
.barra{height:8px;background:#21262d;border-radius:4px;overflow:hidden;margin-top:4px}
.barra i{display:block;height:100%;background:#58a6ff}
table{width:100%;border-collapse:collapse;font-size:13px}
td{padding:4px 2px;border-bottom:1px solid #21262d;font-variant-numeric:tabular-nums}
td:first-child{color:#8b949e}
td.v{text-align:right;font-weight:600}
canvas{width:100%;height:120px;display:block}
.pie{position:fixed;left:0;right:0;bottom:0;background:#161b22;border-top:1px solid #30363d;padding:8px 10px;z-index:9}
.mini{font-size:11px;color:#8b949e;line-height:1.45}
.peli{color:#ffb0b0;font-size:12px;line-height:1.4}
.ctrl{margin:9px 0}
.ctrl .cab{display:flex;justify-content:space-between;align-items:baseline;font-size:12px;margin-bottom:1px}
.ctrl .nom{color:#c9d1d9}
.ctrl .val{font-weight:700;font-variant-numeric:tabular-nums;color:#7ee2a8;transition:color .3s}
.ctrl .val.esp{color:#ffd87a}
.ctrl .val.rec{color:#ff9d9d}
.grupo{overflow:hidden}
.grupo.cerrado{display:none}
#toast{position:fixed;left:10px;right:10px;bottom:78px;padding:11px 14px;border-radius:9px;
 font-size:14px;font-weight:600;text-align:center;opacity:0;pointer-events:none;
 transition:opacity .18s;z-index:20}
#toast.ver{opacity:1}
#toast.tok{background:#17502a;color:#b9f6ca;border:1px solid #2ea043}
#toast.tmal{background:#5c1a1a;color:#ffd0d0;border:1px solid #c0392b}
.franja{display:grid;grid-template-columns:repeat(6,1fr);gap:4px;margin-top:7px}
.franja div{background:#0d1117;border:1px solid #21262d;border-radius:6px;padding:4px 2px;text-align:center}
.franja span{display:block;font-size:9px;color:#8b949e;letter-spacing:.3px}
.franja b{display:block;font-size:14px;font-variant-numeric:tabular-nums}
.franja b.mal{color:#ffb0b0}
.punto{display:inline-block;width:8px;height:8px;border-radius:50%;background:#ffd87a;margin-left:6px}
</style></head><body>

<header>
 <div class="fila">
  <h1>F450 &middot; RAV</h1>
  <span id="pModo" class="pill neutro">--</span>
  <span id="pRob" class="pill neutro">NOMINAL</span>
  <span id="pEnl" class="pill mal">SIN ENLACE</span>
 </div>
 <div id="msg">conectando...</div>
 <!-- [V_5] franja de sensores: se ve en las TRES pestanas -->
 <div class="franja" id="franja">
  <div><span>roll</span><b id="fR">-</b></div>
  <div><span>pitch</span><b id="fP">-</b></div>
  <div><span>yaw</span><b id="fY">-</b></div>
  <div><span id="fAltT">altura m</span><b id="fAlt">-</b></div>
  <div><span>vz m/s</span><b id="fVz">-</b></div>
  <div><span>acel.</span><b id="fThr">-</b></div>
 </div>
</header>

<nav>
 <button class="act" onclick="tab(0,this)">VOLAR</button>
 <button onclick="tab(1,this)">AJUSTAR</button>
 <button onclick="tab(2,this)">DATOS</button>
</nav>

<main>

<!-- ============================ VOLAR ============================ -->
<section class="tab act">

 <div class="tarj">
  <div class="g3">
   <div class="dato"><span>ROLL</span><b id="vR">-</b></div>
   <div class="dato"><span>PITCH</span><b id="vP">-</b></div>
   <div class="dato"><span>YAW</span><b id="vY">-</b></div>
  </div>
  <svg viewBox="0 0 200 100" style="width:100%;height:115px;margin-top:8px;background:#0d1117;border-radius:8px">
   <defs><clipPath id="cp"><rect x="8" y="5" width="184" height="90" rx="6"/></clipPath></defs>
   <g clip-path="url(#cp)"><g id="hz">
     <rect x="-220" y="-220" width="640" height="270" fill="#1b3a5c"/>
     <rect x="-220" y="50" width="640" height="270" fill="#4a3621"/>
     <line x1="-220" y1="50" x2="420" y2="50" stroke="#e6edf3" stroke-width="1.2"/>
     <line x1="62" y1="30" x2="138" y2="30" stroke="#8b949e" stroke-width=".6"/>
     <line x1="62" y1="70" x2="138" y2="70" stroke="#8b949e" stroke-width=".6"/>
   </g></g>
   <path d="M70 50 L92 50 M108 50 L130 50 M100 46 L100 54" stroke="#ffd87a" stroke-width="2" fill="none"/>
  </svg>
 </div>

 <div class="tarj">
  <h2>Acelerador <span id="vThrTxt" style="color:#58a6ff;font-size:14px">0 %</span>
      <span class="mini" id="vHov"></span></h2>
  <input type="range" id="thr" min="0" max="85" step="1" value="0"
         oninput="thrMueve(this.value)" onchange="thrMueve(this.value)">
  <div class="barra"><i id="bThr" style="width:0%"></i></div>
  <div class="g2" style="margin-top:8px">
   <button class="b vsec" onclick="cmd('thrmenos')">&minus; 2 %</button>
   <button class="b vsec" onclick="cmd('thrmas')">+ 2 %</button>
  </div>
 </div>

 <div class="tarj">
  <h2>Altura <span id="pAlt" class="pill neutro">MANUAL</span>
      <span class="mini" id="vAltFu"></span></h2>
  <div class="g3">
   <div class="dato sm"><span>altura (m)</span><b id="aH">-</b></div>
   <div class="dato sm"><span>objetivo (m)</span><b id="aRef">-</b></div>
   <div class="dato sm"><span>vz (m/s)</span><b id="aVz">-</b></div>
  </div>
  <div class="ctrl" style="margin-top:8px">
   <div class="cab"><span class="nom">objetivo de altura</span><span class="val" id="lAlt">0.50 m</span></div>
   <input type="range" id="sAlt" min="0.15" max="2.5" step="0.05" value="0.5"
          oninput="altRef()" onchange="altRef()">
  </div>
  <div class="g2" style="margin-top:4px">
   <button class="b vaz" onclick="cmd('altmodo','&v=1')">MANTENER ALTURA</button>
   <button class="b vsec" onclick="cmd('altmodo','&v=0')">ACELERADOR MANUAL</button>
  </div>
  <div class="mini" style="margin-top:6px">Despega a mano y despues pulsa MANTENER.</div>
 </div>

 <div class="tarj">
  <h2>Referencia de actitud <span class="mini">vuelven solas al centro</span></h2>
  <div class="g2">
   <div class="ctrl"><div class="cab"><span class="nom">roll</span><span class="val" id="lR">0&deg;</span></div>
    <input type="range" id="sR" min="-25" max="25" step="1" value="0" oninput="ang()" onchange="ang()" ontouchend="centrar()" onmouseup="centrar()"></div>
   <div class="ctrl"><div class="cab"><span class="nom">pitch</span><span class="val" id="lP">0&deg;</span></div>
    <input type="range" id="sP" min="-25" max="25" step="1" value="0" oninput="ang()" onchange="ang()" ontouchend="centrar()" onmouseup="centrar()"></div>
  </div>
  <div class="ctrl"><div class="cab"><span class="nom">yaw</span><span class="val" id="lY">0 &deg;/s</span></div>
   <input type="range" id="sY" min="-60" max="60" step="1" value="0" oninput="yaw()" onchange="yaw()" ontouchend="yawCentro()" onmouseup="yawCentro()"></div>
  <div style="height:6px"></div>
  <button class="b vaz" onclick="nivelVuelo()">APRENDER ESTE NIVEL</button>
  <div class="mini" style="margin-top:6px">El giroscopio se calibra en el suelo, pero el
   suelo no esta a nivel y el centro de masa no esta centrado: la actitud a la que el
   dron se queda quieto en el aire no es la que marcaba 0 abajo. Vuela sujetando roll y
   pitch donde no se desplace y pulsa aqui: esa inclinacion pasa a ser el nuevo cero y
   las barras vuelven al centro sin que el dron se mueva.</div>
 </div>

 <div class="tarj">
  <h2 class="plegable" onclick="plega('guia')">Como volar, paso a paso <span id="fl_guia">&#9656;</span></h2>
  <div class="grupo cerrado" id="gr_guia">
   <div class="mini" style="margin:6px 0;line-height:1.6">
    <b>Sin helices</b><br>
    1. DATOS: la IMU debe decir OK y |a| ~ 1.000 quieto.<br>
    2. Inclina a la derecha: roll sube. Nariz arriba: pitch sube. Gira a la
       derecha: yaw sube. Si alguno sale al reves, parate aqui.<br>
    3. AJUSTAR &rarr; prueba de motores: cada boton mueve el motor que dice, y 1 y
       3 giran al reves que 2 y 4.<br>
    4. Comprueba que las helices que vas a poner corresponden al sentido de cada
       motor.<br>
    <b>Con helices, al aire libre</b><br>
    5. Dron en el suelo, nivelado. AJUSTAR &rarr; calibrar giroscopio y poner nivel
       actual a cero. GUARDAR.<br>
    6. ARMAR. Los motores se quedan al ralenti.<br>
    7. Sube el acelerador <b>de seguido</b> hasta que despegue. Mira la marca de
       hover: por debajo de ~20 % el dron <b>no tiene autoridad para corregir</b>,
       asi que si subes a tientas solo se inclina y se cae de lado.<br>
    8. En el aire, usa NIVEL FINO hasta que deje de irse solo. GUARDAR.<br>
    9. Solo entonces: MANTENER ALTURA, con la altura ya estable a mano.<br>
    10. Desarma y descarga el CSV en DATOS si algo fue raro.
   </div>
  </div>
 </div>

 <div class="tarj">
  <h2 class="plegable" onclick="plega('trim')">Nivel fino <span id="fl_trim">&#9662;</span></h2>
  <div class="grupo" id="gr_trim">
   <div class="mini" style="margin:6px 0">Volando, si el dron se va solo hacia un
    lado, pulsa el boton del lado CONTRARIO al que se va. Cada toque inclina
    0.2&deg; el punto que el dron considera nivelado.</div>
   <div class="g4">
    <button class="b vsec" onclick="trim('trimri')">&#9664; izq</button>
    <button class="b vsec" onclick="trim('trimrd')">der &#9654;</button>
    <button class="b vsec" onclick="trim('trimpa')">&#9650; nariz</button>
    <button class="b vsec" onclick="trim('trimpb')">&#9660; nariz</button>
   </div>
   <div class="g2" style="margin-top:6px">
    <div class="dato sm"><span>nivel roll</span><b id="tR">-</b></div>
    <div class="dato sm"><span>nivel pitch</span><b id="tP">-</b></div>
   </div>
   <div class="mini" style="margin-top:6px">No puede hacerse solo: el dron sabe su
    actitud, pero no tiene ningun sensor que le diga si esa actitud lo esta dejando
    quieto o llevandolo hacia la pared. Eso solo lo ve quien mira. Cuando quede
    bien, GUARDAR EN MEMORIA en AJUSTAR.</div>
  </div>
 </div>

 <div class="tarj">
  <h2>Motores <span class="mini" id="vSat"></span></h2>
  <div class="g2">
   <div class="dato sm"><span>1 &middot; adelante-izq</span><b id="m0">-</b><div class="barra"><i id="bm0"></i></div></div>
   <div class="dato sm"><span>2 &middot; adelante-der</span><b id="m1">-</b><div class="barra"><i id="bm1"></i></div></div>
   <div class="dato sm"><span>4 &middot; atras-izq</span><b id="m3">-</b><div class="barra"><i id="bm3"></i></div></div>
   <div class="dato sm"><span>3 &middot; atras-der</span><b id="m2">-</b><div class="barra"><i id="bm2"></i></div></div>
  </div>
 </div>
</section>

<!-- ============================ AJUSTAR ============================ -->
<section class="tab">

 <div class="tarj">
  <div class="mini">Los deslizadores se aplican <b>al momento</b>: no hay boton.
   El numero de la derecha es el valor que el <b>dron confirma</b>; se pone
   ambar mientras va en camino y rojo si el firmware lo recorta por limite.</div>
  <div class="g2" style="margin-top:8px">
   <button class="b vsec" id="bGuardar" onclick="guardar()">GUARDAR EN MEMORIA</button>
   <button class="b vsec" onclick="fabrica()">VALORES DE FABRICA</button>
  </div>
 </div>

 <div class="tarj">
  <h2>0 - Adaptacion al dron real <span id="pAdap" class="pill neutro">MANUAL</span></h2>
  <div class="mini">El dron mide, en vuelo, cuanta aceleracion angular consigue de
   verdad por cada una que pide. <b>g = 1</b> significa que el modelo acierta;
   <b>g &lt; 1</b> que falta autoridad; <b>g negativa</b> que ese eje tiene el
   <b>signo invertido</b> y ninguna ganancia lo va a arreglar.</div>
  <table style="margin-top:8px">
   <tr><td>g roll (p)</td><td class="v" id="gP">-</td></tr>
   <tr><td>g pitch (q)</td><td class="v" id="gQ">-</td></tr>
   <tr><td>g yaw (r)</td><td class="v" id="gR">-</td></tr>
   <tr><td>kp que se esta usando</td><td class="v" id="gKp">-</td></tr>
  </table>
  <div class="g2" style="margin-top:8px">
   <button class="b vaz" onclick="adap(1)">ADAPTATIVO</button>
   <button class="b vsec" onclick="adap(0)">GANANCIAS FIJAS</button>
  </div>
  <div style="height:7px"></div>
  <button class="b vsec" onclick="fijarRecomendadas()">COPIAR LAS GANANCIAS MEDIDAS A LAS FIJAS</button>
  <div style="height:7px"></div>
  <div class="g4">
   <button class="b vsec" onclick="excitar(0)">EXC p</button>
   <button class="b vsec" onclick="excitar(1)">EXC q</button>
   <button class="b vsec" onclick="excitar(2)">EXC r</button>
   <button class="b vsec" onclick="olvidar()">OLVIDAR</button>
  </div>
  <div class="mini" style="margin-top:6px">En modo ADAPTATIVO no fijas la ganancia sino
   el <b>tiempo de respuesta</b> (grupo 1): la ganancia sale de kp = 1/(tau x g) con la g
   medida, asi que el comportamiento se mantiene aunque cambie la bateria o las helices.
   EXC mueve el dron a proposito unos segundos para medir mejor: solo en vuelo y con
   espacio alrededor.</div>
 </div>

 <div class="tarj">
  <h2>Ley de control</h2>
  <div class="g2">
   <div class="dato sm"><span>ley</span><b id="rModo">-</b></div>
   <div class="dato sm"><span>error del modelo</span><b id="rErr">-</b></div>
  </div>
  <div class="g2" style="margin-top:6px">
   <button class="b vaz" onclick="rob(1)">ROBUSTO</button>
   <button class="b vsec" onclick="rob(0)">NOMINAL</button>
  </div>
 </div>

 <div id="ajustes"></div>

 <div class="tarj">
  <h2 class="plegable" onclick="plega('cal')">Calibracion <span id="fl_cal">&#9662;</span></h2>
  <div class="grupo" id="gr_cal">
   <div class="mini" style="margin:6px 0">Dron desarmado y quieto.</div>
   <button class="b vsec" onclick="cmd('calgiro');avisa('Calibrando giroscopio: NO MOVER 2 s')">CALIBRAR GIROSCOPIO</button>
   <div style="height:7px"></div>
   <button class="b vsec" onclick="cmd('nivelar');avisa('Nivel actual puesto a cero')">PONER NIVEL ACTUAL A CERO</button>
   <div style="height:7px"></div>
   <button class="b vsec" onclick="cmd('yawcero');avisa('Rumbo actual puesto a cero')">FIJAR RUMBO ACTUAL A 0</button>
   <div style="height:7px"></div>
   <button class="b vsec" onclick="cmd('reseti');avisa('Integradores a cero')">PONER INTEGRADORES A CERO</button>
  </div>
 </div>

 <div class="tarj">
  <h2 class="plegable" onclick="plega('mot')">Prueba de motores <span id="fl_mot">&#9662;</span></h2>
  <div class="grupo" id="gr_mot">
   <div class="peli" style="margin-top:6px">PELIGRO: quita las helices. Comprueba que
    cada boton mueve el motor que dice (1 adelante-izq, 2 adelante-der, 3 atras-der,
    4 atras-izq) y que 1 y 3 giran al reves que 2 y 4.</div>
   <label style="margin-top:8px"><input type="checkbox" id="sinHel"> Confirmo que las helices estan FUERA</label>
   <div class="ctrl"><div class="cab"><span class="nom">potencia de prueba</span><span class="val" id="lPot">15 %</span></div>
    <input type="range" id="pot" min="5" max="45" step="1" value="15" oninput="lPot.textContent=this.value+' %'"></div>
   <div class="g4">
    <button class="b vsec" onpointerdown="probar(0)" onpointerup="pararPrueba()" onpointerleave="pararPrueba()">M1</button>
    <button class="b vsec" onpointerdown="probar(1)" onpointerup="pararPrueba()" onpointerleave="pararPrueba()">M2</button>
    <button class="b vsec" onpointerdown="probar(2)" onpointerup="pararPrueba()" onpointerleave="pararPrueba()">M3</button>
    <button class="b vsec" onpointerdown="probar(3)" onpointerup="pararPrueba()" onpointerleave="pararPrueba()">M4</button>
   </div>
   <div class="mini" style="margin-top:6px">Manten pulsado. Se para sola en 1.5 s.</div>
  </div>
 </div>

 <div class="tarj">
  <h2 class="plegable" onclick="plega('mod')">Modelo fisico y ESC <span id="fl_mod">&#9656;</span></h2>
  <div class="grupo cerrado" id="gr_mod">
   <div class="mini" style="margin:6px 0">Esto SI tiene boton, y solo funciona
    desarmado: cambia el significado del mando que llega a los motores.</div>
   <table>
    <tr><td>empuje maximo</td><td class="v" id="qFmax">-</td></tr>
    <tr><td>empuje para sostenerse</td><td class="v" id="qFhov">-</td></tr>
    <tr><td>empuje / peso</td><td class="v" id="qTw">-</td></tr>
    <tr><td>acelerador de hover</td><td class="v" id="qThov">-</td></tr>
    <tr><td>torque disponible</td><td class="v" id="qTau">-</td></tr>
   </table>
   <div class="g3" style="margin-top:8px">
    <div><label>masa (kg)</label><input type="number" id="pm" step="0.001"></div>
    <div><label>brazo l (m)</label><input type="number" id="pl" step="0.0001"></div>
    <div><label>w max (rad/s)</label><input type="number" id="pwmax" step="10"></div>
    <div><label>Ix</label><input type="number" id="pix" step="0.0001"></div>
    <div><label>Iy</label><input type="number" id="piy" step="0.0001"></div>
    <div><label>Iz</label><input type="number" id="piz" step="0.0001"></div>
    <div style="grid-column:span 2"><label>kf</label><input type="number" id="pkf" step="1e-7"></div>
    <div><label>km</label><input type="number" id="pkm" step="1e-9"></div>
   </div>
   <div class="g4" style="margin-top:8px">
    <div><label>arr 1</label><input type="number" id="pa0" step="5"></div>
    <div><label>arr 2</label><input type="number" id="pa1" step="5"></div>
    <div><label>arr 3</label><input type="number" id="pa2" step="5"></div>
    <div><label>arr 4</label><input type="number" id="pa3" step="5"></div>
    <div><label>tope 1</label><input type="number" id="pt0" step="5"></div>
    <div><label>tope 2</label><input type="number" id="pt1" step="5"></div>
    <div><label>tope 3</label><input type="number" id="pt2" step="5"></div>
    <div><label>tope 4</label><input type="number" id="pt3" step="5"></div>
   </div>
   <div style="height:8px"></div>
   <button class="b vaz" onclick="enviarModelo()">APLICAR MODELO</button>
  </div>
 </div>
</section>

<!-- ============================ DATOS ============================ -->
<section class="tab">
 <div class="tarj"><h2>Actitud y velocidad angular</h2>
  <table>
   <tr><td>roll, pitch, yaw (&deg;)</td><td class="v" id="sAtt">-</td></tr>
   <tr><td>p, q, r medidos (&deg;/s)</td><td class="v" id="sPqr">-</td></tr>
   <tr><td>p, q, r de referencia</td><td class="v" id="sPqrd">-</td></tr>
   <tr><td>referencia r/p/y</td><td class="v" id="sSp">-</td></tr>
  </table>
 </div>

 <div class="tarj"><h2>IMU <span id="sImuOk" class="pill mal">-</span></h2>
  <table>
   <tr><td>acelerometro (g)</td><td class="v" id="sAcc">-</td></tr>
   <tr><td>modulo |a| (g)</td><td class="v" id="sAmod">-</td></tr>
   <tr><td>giroscopio (&deg;/s)</td><td class="v" id="sGyr">-</td></tr>
   <tr><td>sesgo estimado</td><td class="v" id="sBias">-</td></tr>
   <tr><td>temperatura</td><td class="v" id="sTemp">-</td></tr>
  </table>
  <div class="mini" style="margin-top:6px">Comprobacion de ejes: a la derecha &rarr; roll +,
   nariz arriba &rarr; pitch +, giro a la derecha &rarr; yaw +.</div>
 </div>

 <div class="tarj"><h2>Altura y control</h2>
  <table>
   <tr><td>ToF / ultrasonido (m)</td><td class="v" id="sDist">-</td></tr>
   <tr><td>altura usada (m)</td><td class="v" id="sAltU">-</td></tr>
   <tr><td>z, vz del Kalman</td><td class="v" id="sKz">-</td></tr>
   <tr><td>sesgo vertical (m/s&sup2;)</td><td class="v" id="sKb">-</td></tr>
   <tr><td>empuje f<sub>t</sub> (N)</td><td class="v" id="sFt">-</td></tr>
   <tr><td>pulsos a los ESC (&micro;s)</td><td class="v" id="sPul">-</td></tr>
   <tr><td>autoridad roll/pitch</td><td class="v" id="sEsc">-</td></tr>
   <tr><td>autoridad yaw</td><td class="v" id="sEscY">-</td></tr>
  </table>
 </div>

 <div class="tarj"><h2>Diagnostico de los sensores de distancia</h2>
  <div class="mini">Si arriba pone SIN ALTURA, aqui se ve por que: el sensor no
   esta conectado, contesta pero fuera de rango, o contesta bien y lo descarta
   la fusion. Son situaciones distintas con arreglos distintos.</div>
  <table style="margin-top:8px">
   <tr><td><b>VL53L0X (ToF)</b></td><td class="v" id="dTp">-</td></tr>
   <tr><td>lecturas / descartes</td><td class="v" id="dTl">-</td></tr>
   <tr><td>ultima lectura cruda</td><td class="v" id="dTm">-</td></tr>
   <tr><td>estado del sensor</td><td class="v" id="dTs">-</td></tr>
   <tr><td><b>HC-SR04 (ultrasonido)</b></td><td class="v" id="dUe">-</td></tr>
   <tr><td>ancho del ultimo eco</td><td class="v" id="dUw">-</td></tr>
   <tr><td>distancia cruda</td><td class="v" id="dUm">-</td></tr>
   <tr><td>descartes por rango</td><td class="v" id="dUf">-</td></tr>
  </table>
  <div class="mini" id="dPista" style="margin-top:8px;color:#ffd87a"></div>
 </div>

 <div class="tarj"><h2>Sistema</h2>
  <table>
   <tr><td>lazo de control</td><td class="v" id="sHz">-</td></tr>
   <tr><td>peor ciclo</td><td class="v" id="sUmax">-</td></tr>
   <tr><td>bateria</td><td class="v" id="sBat">-</td></tr>
   <tr><td>memoria libre</td><td class="v" id="sHeap">-</td></tr>
   <tr><td>clientes WiFi</td><td class="v" id="sCli">-</td></tr>
   <tr><td>encendido hace</td><td class="v" id="sUp">-</td></tr>
  </table>
 </div>

 <div class="tarj">
  <h2>Registro de vuelo <span id="pReg" class="pill neutro">PARADO</span></h2>
  <div class="mini">Guarda en el movil TODO lo que manda el dron, a 50 muestras por
   segundo: actitud, velocidades, altura, lo que da cada sensor por separado,
   empuje y los cuatro motores. Al descargarlo sale un CSV que se puede abrir en
   cualquier sitio y mandar para analizar que paso.</div>
  <label style="margin-top:8px"><input type="checkbox" id="regAuto" checked>
   Grabar solo al ARMAR y parar al desarmar</label>
  <div class="g2" style="margin-top:8px">
   <button class="b vaz" id="bGrabar" onclick="conmutaReg()">GRABAR</button>
   <button class="b vsec" onclick="descargarCSV()">DESCARGAR CSV</button>
  </div>
  <table style="margin-top:8px">
   <tr><td>muestras guardadas</td><td class="v" id="regN">0</td></tr>
   <tr><td>duracion</td><td class="v" id="regT">0 s</td></tr>
  </table>
 </div>

 <div class="tarj"><h2>Angulos &nbsp;<span class="mini" style="color:#58a6ff">roll</span>
   <span class="mini" style="color:#7ee2a8">pitch</span>
   <span class="mini" style="color:#ffd87a">yaw</span></h2><canvas id="c1"></canvas></div>
 <div class="tarj"><h2>Velocidad angular &nbsp;<span class="mini" style="color:#58a6ff">p</span>
   <span class="mini" style="color:#7ee2a8">q</span>
   <span class="mini" style="color:#ffd87a">r</span></h2><canvas id="c2"></canvas></div>
 <div class="tarj"><h2>Motores (%)</h2><canvas id="c3"></canvas></div>
 <div class="tarj"><h2>Altura &nbsp;<span class="mini" style="color:#58a6ff">medida</span>
   <span class="mini" style="color:#ffd87a">referencia</span>
   <span class="mini" style="color:#7ee2a8">vz</span></h2><canvas id="c4"></canvas></div>
 <div class="g2">
  <div><label>escala angulos</label>
   <select id="escA" onchange="pinta()"><option>10</option><option selected>45</option><option>90</option><option>180</option></select></div>
  <div><label>escala velocidades</label>
   <select id="escV" onchange="pinta()"><option>50</option><option selected>200</option><option>400</option></select></div>
 </div>
 <div style="height:8px"></div>
 <button class="b vsec" id="bPausa" onclick="conmutaPausa()">PAUSAR</button>
</section>

</main>

<div class="pie">
 <div class="g2">
  <button class="b varmar" onclick="cmd('armar')">ARMAR</button>
  <button class="b vparar" onclick="parar()">PARAR</button>
 </div>
</div>

<div id="toast"></div>

<script>
const $=i=>document.getElementById(i);
let seq=0,pausa=false,D=null,cargado=false,sinGuardar=false,fallo_ant=0,avisoSigno=false;
let grabando=false, modoAnt=0;
const REG=[], REG_MAX=150000;   /* 150 000 muestras a 50 Hz = 50 min */
const N=300,H={r:[],p:[],y:[],wp:[],wq:[],wr:[],m:[[],[],[],[]],h:[],href:[],vz:[]};
const DEG=String.fromCharCode(176);
const MODOS=['DESARMADO','ARMADO','VUELO','BAJADA','PRUEBA'];
const FALLOS=['','inclinacion','IMU','enlace','usuario','sin medida de altura'];

/* ---------------- aviso flotante ---------------- */
let tToast;
function avisa(t,bien){
 const e=$('toast');
 e.textContent=t;
 e.className='ver '+(bien===false?'tmal':'tok');
 clearTimeout(tToast);
 tToast=setTimeout(function(){e.className='';},2300);
}

/* ---------------- navegacion ---------------- */
function tab(i,b){
 document.querySelectorAll('.tab').forEach(function(s,k){s.classList.toggle('act',k==i);});
 document.querySelectorAll('nav button').forEach(function(x){x.classList.remove('act');});
 b.classList.add('act');
 if(i==2)pinta();
}
function plega(k){
 const g=$('gr_'+k),f=$('fl_'+k);
 const c=g.classList.toggle('cerrado');
 f.innerHTML=c?'&#9656;':'&#9662;';
}
/* ================= REGISTRO DE VUELO (caja negra) =================
   Las muestras llegan ya a 50 Hz dentro de la telemetria: el dron las escribe
   en un buffer circular y la pagina se lleva las nuevas en cada refresco. Aqui
   solo hay que acumularlas y volcarlas a CSV. No hace falta tarjeta SD ni
   escribir en la flash del ESP32.                                            */
function conmutaReg(){
 grabando=!grabando;
 if(grabando){REG.length=0;avisa('Grabando');}
 else avisa('Grabacion parada: '+REG.length+' muestras');
 pintaReg();
}
function pintaReg(){
 $('pReg').textContent=grabando?'GRABANDO':'PARADO';
 $('pReg').className='pill '+(grabando?'ok':'neutro');
 $('bGrabar').textContent=grabando?'PARAR':'GRABAR';
 $('regN').textContent=REG.length;
 $('regT').textContent=(REG.length/50).toFixed(1)+' s';
}
function meteReg(d){
 if(!grabando)return;
 const g=d.g, L=g.r.length;
 if(!L)return;
 const s0=d.seq-L;               /* las muestras son seq-L .. seq-1 */
 for(let i=0;i<L;i++){
  const e=g.est[i];
  REG.push([
   ((s0+i)/50).toFixed(2),
   e&7, (e>>3)&3, (e>>5)&1, (e>>6)&1,
   (g.r[i]/10).toFixed(1),(g.p[i]/10).toFixed(1),(g.y[i]/10).toFixed(1),
   (g.wp[i]/10).toFixed(1),(g.wq[i]/10).toFixed(1),(g.wr[i]/10).toFixed(1),
   (g.h[i]/100).toFixed(3),(g.href[i]/100).toFixed(3),(g.vz[i]/100).toFixed(3),
   (g.ft[i]/100).toFixed(2),(g.thr[i]/255).toFixed(3),
   g.tof[i]<0?'':(g.tof[i]/100).toFixed(3),
   g.us[i] <0?'':(g.us[i] /100).toFixed(3),
   (g.m[0][i]/2.55).toFixed(1),(g.m[1][i]/2.55).toFixed(1),
   (g.m[2][i]/2.55).toFixed(1),(g.m[3][i]/2.55).toFixed(1)
  ]);
 }
 if(REG.length>REG_MAX)REG.splice(0,REG.length-REG_MAX);
}
function descargarCSV(){
 if(!REG.length){avisa('No hay nada grabado',false);return;}
 const cab='t_s,modo,alt_modo,alt_ok,saturado,roll_deg,pitch_deg,yaw_deg,'+
   'p_dps,q_dps,r_dps,alt_m,alt_ref_m,vz_ms,ft_N,thr,tof_m,us_m,m1_pc,m2_pc,m3_pc,m4_pc';
 const filas=REG.map(function(f){return f.join(',');});
 const txt=cab+'\n'+filas.join('\n')+'\n';
 const b=new Blob([txt],{type:'text/csv'});
 const u=URL.createObjectURL(b);
 const a=document.createElement('a');
 const d=new Date();
 const dos=function(n){return (n<10?'0':'')+n;};
 a.href=u;
 a.download='vuelo_'+d.getFullYear()+dos(d.getMonth()+1)+dos(d.getDate())+'_'+
            dos(d.getHours())+dos(d.getMinutes())+dos(d.getSeconds())+'.csv';
 document.body.appendChild(a); a.click();
 setTimeout(function(){URL.revokeObjectURL(u);a.remove();},1000);
 avisa('CSV descargado: '+REG.length+' muestras');
}

function conmutaPausa(){pausa=!pausa;$('bPausa').textContent=pausa?'REANUDAR':'PAUSAR';}

/* ---------------- envio con limite de ritmo ---------------- */
const env={};
function manda(k,url){
 const e=env[k]||(env[k]={t:0,id:null});
 const falta=70-(Date.now()-e.t);
 const ir=function(){e.t=Date.now();fetch(url).catch(function(){});};
 clearTimeout(e.id);
 if(falta<=0)ir(); else e.id=setTimeout(ir,falta);
}
function cmd(c,extra){fetch('/cmd?c='+c+(extra||'')).catch(function(){});}

/* ================= TABLA DE AJUSTES =================
   [clave, nombre, min, max, paso, decimales, lector del estado del dron]
   La clave es la que espera /pid en el firmware.                        */
const GRUPOS=[
 {k:'est', t:'1 - Estabilidad (velocidad angular)', abierto:true,
  d:'En ADAPTATIVO mandan los dos tiempos de respuesta y las kp salen solas. En GANANCIAS FIJAS mandan las kp: sube hasta que oscile y baja a la mitad.',
  c:[['taur','tiempo de respuesta interno (s)',0.02,0.6,0.005,3,function(a){return a.taur;}],
     ['taua','tiempo de respuesta del angulo (s)',0.02,0.6,0.005,3,function(a){return a.taua;}],
     ['kprp','kp p (roll)',0,40,0.5,2,function(a){return a.kpr[0];}],
     ['kprq','kp q (pitch)',0,40,0.5,2,function(a){return a.kpr[1];}],
     ['kprr','kp r (yaw)',0,25,0.5,2,function(a){return a.kpr[2];}]]},

 {k:'ang', t:'2 - Seguimiento de angulo', abierto:true,
  d:'Cuanta velocidad angular pide por grado de error. Son 1/s: 6 es una constante de tiempo de 170 ms.',
  c:[['kpar','kp roll',0,15,0.25,2,function(a){return a.kpa[0];}],
     ['kpap','kp pitch',0,15,0.25,2,function(a){return a.kpa[1];}],
     ['kpay','kp yaw',0,10,0.25,2,function(a){return a.kpa[2];}]]},

 {k:'alt', t:'3 - Altura',
  d:'kp y kd fijan la respuesta. ki absorbe el error de masa y de kf, y aqui si hace falta.',
  c:[['kpz','kp z',0,15,0.25,2,function(a){return a.kz[0];}],
     ['kdz','kd z',0,15,0.25,2,function(a){return a.kz[1];}],
     ['kiz','ki z',0,5,0.1,2,function(a){return a.kz[2];}],
     ['vzmax','velocidad de subida (m/s)',0.05,1.5,0.05,2,function(a){return a.vzmax;}],
     ['altmax','altura maxima (m)',0.3,3.5,0.1,2,function(a){return a.altmax;}]]},

 {k:'rob', t:'4 - Control robusto',
  d:'Si la perturbacion puede moverse a L por segundo, basta k2 > L y k1 > 1.5 raiz(L). Si los motores zumban, sube la CAPA antes que bajar k2.',
  c:[['stk1p','k1 p',0,10,0.25,2,function(a){return a.stk1[0];}],
     ['stk2p','k2 p',0,25,0.5,2,function(a){return a.stk2[0];}],
     ['stphip','capa p (rad/s)',0.01,0.5,0.01,3,function(a){return a.stphi[0];}],
     ['stk1q','k1 q',0,10,0.25,2,function(a){return a.stk1[1];}],
     ['stk2q','k2 q',0,25,0.5,2,function(a){return a.stk2[1];}],
     ['stphiq','capa q (rad/s)',0.01,0.5,0.01,3,function(a){return a.stphi[1];}],
     ['stk1r','k1 r',0,10,0.25,2,function(a){return a.stk1[2];}],
     ['stk2r','k2 r',0,25,0.5,2,function(a){return a.stk2[2];}],
     ['stphir','capa r (rad/s)',0.01,0.5,0.01,3,function(a){return a.stphi[2];}],
     ['stzk1','k1 altura',0,10,0.1,2,function(a){return a.stz[0];}],
     ['stzk2','k2 altura',0,25,0.1,2,function(a){return a.stz[1];}],
     ['stzphi','capa altura (m/s)',0.01,0.5,0.01,3,function(a){return a.stz[2];}]]},

 {k:'lim', t:'5 - Limites y nivel',
  d:'Topes de la referencia y correccion del montaje de la IMU.',
  c:[['angmax','angulo maximo (deg)',2,45,1,0,function(a){return a.angmax;}],
     ['ratemax','velocidad maxima (deg/s)',20,400,10,0,function(a){return a.ratemax;}],
     ['trimr','trim roll (deg)',-10,10,0.1,2,function(a){return a.trim[0];}],
     ['trimp','trim pitch (deg)',-10,10,0.1,2,function(a){return a.trim[1];}]]},

 {k:'avz', t:'6 - Avanzado: filtros e integrales',
  d:'Mahony kp corrige la deriva con el acelerometro; ki estima el sesgo del giroscopio. Las ki de actitud solo sirven en modo NOMINAL.',
  c:[['mkp','Mahony kp',0.1,10,0.1,2,function(a){return a.mk[0];}],
     ['mki','Mahony ki',0,0.5,0.01,3,function(a){return a.mk[1];}],
     ['kalq','Kalman q acel',0.05,5,0.05,2,function(a){return a.kalq;}],
     ['kirp','ki p',0,10,0.25,2,function(a){return a.kir[0];}],
     ['kirq','ki q',0,10,0.25,2,function(a){return a.kir[1];}],
     ['kirr','ki r',0,10,0.25,2,function(a){return a.kir[2];}]]}
];

const INFO={};
function pintaAjustes(){
 let h='';
 GRUPOS.forEach(function(g){
  h+='<div class="tarj"><h2 class="plegable" onclick="plega(\''+g.k+'\')">'+g.t+
     ' <span id="fl_'+g.k+'">'+(g.abierto?'&#9662;':'&#9656;')+'</span></h2>'+
     '<div class="grupo'+(g.abierto?'':' cerrado')+'" id="gr_'+g.k+'">'+
     '<div class="mini" style="margin:6px 0">'+g.d+'</div>';
  g.c.forEach(function(c){
   INFO[c[0]]={dec:c[5],get:c[6],nom:c[1]};
   h+='<div class="ctrl"><div class="cab"><span class="nom">'+c[1]+
      '</span><span class="val" id="v_'+c[0]+'">-</span></div>'+
      '<input type="range" id="t_'+c[0]+'" min="'+c[2]+'" max="'+c[3]+
      '" step="'+c[4]+'" oninput="toca(\''+c[0]+'\')"></div>';
  });
  h+='</div></div>';
 });
 $('ajustes').innerHTML=h;
}

/* --- el usuario mueve un deslizador: se aplica solo --- */
const tocado={}, pedido={};
function toca(k){
 const v=+$('t_'+k).value, i=INFO[k];
 tocado[k]=Date.now();
 pedido[k]=v;
 const e=$('v_'+k);
 e.textContent=v.toFixed(i.dec);
 e.className='val esp';
 manda(k,'/pid?'+k+'='+v);
 sinGuardar=true;
 avisa(i.nom+' = '+v.toFixed(i.dec));
}

/* --- llega telemetria: confirmar, o avisar de que el firmware lo recorto --- */
function sincroniza(a){
 for(const k in INFO){
  const i=INFO[k], s=$('t_'+k), e=$('v_'+k);
  if(!s||!e)continue;
  const real=i.get(a);
  if(real===undefined||real===null)continue;
  if(tocado[k]&&(Date.now()-tocado[k]<700))continue;
  if(k in pedido){
   const paso=+s.step||0.01;
   if(Math.abs(real-pedido[k])>paso*0.6){
    e.textContent=real.toFixed(i.dec);
    e.className='val rec';
    avisa(i.nom+' limitado a '+real.toFixed(i.dec),false);
   }else{
    e.textContent=real.toFixed(i.dec);
    e.className='val';
   }
   delete pedido[k];
   s.value=real;
   continue;
  }
  if(document.activeElement!==s){
   s.value=real;
   e.textContent=real.toFixed(i.dec);
   e.className='val';
  }
 }
}

function guardar(){
 cmd('guardar');
 sinGuardar=false;
 avisa('Ajustes guardados en memoria');
}
function fabrica(){
 cmd('defecto');
 cargado=false;
 for(const k in pedido)delete pedido[k];
 for(const k in tocado)delete tocado[k];
 sinGuardar=true;
 avisa('Valores de fabrica cargados (sin guardar)');
}

/* ---------------- mandos de vuelo ---------------- */
function parar(){$('thr').value=0;thrMueve(0);cmd('parar');avisa('PARADO',false);}
function thrMueve(v){
 $('vThrTxt').textContent=v+' %';
 $('bThr').style.width=v+'%';
 manda('thr','/cmd?c=thr&v='+(v/100).toFixed(3));
}
function ang(){
 const r=+$('sR').value,p=+$('sP').value;
 $('lR').textContent=r+'°';$('lP').textContent=p+'°';
 manda('ang','/cmd?c=ang&r='+r+'&p='+p);
}
function centrar(){$('sR').value=0;$('sP').value=0;ang();}
function yaw(){const v=+$('sY').value;$('lY').textContent=v+' °/s';manda('yaw','/cmd?c=yaw&v='+v);}
function yawCentro(){$('sY').value=0;yaw();}
function altRef(){
 const v=(+$('sAlt').value).toFixed(2);
 $('lAlt').textContent=v+' m';
 manda('altref','/cmd?c=altref&v='+v);
}
function adap(v){cmd('adaptar','&v='+v);avisa(v?'Ganancias ADAPTATIVAS':'Ganancias fijas');}
function excitar(i){cmd('excitar','&v='+i);avisa('Excitando el eje '+'pqr'[i]+' unos segundos');}
function olvidar(){cmd('resetident');avisa('Identificacion reiniciada');}
function nivelVuelo(){
 cmd('nivelvuelo');
 $('sR').value=0;$('sP').value=0;
 $('lR').textContent='0'+DEG;$('lP').textContent='0'+DEG;
 avisa('Nivel aprendido: las barras vuelven al centro');
}
function fijarRecomendadas(){
 if(!D||!D.id)return;
 const k=D.id.kpef;
 fetch('/pid?kprp='+k[0].toFixed(2)+'&kprq='+k[1].toFixed(2)+'&kprr='+k[2].toFixed(2)).catch(function(){});
 sinGuardar=true;
 avisa('Ganancias medidas copiadas a las fijas');
}
function trim(c){cmd(c);}
function rob(v){
 cmd('robatt','&v='+v);
 setTimeout(function(){cmd('robalt','&v='+v);},60);
 avisa(v?'Control ROBUSTO activado':'Control NOMINAL');
}

/* ---------------- prueba de motores ---------------- */
let tPrueba=null;
function probar(m){
 if(!$('sinHel').checked){avisa('Marca primero la casilla de helices fuera',false);return;}
 const v=(+$('pot').value)/100;
 const env2=function(){fetch('/motor?ok=1&m='+m+'&v='+v.toFixed(3)).catch(function(){});};
 env2(); clearInterval(tPrueba); tPrueba=setInterval(env2,400);
 avisa('Motor M'+(m+1)+' al '+$('pot').value+' %');
}
function pararPrueba(){clearInterval(tPrueba);tPrueba=null;fetch('/motor?ok=1&m=-1&v=0').catch(function(){});}

/* ---------------- modelo fisico ---------------- */
const CMOD=[['m','pm'],['l','pl'],['wmax','pwmax'],['ix','pix'],['iy','piy'],
            ['iz','piz'],['kf','pkf'],['km','pkm'],
            ['pa0','pa0'],['pa1','pa1'],['pa2','pa2'],['pa3','pa3'],
            ['pt0','pt0'],['pt1','pt1'],['pt2','pt2'],['pt3','pt3']];
function enviarModelo(){
 if(D&&D.modo!=0){avisa('Solo se puede cambiar el modelo desarmado',false);return;}
 const q=CMOD.map(function(x){return x[0]+'='+(+$(x[1]).value||0);}).join('&');
 fetch('/modelo?'+q).then(function(){cargado=false;avisa('Modelo aplicado');}).catch(function(){});
 sinGuardar=true;
}
function pon(id,v,d){const e=$(id);if(e&&e!==document.activeElement)e.value=(+v).toFixed(d);}
function rellenaModelo(a){
 const f=a.fis;
 pon('pm',f[0],4);pon('pl',f[1],4);pon('pix',f[2],5);pon('piy',f[3],5);
 pon('piz',f[4],5);pon('pkf',f[5],9);pon('pkm',f[6],11);pon('pwmax',f[7],0);
 for(let i=0;i<4;i++){pon('pa'+i,a.parr[i],0);pon('pt'+i,a.ptop[i],0);}
 $('sR').min=-a.angmax;$('sR').max=a.angmax;
 $('sP').min=-a.angmax;$('sP').max=a.angmax;
 $('sAlt').max=a.altmax;
}

/* ---------------- graficas ---------------- */
function mete(d){
 const g=d.g;
 for(let i=0;i<g.r.length;i++){
  H.r.push(g.r[i]/10);H.p.push(g.p[i]/10);H.y.push(g.y[i]/10);
  H.wp.push(g.wp[i]/10);H.wq.push(g.wq[i]/10);H.wr.push(g.wr[i]/10);
  for(let k=0;k<4;k++)H.m[k].push(g.m[k][i]*100/255);
  H.h.push(g.h[i]/100);H.href.push(g.href[i]/100);H.vz.push(g.vz[i]/100);
 }
 const cl=['r','p','y','wp','wq','wr','h','href','vz'];
 for(const k of cl)while(H[k].length>N)H[k].shift();
 for(let k=0;k<4;k++)while(H.m[k].length>N)H.m[k].shift();
}
function traza(cv,series,lim,cero){
 const dpr=window.devicePixelRatio||1;
 const w=cv.clientWidth,h=cv.clientHeight||120;
 if(cv.width!==w*dpr){cv.width=w*dpr;cv.height=h*dpr;}
 const x=cv.getContext('2d');
 x.setTransform(dpr,0,0,dpr,0,0);
 x.clearRect(0,0,w,h);
 x.strokeStyle='#21262d';x.lineWidth=1;
 for(let i=0;i<=4;i++){const yy=h*i/4;x.beginPath();x.moveTo(0,yy);x.lineTo(w,yy);x.stroke();}
 if(cero){x.strokeStyle='#30363d';x.beginPath();x.moveTo(0,h/2);x.lineTo(w,h/2);x.stroke();}
 for(const s of series){
  const d=s.d;if(!d.length)continue;
  x.strokeStyle=s.c;x.lineWidth=1.6;x.beginPath();
  for(let i=0;i<d.length;i++){
   const px=w*i/(N-1);
   const v=Math.max(-lim,Math.min(lim,d[i]));
   const py=cero?h/2-(v/lim)*(h/2-3):h-(v/lim)*(h-3);
   i?x.lineTo(px,py):x.moveTo(px,py);
  }
  x.stroke();
 }
 x.fillStyle='#484f58';x.font='10px system-ui';
 x.fillText(cero?('+'+lim):(''+lim),3,11);
 if(cero)x.fillText('-'+lim,3,h-3);
}
function pinta(){
 if(!document.querySelectorAll('.tab')[2].classList.contains('act'))return;
 const a=+$('escA').value,v=+$('escV').value;
 traza($('c1'),[{d:H.r,c:'#58a6ff'},{d:H.p,c:'#7ee2a8'},{d:H.y,c:'#ffd87a'}],a,true);
 traza($('c2'),[{d:H.wp,c:'#58a6ff'},{d:H.wq,c:'#7ee2a8'},{d:H.wr,c:'#ffd87a'}],v,true);
 traza($('c3'),[{d:H.m[0],c:'#58a6ff'},{d:H.m[1],c:'#7ee2a8'},
                {d:H.m[2],c:'#ffd87a'},{d:H.m[3],c:'#ff9d9d'}],100,false);
 traza($('c4'),[{d:H.h,c:'#58a6ff'},{d:H.href,c:'#ffd87a'},{d:H.vz,c:'#7ee2a8'}],3,true);
}

/* ---------------- refresco ---------------- */
function f1(v){return (+v).toFixed(1);}
function f2(v){return (+v).toFixed(2);}
function f3(v){return (+v).toFixed(3);}
function v3(a,d){return a.map(function(x){return (+x).toFixed(d);}).join('  ');}

function aplica(d){
 D=d;
 $('pEnl').textContent=d.enl?'ENLACE':'SIN LATIDO';
 $('pEnl').className='pill '+(d.enl?'ok':'mal');
 $('pModo').textContent=MODOS[d.modo]||'?';
 $('pModo').className='pill '+(d.modo==0?'neutro':(d.modo==3?'mal':(d.modo==4?'aviso':'ok')));
 $('msg').textContent=d.msg;
 if(d.fallo&&d.fallo!=fallo_ant)avisa('Corte por '+FALLOS[d.fallo],false);
 fallo_ant=d.fallo;

 $('vR').textContent=f1(d.att[0]);$('vP').textContent=f1(d.att[1]);$('vY').textContent=f1(d.att[2]);
 $('hz').setAttribute('transform','rotate('+(-d.att[0])+' 100 50) translate(0 '+(d.att[1]*1.2)+')');
 if($('thr')!==document.activeElement){
  const t=Math.round(d.sp[3]*100);
  if(+$('thr').value!==t){$('thr').value=t;$('vThrTxt').textContent=t+' %';}
 }
 $('bThr').style.width=(d.thr*100).toFixed(0)+'%';
 $('vHov').textContent='hover ~'+(d.mod.thov*100).toFixed(0)+' %';
 $('vSat').textContent=d.mot.sat<0.999?('autoridad al '+(d.mot.sat*100).toFixed(0)+' %'):'';
 $('vSat').style.color=d.mot.sat<0.8?'#ffb0b0':'#8b949e';
 for(let i=0;i<4;i++){
  $('m'+i).textContent=d.mot.us[i];
  $('bm'+i).style.width=(d.mot.n[i]*100).toFixed(0)+'%';
 }

 /* --- franja de sensores, visible en las tres pestanas --- */
 $('fR').textContent=f1(d.att[0]);
 $('fP').textContent=f1(d.att[1]);
 $('fY').textContent=f1(d.att[2]);
 $('fAlt').textContent=d.alt2.val?f2(d.alt2.h):'--';
 $('fAlt').className=d.alt2.val?'':'mal';
 $('fAltT').textContent=d.alt2.val?(d.alt2.fuente==1?'alt m (ToF)':'alt m (ultra)'):'sin altura';
 $('fVz').textContent=d.alt2.val?f2(d.alt2.vz):'--';
 $('fThr').textContent=(d.thr*100).toFixed(0)+'%';


 /* --- identificacion en linea --- */
 const I=d.id, EL=['gP','gQ','gR'];
 for(let i=0;i<3;i++){
  const fiable=(I.ok>>i)&1, malo=(I.sig>>i)&1;
  const e=$(EL[i]);
  e.textContent=I.g[i].toFixed(2)+(malo?'   SIGNO INVERTIDO':(fiable?'':'   (sin datos)'));
  e.style.color=malo?'#ffb0b0':(fiable?(Math.abs(I.g[i]-1)>0.4?'#ffd87a':'#7ee2a8'):'#8b949e');
 }
 $('gKp').textContent=v3(I.kpef,1);
 $('tR').textContent=d.aj.trim[0].toFixed(1)+DEG;
 $('tP').textContent=d.aj.trim[1].toFixed(1)+DEG;
 $('pAdap').textContent=d.aj.adap?'ADAPTATIVO':'MANUAL';
 $('pAdap').className='pill '+(d.aj.adap?'ok':'neutro');
 if(I.sig&&!avisoSigno){avisoSigno=true;avisa('SIGNO INVERTIDO en un eje: revisa IMU_SIGNO',false);}
 if(!I.sig)avisoSigno=false;

 const R=d.rob, rb=(R.att||R.alt);
 $('pRob').textContent=rb?'ROBUSTO':'NOMINAL';
 $('pRob').className='pill '+(rb?'ok':'neutro');
 $('rModo').textContent=rb?'ROBUSTO':'nominal';
 $('rErr').textContent=(R.err>0?'+':'')+f1(R.err)+' %';
 $('rErr').style.color=Math.abs(R.err)>15?'#ffb0b0':'#7ee2a8';

 const A=d.alt2, hold=(A.modo==1);
 $('pAlt').textContent=hold?'MANTENIENDO':'MANUAL';
 $('pAlt').className='pill '+(hold?'ok':'neutro');
 $('vAltFu').textContent=A.val?(A.fuente==1?'ToF':'ultrasonido'):'SIN MEDIDA';
 $('vAltFu').style.color=A.val?'#8b949e':'#ffb0b0';
 /* [V_7] Sin medida valida no se ensena un numero: un numero sin sensor que lo
    respalde es peor que un guion, porque parece un dato. */
 $('aH').textContent  =A.val?f2(A.h):'--';
 /* [V_8] La casilla y la barra muestran el OBJETIVO que puso el usuario, que
    no se mueve solo. A.ref es la rampa interna y solo va a la grafica. */
 $('aRef').textContent=f2(A.obj);
 $('aVz').textContent =A.val?f2(A.vz):'--';
 $('thr').disabled=hold;
 /* La barra solo se re-sincroniza con el objetivo real del dron (por ejemplo
    al entrar en MANTENER, que captura la altura actual), nunca con la rampa. */
 if($('sAlt')!==document.activeElement&&Math.abs(+$('sAlt').value-A.obj)>0.06){
  $('sAlt').value=A.obj.toFixed(2);$('lAlt').textContent=A.obj.toFixed(2)+' m';
 }

 $('sAtt').textContent=v3(d.att,1);
 $('sPqr').textContent=v3(d.rate,1);
 $('sPqrd').textContent=v3(d.rated,1);
 $('sSp').textContent=v3(d.sp.slice(0,3),1);
 $('sImuOk').textContent=d.imu.ok?'OK':'FALLO';
 $('sImuOk').className='pill '+(d.imu.ok?'ok':'mal');
 $('sAcc').textContent=v3(d.imu.a,3);
 $('sAmod').textContent=f3(Math.hypot(d.imu.a[0],d.imu.a[1],d.imu.a[2]));
 $('sGyr').textContent=v3(d.imu.g,2);
 $('sBias').textContent=v3(d.imu.bias,3);
 $('sTemp').textContent=f1(d.imu.temp)+' C';
 $('sDist').textContent=(d.tof.ok?f3(d.tof.m)+' m':'--')+'   '+(d.us.ok?f3(d.us.m)+' m':'--');

 /* --- [V_7] diagnostico de los sensores de distancia --- */
 const G2=d.dg;
 const EST={0:'0 ok',1:'1 sigma',2:'2 senal debil',3:'3 demasiado cerca',
            4:'4 fuera de rango',5:'5 fallo de hardware'};
 $('dTp').textContent=G2.tp?'detectado en I2C1':'NO RESPONDE';
 $('dTp').style.color=G2.tp?'#7ee2a8':'#ffb0b0';
 $('dTl').textContent=G2.tlec+'  /  '+G2.tfal;
 $('dTm').textContent=G2.tmm+' mm';
 $('dTs').textContent=EST[G2.tst]||G2.tst;
 $('dUe').textContent=G2.ue+' ecos recibidos';
 $('dUe').style.color=G2.ue>0?'#7ee2a8':'#ffb0b0';
 $('dUw').textContent=G2.uus+' us';
 $('dUm').textContent=f3(G2.um)+' m';
 $('dUf').textContent=G2.ufal;
 let pista='';
 if(!G2.tp) pista+='El VL53L0X no contesta en I2C1 (pines 4 y 5): revisa alimentacion y cableado. ';
 if(G2.ue===0) pista+='El HC-SR04 no ha devuelto ni un eco: revisa TRIG (6), ECHO (7) y que ECHO pase por un divisor, porque saca 5 V. ';
 if(G2.tp&&G2.tst==4) pista+='El ToF contesta pero fuera de rango: normal por encima de 25 cm. ';
 if(G2.ue>0&&!d.us.ok) pista+='El ultrasonido mide pero la distancia cae fuera de la banda aceptada. ';
 if(!pista&&!d.alt2.val) pista='Los sensores responden; la altura se descarta por la fusion o por el tiempo sin medida.';
 $('dPista').textContent=pista;
 $('sAltU').textContent=f3(d.alt);
 $('sKz').textContent=f3(-A.h)+'   '+f3(-A.vz);
 $('sKb').textContent=f3(A.bias);
 $('sFt').textContent=f2(d.u[3]);
 $('sPul').textContent=d.mot.us.join('  ');
 $('sEsc').textContent=(d.mot.sat*100).toFixed(0)+' %';
 $('sEsc').style.color=d.mot.sat<0.8?'#ffb0b0':'#7ee2a8';
 $('sEscY').textContent=(d.mot.saty*100).toFixed(0)+' %';
 $('sHz').textContent=d.sis.hz+' Hz';
 $('sUmax').textContent=d.sis.umax+' us';
 $('sBat').textContent=d.bat.ok?(f2(d.bat.v)+' V'):'no instalada';
 $('sHeap').textContent=(d.sis.heap/1024).toFixed(0)+' kB';
 $('sCli').textContent=d.sis.cli;
 $('sUp').textContent=Math.floor(d.sis.up/60)+' min '+(d.sis.up%60)+' s';

 $('qFmax').textContent=f2(d.mod.fmax)+' N';
 $('qFhov').textContent=f2(d.mod.fhov)+' N';
 $('qTw').textContent=f2(d.mod.tw);
 $('qThov').textContent=(d.mod.thov*100).toFixed(0)+' %';
 $('qTau').textContent=f3(d.mod.taumax)+' N.m';

 sincroniza(d.aj);
 if(!cargado){rellenaModelo(d.aj);cargado=true;}
 $('bGuardar').innerHTML='GUARDAR EN MEMORIA'+(sinGuardar?'<span class="punto"></span>':'');

 /* el registro se llena SIEMPRE, aunque las graficas esten en pausa */
 if($('regAuto').checked){
  if(modoAnt==0&&d.modo!=0&&!grabando){grabando=true;REG.length=0;avisa('Grabando (armado)');}
  if(modoAnt!=0&&d.modo==0&&grabando){grabando=false;avisa('Grabacion parada: '+REG.length+' muestras');}
 }
 modoAnt=d.modo;
 meteReg(d);
 pintaReg();

 if(!pausa){mete(d);pinta();}
}

async function tic(){
 try{
  const res=await fetch('/t?s='+seq,{cache:'no-store'});
  const d=await res.json();
  seq=d.seq;
  aplica(d);
 }catch(e){
  $('pEnl').textContent='SIN ENLACE';$('pEnl').className='pill mal';
 }
}
pintaAjustes();
setInterval(tic,100);
window.addEventListener('resize',pinta);
tic();
</script>
</body></html>)HTML";

#endif // PAGINA_H
