#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <math.h>

// =============================================
// HABITABOT ESP32 v2.4
// v2.3 + simulacion + portal cautivo
// =============================================

const char* WIFI_NOMBRE = "La Casa Blanca";
const char* WIFI_CLAVE  = "whitehouse";

WebServer servidor(80);
DNSServer dnsServer;
Preferences memoria;

const byte DNS_PORT = 53;

// ============ PINES ============
const int PIN_BOMBA          = 23;
const int PIN_SERVO_ARENA    = 18;
const int PIN_RELE_ESPUMA    = 19;
const int PIN_RETORNO_AGUA   = 4;
const int PIN_RETORNO_JABON  = 16;

const int TRIG_AGUA   = 27;
const int ECHO_AGUA   = 26;
const int TRIG_ARENA  = 14;
const int ECHO_ARENA  = 35;
const int TRIG_ESPUMA = 13;
const int ECHO_ESPUMA = 36;

// ============ INDICES ============
const int AGUA = 0, ARENA = 1, ESPUMA = 2;

// ============ MEDIDAS ============
const float alturaMaxima[3]  = { 14.0f, 11.0f, 7.0f };
const float volumenMaximo[3] = { 1500.0f, 1250.0f, 850.0f };
float distanciaVacio[3] = { 17.0f, 14.0f, 10.0f };
const float TOLERANCIA_AGUA = 0.30f;

// ============ SENSORES ============
const int trigPins[3] = { TRIG_AGUA, TRIG_ARENA, TRIG_ESPUMA };
const int echoPins[3] = { ECHO_AGUA, ECHO_ARENA, ECHO_ESPUMA };

float distancia[3] = { -1, -1, -1 };
float historial[3][5] = {};
int cantidad[3] = { 0, 0, 0 };
int posicion[3] = { 0, 0, 0 };
unsigned long ultimaLectura[3] = { 0, 0, 0 };
unsigned long ultimoDisparo = 0;
int sensorActual = 0;

// ============ SIMULACION ============
bool simulacionActiva = false;
float simPorcentaje[3] = { 0.0f, 0.0f, 0.0f };  // 0-100 virtual
unsigned long ultimaSimulacion = 0;

// ============ ACTUADORES ============
Servo servoArena;
const int SERVO_CERRADO = 75;
const int SERVO_ABIERTO = 5;
const int RELE_ACTIVO = LOW;
const int RELE_INACTIVO = HIGH;

bool bombaActiva = false;
bool arenaAbierta = false;
bool espumaActiva = false;
bool retornoAguaActivo = false;
bool retornoJabonActivo = false;

unsigned long inicioBomba = 0, inicioArena = 0, inicioEspuma = 0;
unsigned long inicioRetornoAgua = 0, inicioRetornoJabon = 0;

const unsigned long LIMITE_BOMBA = 30000UL;
const unsigned long LIMITE_ARENA = 15000UL;
const unsigned long LIMITE_ESPUMA = 15000UL;
const unsigned long LIMITE_RETORNO = 45000UL;

// ============ OBJETIVOS ============
int objetivoPct[3] = { 100, 100, 100 };
float objetivoMl[3] = { 1500.0f, 1250.0f, 850.0f };
bool modoTermostato[3] = { false, false, false };
bool unidadMl[3] = { false, false, false };
const int NIVEL_VACIO = 3;

// ============ SENSORES: UTILIDADES ============
float medirCM(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(10);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long t = pulseIn(echo, HIGH, 30000UL);
  if (t == 0) return -1.0f;
  float cm = t * 0.0343f / 2.0f;
  if (cm < 2.0f || cm > 400.0f) return -1.0f;
  return cm;
}

float mediana(int s) {
  int n = cantidad[s]; if (n <= 0) return -1.0f;
  float v[5];
  for (int i = 0; i < n; i++) v[i] = historial[s][i];
  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++)
      if (v[j] < v[i]) { float t = v[i]; v[i] = v[j]; v[j] = t; }
  return v[n / 2];
}

// Avanza la simulacion segun los actuadores activos
void actualizarSimulacion() {
  unsigned long ahora = millis();
  if (ahora - ultimaSimulacion < 100UL) return;
  float dt = (ahora - ultimaSimulacion) / 1000.0f;
  ultimaSimulacion = ahora;

  // Velocidades simuladas (%/s)
  const float velLlenado = 8.0f;
  const float velRetorno = 10.0f;

  // Agua
  if (bombaActiva) simPorcentaje[AGUA] += velLlenado * dt;
  if (retornoAguaActivo) simPorcentaje[AGUA] -= velRetorno * dt;
  simPorcentaje[AGUA] = constrain(simPorcentaje[AGUA], 0.0f, 100.0f);

  // Arena
  if (arenaAbierta) simPorcentaje[ARENA] += velLlenado * 0.7f * dt;
  simPorcentaje[ARENA] = constrain(simPorcentaje[ARENA], 0.0f, 100.0f);

  // Espuma
  if (espumaActiva) simPorcentaje[ESPUMA] += velLlenado * 0.9f * dt;
  if (retornoJabonActivo) simPorcentaje[ESPUMA] -= velRetorno * dt;
  simPorcentaje[ESPUMA] = constrain(simPorcentaje[ESPUMA], 0.0f, 100.0f);
}

void actualizarSensores() {
  if (simulacionActiva) { actualizarSimulacion(); return; }

  unsigned long ahora = millis();
  if (ahora - ultimoDisparo < 100UL) return;
  ultimoDisparo = ahora;
  int i = sensorActual;
  sensorActual = (sensorActual + 1) % 3;
  float l = medirCM(trigPins[i], echoPins[i]);
  if (l > 0) {
    historial[i][posicion[i]] = l;
    posicion[i] = (posicion[i] + 1) % 5;
    if (cantidad[i] < 5) cantidad[i]++;
    distancia[i] = mediana(i);
    ultimaLectura[i] = ahora;
  }
  for (int j = 0; j < 3; j++) {
    if (distancia[j] >= 0 && ahora - ultimaLectura[j] > 3000UL) {
      distancia[j] = -1; cantidad[j] = 0; posicion[j] = 0;
    }
  }
}

// En simulacion usa simPorcentaje; en real usa sensores
float alturaEfectiva(int s) {
  if (simulacionActiva) return simPorcentaje[s] * alturaMaxima[s] / 100.0f;
  return obtenerAlturaReal(s);
}

float obtenerAlturaReal(int s) {
  if (s < 0 || s > 2 || distancia[s] < 0) return -1;
  float a = distanciaVacio[s] - distancia[s];
  if (s == AGUA && a <= TOLERANCIA_AGUA) a = 0;
  return constrain(a, 0.0f, alturaMaxima[s]);
}

float obtenerAltura(int s) { return alturaEfectiva(s); }

int obtenerPorcentaje(int s) {
  if (simulacionActiva) return constrain((int)roundf(simPorcentaje[s]), 0, 100);
  float a = obtenerAlturaReal(s); if (a < 0) return 0;
  return constrain((int)roundf(a * 100.0f / alturaMaxima[s]), 0, 100);
}

float obtenerVolumenMl(int s) {
  if (simulacionActiva) return simPorcentaje[s] * volumenMaximo[s] / 100.0f;
  float a = obtenerAlturaReal(s); if (a < 0) return -1.0f;
  return a * (volumenMaximo[s] / alturaMaxima[s]);
}

// Distancia mostrada: en simulacion se recalcula desde el porcentaje simulado
float distanciaMostrada(int s) {
  if (simulacionActiva) {
    float altura = simPorcentaje[s] * alturaMaxima[s] / 100.0f;
    return distanciaVacio[s] - altura;
  }
  return distancia[s];
}

int objetivoEfectivoPct(int s) {
  if (unidadMl[s]) return constrain((int)roundf(objetivoMl[s] * 100.0f / volumenMaximo[s]), 0, 100);
  return constrain(objetivoPct[s], 0, 100);
}
bool estaLleno(int s) {
  float a = alturaEfectiva(s);
  return a >= 0 && a >= alturaMaxima[s];
}

bool calibrarVacio(int s) {
  if (s < 0 || s > 2 || distancia[s] < 0) return false;
  distanciaVacio[s] = distancia[s];
  const char* k[3] = { "aguaVacia", "arenaVacia", "espumaVacia" };
  memoria.putFloat(k[s], distanciaVacio[s]);
  return true;
}

// ============ CONTROL ============
void controlarBomba(bool on) {
  if (on && retornoAguaActivo) return;
  bombaActiva = on;
  digitalWrite(PIN_BOMBA, on ? RELE_ACTIVO : RELE_INACTIVO);
  if (on) inicioBomba = millis();
}
void controlarArena(bool on) {
  if (!servoArena.attached()) return;
  arenaAbierta = on;
  servoArena.write(on ? SERVO_ABIERTO : SERVO_CERRADO);
  if (on) inicioArena = millis();
}
void controlarEspuma(bool on) {
  espumaActiva = on;
  digitalWrite(PIN_RELE_ESPUMA, on ? RELE_ACTIVO : RELE_INACTIVO);
  if (on) inicioEspuma = millis();
}
void controlarRetornoAgua(bool on) {
  if (on && bombaActiva) return;
  retornoAguaActivo = on;
  digitalWrite(PIN_RETORNO_AGUA, on ? RELE_ACTIVO : RELE_INACTIVO);
  if (on) inicioRetornoAgua = millis();
}
void controlarRetornoJabon(bool on) {
  if (on && espumaActiva) return;
  retornoJabonActivo = on;
  digitalWrite(PIN_RETORNO_JABON, on ? RELE_ACTIVO : RELE_INACTIVO);
  if (on) inicioRetornoJabon = millis();
}
void detenerTodo() {
  controlarBomba(false); controlarArena(false); controlarEspuma(false);
  controlarRetornoAgua(false); controlarRetornoJabon(false);
  simulacionActiva = false;
}

// ============ PAGINA WEB ============
const char PAGINA[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="es" data-theme="dark">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>HABITABOT v2.4</title>
<style>
:root{
  --bg:#07111f; --card:#142238; --card2:#0a172a; --border:#30445e;
  --text:#f1f5f9; --muted:#a8bbd3; --muted2:#b2c5dc;
  --blue:#38bdf8; --yellow:#fbbf24; --purple:#c084fc; --green:#34d399; --sim:#fb923c;
}
[data-theme="light"]{
  --bg:#eef2f7; --card:#ffffff; --card2:#f5f8fc; --border:#c7d3e0;
  --text:#0f172a; --muted:#475569; --muted2:#475569;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font-family:Arial,sans-serif}
header{padding:22px 12px;text-align:center;background:var(--card);border-bottom:1px solid var(--border)}
h1{margin:0;color:var(--blue);letter-spacing:3px}
header p{color:var(--muted);margin:6px 0 0;font-size:13px}
#conexion{padding:12px;background:var(--card2);color:var(--green);border-radius:10px;margin-bottom:12px;font-weight:bold;position:sticky;top:0;z-index:100;transition:all .3s}
#conexion.desconectado{color:#f87171;animation:parp 1s infinite}
#conexion.simulando{color:var(--sim);background:#3a1e08;animation:parpSim 1.2s infinite}
@keyframes parp{0%,100%{opacity:1}50%{opacity:.55}}
@keyframes parpSim{0%,100%{box-shadow:0 0 0 0 #fb923c00}50%{box-shadow:0 0 0 4px #fb923c66}}
#mensaje{color:var(--muted2);font-size:13px}
main{max-width:1200px;margin:auto;padding:18px}
button{border:0;border-radius:10px;padding:11px;font-weight:bold;cursor:pointer;font-size:13px;transition:all .2s}
button:disabled{opacity:.5;cursor:wait}
button:hover:not(:disabled){filter:brightness(1.12)}
button:focus-visible,input:focus,select:focus{outline:2px solid var(--blue);outline-offset:1px}
#btn-detener{background:#ef4444;color:#fff;width:100%;margin-bottom:10px}
#btn-simular{background:#fb923c;color:#0a172a;width:100%;margin-bottom:10px}
#btn-simular.activo{background:#7c2d12;color:#fed7aa;border:2px solid var(--sim)}
#btn-tema{background:#273e5d;color:#fff;margin-bottom:10px;width:100%}
#btn-simple{background:#475569;color:#fff;margin-bottom:18px;width:100%}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:18px}
.card{background:var(--card);border:1px solid var(--border);border-radius:18px;padding:18px;overflow:hidden;position:relative;transition:box-shadow .3s}
.card.agua{border-top:4px solid var(--blue)}
.card.arena{border-top:4px solid var(--yellow)}
.card.espuma{border-top:4px solid var(--purple)}
.card.alcanzado{border-color:var(--green);box-shadow:0 0 0 2px #34d39955}
.card.alcanzado::before{content:'✓ OBJETIVO';position:absolute;top:8px;right:8px;background:var(--green);color:#064e3b;padding:3px 8px;border-radius:6px;font-size:10px;font-weight:bold;z-index:20}
.visual{display:flex;justify-content:space-around;align-items:center;min-height:220px;gap:12px}
.tanque{width:120px;height:185px;flex-shrink:0;position:relative;background:var(--card2);border:3px solid #64748b;border-radius:12px 12px 20px 20px;overflow:hidden}
.tanque-espuma{clip-path:polygon(0 35%,100% 0,100% 100%,0 100%);border-radius:0}
.relleno{position:absolute;bottom:0;left:0;width:100%;height:0%;transition:height .8s ease}
.linea-objetivo{position:absolute;left:0;width:100%;height:2px;background:var(--yellow);z-index:9;box-shadow:0 0 6px var(--yellow);cursor:ns-resize;touch-action:none}
.linea-objetivo::after{content:attr(data-label);position:absolute;right:-2px;top:-16px;font-size:10px;color:var(--yellow);background:var(--bg);padding:0 4px;border-radius:4px;font-weight:bold;white-space:nowrap}
.numero{position:absolute;inset:0;z-index:10;display:grid;place-items:center;font-size:20px;font-weight:bold;text-shadow:0 2px 7px #0008}
.datos{text-align:center;flex:1}
.porcentaje{font-size:34px;font-weight:bold}
.detalle{font-size:13px;color:var(--muted2);margin-top:8px}
.spark{margin-top:10px;background:var(--card2);border-radius:8px;padding:6px}
.agua-fill{background:linear-gradient(#67e8f9,#0284c7,#075985)}
.agua-fill::before,.agua-fill::after{content:"";position:absolute;top:-10px;left:-45%;width:190%;height:22px;border-radius:50%;background:#67e8f9;animation:ola 2s ease-in-out infinite}
.agua-fill::after{top:-6px;background:#bae6fd88;animation-duration:3s;animation-direction:reverse}
@keyframes ola{0%,100%{transform:translateX(-10%) rotate(-3deg)}50%{transform:translateX(10%) rotate(3deg)}}
.arena-fill{background:radial-gradient(#ffe0a0 18%,transparent 22%),radial-gradient(#85562f 18%,transparent 22%),linear-gradient(#e9c68b,#bb884b,#85562f);background-size:9px 9px,13px 13px,100% 100%}
.arena-fill::before{content:"";position:absolute;top:-7px;left:-5%;width:110%;height:17px;border-radius:50%;background:#f2d097}
.granos{position:absolute;inset:0;z-index:4;opacity:0;overflow:hidden}
.tanque.cayendo .granos{opacity:1}
.granos i{position:absolute;top:-12px;left:var(--x);width:var(--tam);height:var(--tam);border-radius:50%;background:var(--color);animation:caer var(--tiempo) linear infinite;animation-delay:var(--retraso)}
@keyframes caer{0%{transform:translate(0,-10px);opacity:0}12%{opacity:1}85%{opacity:1}100%{transform:translate(var(--desvio),195px);opacity:0}}
.espuma-fill{background:linear-gradient(#ffffff,#e2e8f0,#94a3b8)}
.burbujas{position:absolute;top:50%;bottom:0;left:0;right:0;z-index:4;overflow:hidden;opacity:0}
.tanque.burbujeando .burbujas{opacity:1}
.burbujas i{position:absolute;left:var(--x);bottom:-15px;width:var(--tam);height:var(--tam);border-radius:50%;border:2px solid white;background:#ffffff55;animation:subir var(--tiempo) linear infinite;animation-delay:var(--retraso)}
@keyframes subir{0%{transform:translateY(0);opacity:0}15%{opacity:1}80%{opacity:.9}100%{transform:translateY(-105px);opacity:0}}
.botones{display:flex;gap:8px;margin-top:10px}
.botones button{flex:1}
.encender{background:var(--green);color:#064e3b}
.apagar{background:#475569;color:#fff}
.calibrar{display:block;width:100%;margin-top:10px;background:#273e5d;color:var(--text)}
.estado{padding:12px;margin:12px 0;background:var(--card2);text-align:center;border-radius:10px;font-size:14px}
.lleno{display:none;padding:10px;margin:12px 0;background:#065f46;color:#d1fae5;border-radius:10px;text-align:center;font-weight:bold;font-size:13px}
.lleno.visible{display:block}
.config{background:var(--card2);border:1px solid #29415e;border-radius:13px;padding:14px;margin-top:14px}
.config h3{margin:0 0 10px;font-size:14px;color:#93c5fd}
.fila{display:flex;gap:8px;align-items:center;margin-bottom:8px;flex-wrap:wrap}
.fila label{font-size:12px;color:var(--muted2);min-width:60px}
.fila input{background:var(--bg);border:1px solid var(--border);color:var(--text);padding:8px;border-radius:8px;width:90px;font-size:14px}
.fila select{background:var(--bg);border:1px solid var(--border);color:var(--text);padding:8px;border-radius:8px;font-size:13px;flex:1}
.fila button{background:#2563eb;color:#fff;padding:9px 14px}
.total{margin-top:20px;padding:20px;background:#112d45;border:1px solid var(--blue);border-radius:17px;text-align:center}
.total strong{color:#67e8f9;font-size:28px}
.interlock{color:var(--yellow);font-size:11px;margin-top:6px}
.historial{margin-top:20px;background:var(--card);border:1px solid var(--border);border-radius:13px;padding:14px}
.historial h3{margin:0 0 8px;font-size:14px;color:#93c5fd}
#listaEventos{max-height:160px;overflow-y:auto;font-size:12px;color:var(--muted2);font-family:monospace}
#listaEventos div{padding:3px 0;border-bottom:1px dashed #ffffff11}
.grafica{margin-top:20px;padding:12px;background:var(--card2);border:1px solid #29415e;border-radius:13px}
.grafica svg{display:block;width:100%;height:auto;margin-top:10px;background:#0b192e;border-radius:10px;touch-action:pan-y}
.formula{margin-top:10px;color:var(--muted2);font-size:12px}
.area{margin-top:12px;text-align:center;font-size:21px;font-weight:bold}
[data-simple="1"] .config,
[data-simple="1"] .grafica,
[data-simple="1"] .historial{display:none}
.toggle-grupo{display:flex;gap:6px;margin-top:10px}
.toggle-grupo button{flex:1;padding:12px;background:var(--card2);border:2px solid var(--border);color:var(--text)}
.toggle-grupo button.activo{background:var(--green);border-color:var(--green);color:#064e3b}
@media (max-width:640px){
  .tanque{width:82px;height:130px}
  .porcentaje{font-size:24px}
  .visual{min-height:160px}
  .card{padding:14px}
  h2{font-size:16px}
  .config{padding:10px}
  #listaEventos{max-height:100px}
  .grafica{padding:8px}
  .formula{font-size:11px}
  .area{font-size:17px}
}
@media(prefers-reduced-motion:reduce){
  .granos i,.burbujas i,.agua-fill::before,.agua-fill::after{animation:none}
}
</style>
</head>
<body>
<header>
<h1>HABITABOT v2.4</h1>
<p>Llenado por objetivo + retorno + volumen mL + gráficas + simulación</p>
</header>

<main>
<div id="conexion">● Conectando con ESP32...</div>
<p id="mensaje">Sistema listo</p>

<button id="btn-simular" onclick="toggleSimulacion()">🧪 Iniciar simulación</button>
<button id="btn-tema" onclick="toggleTema()">🌓 Cambiar tema</button>
<button id="btn-simple" onclick="toggleSimple()">🔧 Modo simple</button>
<button id="btn-detener" onclick="detenerTodo()">🛑 DETENER TODO</button>

<div class="grid">

<!-- AGUA -->
<section class="card agua" id="cardAgua">
<h2>💧 Agua</h2>
<div class="visual">
  <div class="tanque" id="tanqueAgua">
    <div class="linea-objetivo" id="objLineaAgua" data-label="--" style="bottom:100%"></div>
    <div class="relleno agua-fill" id="rellenoAgua"></div>
    <div class="numero" id="numeroAgua">--</div>
  </div>
  <div class="datos">
    <div class="porcentaje" id="porcentajeAgua">--</div>
    <div class="detalle" id="volumenAgua">-- mL</div>
    <div class="detalle" id="distanciaAgua">--</div>
    <div class="detalle" id="alturaAgua">-- / 14 cm</div>
  </div>
</div>
<div class="spark"><canvas id="sparkAgua" width="280" height="40"></canvas></div>
<div class="lleno" id="llenoAgua">TANQUE LLENO</div>
<div class="estado">Bomba llenado: <b id="estadoAgua">⚪ Apagada</b></div>
<div class="toggle-grupo">
  <button id="btnLlenarAgua" onclick="enviar('bomba','on')">▶ Llenar</button>
  <button id="btnDevolverAgua" onclick="enviar('retorno_agua','on')">◀ Devolver</button>
</div>
<button class="calibrar" onclick="calibrar('agua')">Calibrar agua vacía (0%)</button>
<div class="config">
  <h3>⚙️ Objetivo de llenado</h3>
  <div class="fila"><label>Unidad</label>
    <select id="unidadAgua" onchange="cambiarUnidad('agua')"><option value="pct">Porcentaje (%)</option><option value="ml">Mililitros (mL)</option></select>
  </div>
  <div class="fila"><label>Objetivo</label>
    <input type="number" id="objAgua" value="100" min="0" max="100" step="1">
    <button onclick="guardarObjetivo('agua')">Guardar</button>
  </div>
  <div class="fila"><label>Modo <span title="Semáforo: apaga al llegar. Termostato: mantiene nivel.">ⓘ</span></label>
    <select id="modoAgua" onchange="guardarModo('agua')"><option value="0">Semáforo</option><option value="1">Termostato</option></select>
  </div>
  <p class="interlock" id="infoObjAgua"></p>
</div>
<div class="grafica">
  <b>Gráfica del agua</b>
  <svg id="grafAgua" viewBox="0 0 340 210"></svg>
  <div class="formula">f(x) = x³/27 − x²/3 + x + 2</div>
  <div class="area" style="color:#38bdf8">Área = 18 cm²</div>
</div>
</section>

<!-- ARENA -->
<section class="card arena" id="cardArena">
<h2>🏖️ Arena</h2>
<div class="visual">
  <div class="tanque" id="tanqueArena">
    <div class="linea-objetivo" id="objLineaArena" data-label="--" style="bottom:100%"></div>
    <div class="relleno arena-fill" id="rellenoArena"></div>
    <div class="granos" id="granosArena"></div>
    <div class="numero" id="numeroArena">--</div>
  </div>
  <div class="datos">
    <div class="porcentaje" id="porcentajeArena">--</div>
    <div class="detalle" id="volumenArena">-- mL</div>
    <div class="detalle" id="distanciaArena">--</div>
    <div class="detalle" id="alturaArena">-- / 11 cm</div>
  </div>
</div>
<div class="spark"><canvas id="sparkArena" width="280" height="40"></canvas></div>
<div class="lleno" id="llenoArena">TANQUE LLENO</div>
<div class="estado">Compuerta: <b id="estadoArena">⚪ Cerrada</b></div>
<div class="toggle-grupo">
  <button id="btnLlenarArena" onclick="enviar('arena','on')">▶ Abrir</button>
  <button id="btnDevolverArena" onclick="enviar('arena','off')">■ Cerrar</button>
</div>
<button class="calibrar" onclick="calibrar('arena')">Calibrar arena vacía (0%)</button>
<div class="config">
  <h3>⚙️ Objetivo de llenado</h3>
  <div class="fila"><label>Unidad</label>
    <select id="unidadArena" onchange="cambiarUnidad('arena')"><option value="pct">Porcentaje (%)</option><option value="ml">Mililitros (mL)</option></select>
  </div>
  <div class="fila"><label>Objetivo</label>
    <input type="number" id="objArena" value="100" min="0" max="100" step="1">
    <button onclick="guardarObjetivo('arena')">Guardar</button>
  </div>
  <div class="fila"><label>Modo <span title="Semáforo: cierra al llegar. Termostato: mantiene nivel.">ⓘ</span></label>
    <select id="modoArena" onchange="guardarModo('arena')"><option value="0">Semáforo</option><option value="1">Termostato</option></select>
  </div>
  <p class="interlock" id="infoObjArena"></p>
</div>
<div class="grafica">
  <b>Gráfica de la arena</b>
  <svg id="grafArena" viewBox="0 0 340 210"></svg>
  <div class="formula">5−x, x+1, 10−1.5x</div>
  <div class="area" style="color:#fbbf24">Área = 21 cm²</div>
</div>
</section>

<!-- ESPUMA -->
<section class="card espuma" id="cardEspuma">
<h2>🫧 Espuma</h2>
<div class="visual">
  <div class="tanque tanque-espuma" id="tanqueEspuma">
    <div class="linea-objetivo" id="objLineaEspuma" data-label="--" style="bottom:100%"></div>
    <div class="relleno espuma-fill" id="rellenoEspuma"></div>
    <div class="burbujas" id="burbujasEspuma"></div>
    <div class="numero" id="numeroEspuma">--</div>
  </div>
  <div class="datos">
    <div class="porcentaje" id="porcentajeEspuma">--</div>
    <div class="detalle" id="volumenEspuma">-- mL</div>
    <div class="detalle" id="distanciaEspuma">--</div>
    <div class="detalle" id="alturaEspuma">-- / 7 cm</div>
  </div>
</div>
<div class="spark"><canvas id="sparkEspuma" width="280" height="40"></canvas></div>
<div class="lleno" id="llenoEspuma">TANQUE LLENO</div>
<div class="estado">Generador: <b id="estadoEspuma">⚪ Apagado</b></div>
<div class="toggle-grupo">
  <button id="btnLlenarEspuma" onclick="enviar('espuma','on')">▶ Generar</button>
  <button id="btnDevolverEspuma" onclick="enviar('retorno_jabon','on')">◀ Devolver</button>
</div>
<button class="calibrar" onclick="calibrar('espuma')">Calibrar espuma vacía (0%)</button>
<div class="config">
  <h3>⚙️ Objetivo de llenado</h3>
  <div class="fila"><label>Unidad</label>
    <select id="unidadEspuma" onchange="cambiarUnidad('espuma')"><option value="pct">Porcentaje (%)</option><option value="ml">Mililitros (mL)</option></select>
  </div>
  <div class="fila"><label>Objetivo</label>
    <input type="number" id="objEspuma" value="100" min="0" max="100" step="1">
    <button onclick="guardarObjetivo('espuma')">Guardar</button>
  </div>
  <div class="fila"><label>Modo <span title="Semáforo: apaga al llegar. Termostato: mantiene nivel.">ⓘ</span></label>
    <select id="modoEspuma" onchange="guardarModo('espuma')"><option value="0">Semáforo</option><option value="1">Termostato</option></select>
  </div>
  <p class="interlock" id="infoObjEspuma"></p>
</div>
<div class="grafica">
  <b>Gráfica de la espuma</b>
  <svg id="grafEspuma" viewBox="0 0 340 210"></svg>
  <div class="formula">f(x) = −x²/4 + x + 2</div>
  <div class="area" style="color:#c084fc">Área = 10.67 cm²</div>
</div>
</section>

</div>

<div class="historial">
  <h3>📋 Historial reciente</h3>
  <div id="listaEventos"></div>
</div>

<div class="total">
<h2>Área total bajo las curvas</h2>
<strong>49.67 cm²</strong>
<p>Agua 18 | Arena 21 | Espuma 10.67</p>
<small>Gráficas matemáticas ilustrativas. No representan el volumen real.</small>
</div>

<div class="total" style="margin-top:12px">
<h2>Volumen total disponible</h2>
<strong id="volTotal">-- mL</strong>
<p>Agua 1500 mL | Arena 1250 mL | Espuma 850 mL</p>
</div>
</main>

<script>
const $ = id => document.getElementById(id);

const themeIni = localStorage.getItem('hab_theme') || 'dark';
document.documentElement.setAttribute('data-theme', themeIni);
document.documentElement.setAttribute('data-simple', localStorage.getItem('hab_simple') || '0');

const toggleTema = () => {
  const a = document.documentElement.getAttribute('data-theme');
  const n = a === 'dark' ? 'light' : 'dark';
  document.documentElement.setAttribute('data-theme', n);
  localStorage.setItem('hab_theme', n);
};
const toggleSimple = () => {
  const a = document.documentElement.getAttribute('data-simple');
  const n = a === '1' ? '0' : '1';
  document.documentElement.setAttribute('data-simple', n);
  localStorage.setItem('hab_simple', n);
};

const eventos = [];
const registrarEvento = (txt) => {
  const h = new Date().toLocaleTimeString();
  eventos.unshift(`${h} — ${txt}`);
  if (eventos.length > 20) eventos.pop();
  $('listaEventos').innerHTML = eventos.map(e => `<div>${e}</div>`).join('');
};

const enviar = async (disp, est) => {
  const btns = document.querySelectorAll(`button[onclick*="'${disp}','${est}'"]`);
  const orig = [];
  btns.forEach(b => { orig.push(b.textContent); b.disabled = true; b.textContent = '⏳'; });
  try {
    const r = await fetch(`/control?dispositivo=${disp}&estado=${est}`, {cache:'no-store'});
    const d = await r.json();
    $('mensaje').textContent = d.mensaje;
    registrarEvento(d.mensaje);
    if (disp !== 'todo' && est === 'on') {
      if (disp === 'bomba') await fetch('/control?dispositivo=retorno_agua&estado=off');
      if (disp === 'retorno_agua') await fetch('/control?dispositivo=bomba&estado=off');
      if (disp === 'espuma') await fetch('/control?dispositivo=retorno_jabon&estado=off');
      if (disp === 'retorno_jabon') await fetch('/control?dispositivo=espuma&estado=off');
    }
    await actualizarDatos();
  } catch(e) {
    $('mensaje').textContent = 'Error de comunicación';
    registrarEvento('Error de comunicación');
  } finally {
    btns.forEach((b, i) => { b.disabled = false; b.textContent = orig[i]; });
  }
};

const calibrar = (mat) => enviar('calibrar_' + mat, 'on');
const detenerTodo = () => { if (confirm('¿Detener todos los actuadores y salir de simulación?')) enviar('todo', 'off'); };

// SIMULACION
let simActiva = false;
const toggleSimulacion = async () => {
  const accion = simActiva ? 'off' : 'on';
  try {
    const r = await fetch(`/simulacion?estado=${accion}`, {cache:'no-store'});
    const d = await r.json();
    $('mensaje').textContent = d.mensaje;
    registrarEvento(d.mensaje);
    simActiva = (accion === 'on');
    actualizarBotonSim();
    await actualizarDatos();
  } catch(e) {
    $('mensaje').textContent = 'Error de comunicación';
  }
};

const actualizarBotonSim = () => {
  const b = $('btn-simular');
  if (simActiva) {
    b.textContent = '❌ Cancelar simulación';
    b.classList.add('activo');
    $('conexion').classList.add('simulando');
    $('conexion').textContent = '🧪 MODO SIMULACIÓN ACTIVO';
  } else {
    b.textContent = '🧪 Iniciar simulación';
    b.classList.remove('activo');
    $('conexion').classList.remove('simulando');
  }
};

const cap = { agua:1500, arena:1250, espuma:850 };
const cambiarUnidad = async (mat) => {
  const m = mat.charAt(0).toUpperCase() + mat.slice(1);
  const sel = $('unidad' + m), inp = $('obj' + m);
  if (sel.value === 'ml') { inp.max = cap[mat]; inp.step = 10; if (parseFloat(inp.value) <= 100) inp.value = cap[mat]; }
  else { inp.max = 100; inp.step = 1; }
  await guardarObjetivo(mat);
};
const guardarObjetivo = async (mat) => {
  const m = mat.charAt(0).toUpperCase() + mat.slice(1);
  const val = parseFloat($('obj' + m).value) || 0;
  const unidad = $('unidad' + m).value;
  try {
    const r = await fetch(`/objetivo?material=${mat}&unidad=${unidad}&valor=${val}`, {cache:'no-store'});
    const d = await r.json();
    $('mensaje').textContent = d.mensaje;
    registrarEvento(d.mensaje);
    await actualizarDatos();
  } catch(e) { $('mensaje').textContent = 'Error de comunicación'; }
};
const guardarModo = async (mat) => {
  const m = mat.charAt(0).toUpperCase() + mat.slice(1);
  try {
    const r = await fetch(`/modo?material=${mat}&valor=${$('modo' + m).value}`, {cache:'no-store'});
    const d = await r.json();
    $('mensaje').textContent = d.mensaje;
    registrarEvento(d.mensaje);
    await actualizarDatos();
  } catch(e) {}
};

const histData = { Agua: [], Arena: [], Espuma: [] };
const dibujarSpark = (id, data, color) => {
  const c = $(id); if (!c) return;
  const ctx = c.getContext('2d');
  ctx.clearRect(0, 0, c.width, c.height);
  if (data.length < 2) return;
  const dx = c.width / 60;
  ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = 2;
  data.forEach((v, i) => {
    const x = i * dx, y = c.height - (v / 100) * (c.height - 4) - 2;
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();
  ctx.fillStyle = color + '33';
  ctx.lineTo((data.length - 1) * dx, c.height); ctx.lineTo(0, c.height); ctx.closePath(); ctx.fill();
};

const activarDrag = (idLinea, idTanque, mat) => {
  const linea = $(idLinea), tanque = $(idTanque);
  if (!linea || !tanque) return;
  linea.addEventListener('pointerdown', e => {
    e.preventDefault();
    const rect = tanque.getBoundingClientRect();
    const mover = ev => {
      const pct = Math.round(100 - ((ev.clientY - rect.top) / rect.height) * 100);
      const p = Math.max(0, Math.min(100, pct));
      linea.style.bottom = p + '%';
      linea.setAttribute('data-label', p + '%');
    };
    const soltar = () => {
      document.removeEventListener('pointermove', mover);
      document.removeEventListener('pointerup', soltar);
      const p = Math.round(parseFloat(linea.style.bottom));
      const m = mat.charAt(0).toUpperCase() + mat.slice(1);
      $('unidad' + m).value = 'pct';
      $('obj' + m).value = p;
      guardarObjetivo(mat);
    };
    document.addEventListener('pointermove', mover);
    document.addEventListener('pointerup', soltar);
  });
};
activarDrag('objLineaAgua', 'tanqueAgua', 'agua');
activarDrag('objLineaArena', 'tanqueArena', 'arena');
activarDrag('objLineaEspuma', 'tanqueEspuma', 'espuma');

const beep = () => {
  try {
    const ctx = new (window.AudioContext || window.webkitAudioContext)();
    const o = ctx.createOscillator(), g = ctx.createGain();
    o.frequency.value = 880; o.connect(g); g.connect(ctx.destination);
    g.gain.setValueAtTime(0.15, ctx.currentTime);
    g.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + 0.2);
    o.start(); o.stop(ctx.currentTime + 0.2);
  } catch(e) {}
};
const alcanzadoPrevio = { Agua: false, Arena: false, Espuma: false };

const capaArena = $('granosArena');
if (capaArena) {
  for (let i = 0; i < 34; i++) {
    const grano = document.createElement('i');
    grano.style.setProperty('--x', (35 + i * 19 % 30) + '%');
    grano.style.setProperty('--tam', (2 + i % 3) + 'px');
    grano.style.setProperty('--tiempo', (.65 + i % 5 * .15) + 's');
    grano.style.setProperty('--retraso', (-i * .073) + 's');
    grano.style.setProperty('--desvio', (i * 13 % 41 - 20) + 'px');
    const colores = ['#ffe0a0', '#c18b49', '#f6cf88'];
    grano.style.setProperty('--color', colores[i % 3]);
    capaArena.appendChild(grano);
  }
}
const capaBurbujas = $('burbujasEspuma');
if (capaBurbujas) {
  for (let i = 0; i < 13; i++) {
    const burbuja = document.createElement('i');
    burbuja.style.setProperty('--x', (8 + i * 23 % 84) + '%');
    burbuja.style.setProperty('--tam', (7 + i * 7 % 12) + 'px');
    burbuja.style.setProperty('--tiempo', (1.9 + i % 5 * .55) + 's');
    burbuja.style.setProperty('--retraso', (-i * .41) + 's');
    capaBurbujas.appendChild(burbuja);
  }
}

const fAgua = x => x*x*x/27 - x*x/3 + x + 2;
const fArena = x => x <= 2 ? 5 - x : x <= 4 ? x + 1 : 10 - 1.5 * x;
const fEspuma = x => -x*x/4 + x + 2;

const crearGrafica = (id, f, maxX, maxY, color) => {
  const svg = $(id); if (!svg) return;
  const ns = 'http://www.w3.org/2000/svg';
  svg.textContent = '';
  const L = 38, R = 324, T = 30, B = 175;
  const X = x => L + x / maxX * (R - L);
  const Y = y => B - y / maxY * (B - T);
  const crear = (tipo, attrs, contenido) => {
    const e = document.createElementNS(ns, tipo);
    Object.keys(attrs).forEach(k => e.setAttribute(k, attrs[k]));
    if (contenido !== undefined) e.textContent = contenido;
    svg.appendChild(e); return e;
  };
  for (let y = 0; y <= maxY; y++) {
    crear('line', { x1:L, y1:Y(y), x2:R, y2:Y(y), stroke:'#30435e', 'stroke-dasharray':'3 5' });
    crear('text', { x:28, y:Y(y)+4, fill:'#a6bdd9', 'font-size':11, 'text-anchor':'end' }, String(y));
  }
  for (let x = 0; x <= maxX; x++) {
    crear('line', { x1:X(x), y1:T, x2:X(x), y2:B, stroke:'#263b56', 'stroke-dasharray':'3 5' });
    crear('text', { x:X(x), y:195, fill:'#a6bdd9', 'font-size':11, 'text-anchor':'middle' }, String(x));
  }
  let curva = '';
  for (let i = 0; i <= 160; i++) {
    const x = maxX * i / 160;
    curva += (i === 0 ? 'M ' : 'L ') + X(x) + ' ' + Y(f(x)) + ' ';
  }
  crear('path', { d:curva + 'L ' + R + ' ' + B + ' L ' + L + ' ' + B + ' Z', fill:color, 'fill-opacity':0.22 });
  crear('path', { d:curva, fill:'none', stroke:color, 'stroke-width':9, opacity:0.15 });
  crear('path', { d:curva, fill:'none', stroke:color, 'stroke-width':3.5, 'stroke-linecap':'round' });
  for (let x = 0; x <= maxX; x++) crear('circle', { cx:X(x), cy:Y(f(x)), r:3.5, fill:'#fff', stroke:color, 'stroke-width':2 });
  const guia = crear('line', { x1:L, y1:T, x2:L, y2:B, stroke:color, 'stroke-dasharray':'4 4' });
  const punto = crear('circle', { cx:L, cy:Y(f(0)), r:6, fill:'#fff', stroke:color, 'stroke-width':3 });
  const etiqueta = crear('text', { x:170, y:18, fill:'#fff', 'font-size':12, 'text-anchor':'middle' }, '');
  const mostrar = x => {
    x = Math.max(0, Math.min(maxX, x));
    const y = f(x);
    guia.setAttribute('x1', X(x)); guia.setAttribute('x2', X(x));
    punto.setAttribute('cx', X(x)); punto.setAttribute('cy', Y(y));
    etiqueta.textContent = 'x: ' + x.toFixed(2) + ' | y: ' + y.toFixed(2);
  };
  const zona = crear('rect', { x:L, y:T, width:R-L, height:B-T, fill:'transparent', style:'cursor:crosshair' });
  const mover = e => {
    const rect = svg.getBoundingClientRect();
    const px = (e.clientX - rect.left) * 340 / rect.width;
    mostrar((px - L) / (R - L) * maxX);
  };
  zona.addEventListener('pointermove', mover);
  zona.addEventListener('pointerdown', mover);
  mostrar(maxX / 2);
};
crearGrafica('grafAgua', fAgua, 6, 6, '#38bdf8');
crearGrafica('grafArena', fArena, 6, 6, '#fbbf24');
crearGrafica('grafEspuma', fEspuma, 4, 5, '#c084fc');

const actualizarTanque = (nombre, pct, vol, dist, altura, maxCm, objetivo, lleno) => {
  const valido = Number.isFinite(dist) && dist >= 0 && Number.isFinite(altura) && altura >= 0;
  const p = valido ? Math.max(0, Math.min(100, pct)) : 0;
  $('relleno' + nombre).style.height = p + '%';
  $('numero' + nombre).textContent = valido ? p + '%' : '--';
  $('porcentaje' + nombre).textContent = valido ? p + '%' : '--';
  $('volumen' + nombre).textContent = valido && vol >= 0 ? Math.round(vol) + ' mL' : '-- mL';
  $('distancia' + nombre).textContent = valido ? 'Sensor: ' + dist.toFixed(1) + ' cm' : 'Sensor sin lectura';
  $('altura' + nombre).textContent = valido ? altura.toFixed(1) + ' / ' + maxCm + ' cm' : '-- / ' + maxCm + ' cm';
  $('lleno' + nombre).classList.toggle('visible', valido && Boolean(lleno));
  const linea = $('objLinea' + nombre);
  if (linea) {
    linea.style.bottom = Math.min(100, Math.max(0, objetivo)) + '%';
    linea.setAttribute('data-label', objetivo + '%');
  }
  const alcanzado = valido && p >= objetivo;
  $('card' + nombre).classList.toggle('alcanzado', alcanzado);
  if (alcanzado && !alcanzadoPrevio[nombre]) beep();
  alcanzadoPrevio[nombre] = alcanzado;
  histData[nombre].push(p);
  if (histData[nombre].length > 60) histData[nombre].shift();
  const colores = { Agua:'#38bdf8', Arena:'#fbbf24', Espuma:'#c084fc' };
  dibujarSpark('spark' + nombre, histData[nombre], colores[nombre]);
};

let arenaAnterior = null, animarArenaHasta = 0, lastOk = Date.now();

const actualizarDatos = async () => {
  try {
    const r = await fetch('/datos', {cache:'no-store'});
    if (!r.ok) throw new Error();
    const d = await r.json();

    simActiva = Boolean(d.simulacion);
    actualizarBotonSim();

    actualizarTanque('Agua',   d.pct_agua,   d.vol_agua,   d.agua,   d.altura_agua,   14, d.objetivo_agua,   d.lleno_agua);
    actualizarTanque('Arena',  d.pct_arena,  d.vol_arena,  d.arena,  d.altura_arena,  11, d.objetivo_arena,  d.lleno_arena);
    actualizarTanque('Espuma', d.pct_espuma, d.vol_espuma, d.espuma, d.altura_espuma,  7, d.objetivo_espuma, d.lleno_espuma);

    ['Agua','Arena','Espuma'].forEach(m => {
      if (document.activeElement !== $('obj' + m)) {
        const mat = m.toLowerCase();
        $('unidad' + m).value = d['unidad_' + mat] ? 'ml' : 'pct';
        $('obj' + m).value = d['unidad_' + mat] ? Math.round(d['objetivo_' + mat + '_ml']) : d['objetivo_' + mat];
        $('modo' + m).value = d['modo_' + mat] ? '1' : '0';
      }
    });

    $('estadoAgua').textContent = d.bomba ? '🟢 Encendida' : '⚪ Apagada';
    $('estadoArena').textContent = d.arena_abierta ? '🟢 Abierta' : '⚪ Cerrada';
    $('estadoEspuma').textContent = d.espuma_abierta ? '🟢 Encendido' : '⚪ Apagado';

    $('infoObjAgua').textContent = d.unidad_agua
      ? `Objetivo: ${Math.round(d.objetivo_agua_ml)} mL (${d.objetivo_agua}%)`
      : `Objetivo: ${d.objetivo_agua}% (${Math.round(d.objetivo_agua*15)} mL)`;
    $('infoObjArena').textContent = d.unidad_arena
      ? `Objetivo: ${Math.round(d.objetivo_arena_ml)} mL (${d.objetivo_arena}%)`
      : `Objetivo: ${d.objetivo_arena}% (${Math.round(d.objetivo_arena*12.5)} mL)`;
    $('infoObjEspuma').textContent = d.unidad_espuma
      ? `Objetivo: ${Math.round(d.objetivo_espuma_ml)} mL (${d.objetivo_espuma}%)`
      : `Objetivo: ${d.objetivo_espuma}% (${Math.round(d.objetivo_espuma*8.5)} mL)`;

    const vt = (d.vol_agua >= 0 ? d.vol_agua : 0) + (d.vol_arena >= 0 ? d.vol_arena : 0) + (d.vol_espuma >= 0 ? d.vol_espuma : 0);
    $('volTotal').textContent = Math.round(vt) + ' mL';

    const ahora = Date.now();
    if (arenaAnterior !== null && d.pct_arena > arenaAnterior) animarArenaHasta = ahora + 2300;
    arenaAnterior = d.pct_arena;
    $('tanqueArena').classList.toggle('cayendo', Boolean(d.arena_abierta) || ahora < animarArenaHasta);
    $('tanqueEspuma').classList.toggle('burbujeando', Boolean(d.espuma_abierta) || d.pct_espuma > 0);

    if (!simActiva) {
      $('conexion').textContent = '● ESP32 conectado';
      $('conexion').classList.remove('desconectado');
    }
    lastOk = Date.now();
  } catch(e) {
    if (!simActiva) {
      $('conexion').textContent = '● Reconectando...';
      $('conexion').classList.add('desconectado');
    }
  }
};

setInterval(() => {
  if (Date.now() - lastOk > 3000 && !simActiva) $('conexion').classList.add('desconectado');
}, 1000);

actualizarDatos();
setInterval(actualizarDatos, 900);
</script>
</body>
</html>
)HTML";

// ============ ENDPOINTS ============
void responder(bool ok, String msg) {
  String j = "{\"ok\":"; j += ok ? "true" : "false";
  j += ",\"mensaje\":\""; j += msg; j += "\"}";
  servidor.send(ok ? 200 : 400, "application/json", j);
}

void manejarControl() {
  String d = servidor.arg("dispositivo");
  String e = servidor.arg("estado");
  if (e != "on" && e != "off") { responder(false, "Estado invalido"); return; }
  bool on = (e == "on");

  if (d == "bomba") {
    if (on && retornoAguaActivo) { responder(false, "Retorno agua activo, espere"); return; }
    controlarBomba(on); responder(true, on ? "Bomba encendida" : "Bomba apagada");
  } else if (d == "arena") {
    controlarArena(on); responder(true, on ? "Arena abierta" : "Arena cerrada");
  } else if (d == "espuma") {
    controlarEspuma(on); responder(true, on ? "Generador encendido" : "Generador apagado");
  } else if (d == "retorno_agua") {
    if (on && bombaActiva) { responder(false, "Llenado activo, espere"); return; }
    controlarRetornoAgua(on); responder(true, on ? "Retorno agua encendido" : "Retorno agua apagado");
  } else if (d == "retorno_jabon") {
    if (on && espumaActiva) { responder(false, "Generador activo, espere"); return; }
    controlarRetornoJabon(on); responder(true, on ? "Retorno jabón encendido" : "Retorno jabón apagado");
  } else if (d == "todo") {
    detenerTodo(); responder(true, "Todo detenido");
  } else if (d.startsWith("calibrar_")) {
    if (simulacionActiva) { responder(false, "No calibrar en simulación"); return; }
    int s = d == "calibrar_agua" ? AGUA : d == "calibrar_arena" ? ARENA : ESPUMA;
    bool ok = calibrarVacio(s);
    responder(ok, ok ? "Calibrado al 0%" : "Sensor sin lectura");
  } else {
    responder(false, "Dispositivo desconocido");
  }
}

void manejarObjetivo() {
  String mat = servidor.arg("material");
  String u = servidor.arg("unidad");
  float v = servidor.arg("valor").toFloat();
  int s = mat == "agua" ? AGUA : mat == "arena" ? ARENA : mat == "espuma" ? ESPUMA : -1;
  if (s < 0 || (u != "pct" && u != "ml")) { responder(false, "Invalido"); return; }
  bool esMl = (u == "ml");
  unidadMl[s] = esMl;
  if (esMl) {
    v = constrain(v, 0, volumenMaximo[s]);
    objetivoMl[s] = v;
    objetivoPct[s] = constrain((int)roundf(v * 100.0f / volumenMaximo[s]), 0, 100);
  } else {
    v = constrain(v, 0, 100);
    objetivoPct[s] = (int)roundf(v);
    objetivoMl[s] = objetivoPct[s] * volumenMaximo[s] / 100.0f;
  }
  const char* kP[3] = { "objAguaPct", "objArenaPct", "objEspumaPct" };
  const char* kM[3] = { "objAguaMl", "objArenaMl", "objEspumaMl" };
  const char* kU[3] = { "uniAgua", "uniArena", "uniEspuma" };
  memoria.putInt(kP[s], objetivoPct[s]);
  memoria.putFloat(kM[s], objetivoMl[s]);
  memoria.putBool(kU[s], unidadMl[s]);
  char b[80];
  snprintf(b, sizeof(b), "Objetivo %s: %d%% (%.0f mL)", mat.c_str(), objetivoPct[s], objetivoMl[s]);
  responder(true, String(b));
}

void manejarModo() {
  String mat = servidor.arg("material");
  int v = servidor.arg("valor").toInt();
  int s = mat == "agua" ? AGUA : mat == "arena" ? ARENA : mat == "espuma" ? ESPUMA : -1;
  if (s < 0) { responder(false, "Invalido"); return; }
  modoTermostato[s] = (v != 0);
  const char* k[3] = { "modoAgua", "modoArena", "modoEspuma" };
  memoria.putBool(k[s], modoTermostato[s]);
  responder(true, String("Modo ") + (modoTermostato[s] ? "termostato" : "semaforo"));
}

void manejarSimulacion() {
  String e = servidor.arg("estado");
  if (e == "on") {
    simulacionActiva = true;
    // Inicializar simulacion con los niveles actuales reales si los hay
    for (int i = 0; i < 3; i++) {
      simPorcentaje[i] = (float)obtenerPorcentaje(i);
    }
    ultimaSimulacion = millis();
    Serial.println("SIMULACION ACTIVADA");
    responder(true, "Simulación activada");
  } else if (e == "off") {
    simulacionActiva = false;
    Serial.println("SIMULACION DESACTIVADA");
    responder(true, "Simulación cancelada");
  } else {
    responder(false, "Estado invalido");
  }
}

void manejarDatos() {
  float hA = obtenerAltura(AGUA), hAr = obtenerAltura(ARENA), hE = obtenerAltura(ESPUMA);
  float vA = obtenerVolumenMl(AGUA), vAr = obtenerVolumenMl(ARENA), vE = obtenerVolumenMl(ESPUMA);
  int pA = obtenerPorcentaje(AGUA), pAr = obtenerPorcentaje(ARENA), pE = obtenerPorcentaje(ESPUMA);
  int oA = objetivoEfectivoPct(AGUA), oAr = objetivoEfectivoPct(ARENA), oE = objetivoEfectivoPct(ESPUMA);
  float dA = distanciaMostrada(AGUA), dAr = distanciaMostrada(ARENA), dE = distanciaMostrada(ESPUMA);
  char json[1300];
  snprintf(json, sizeof(json),
    "{"
    "\"agua\":%.2f,\"arena\":%.2f,\"espuma\":%.2f,"
    "\"pct_agua\":%d,\"pct_arena\":%d,\"pct_espuma\":%d,"
    "\"vol_agua\":%.1f,\"vol_arena\":%.1f,\"vol_espuma\":%.1f,"
    "\"altura_agua\":%.2f,\"altura_arena\":%.2f,\"altura_espuma\":%.2f,"
    "\"lleno_agua\":%d,\"lleno_arena\":%d,\"lleno_espuma\":%d,"
    "\"bomba\":%d,\"arena_abierta\":%d,\"espuma_abierta\":%d,"
    "\"retorno_agua\":%d,\"retorno_jabon\":%d,"
    "\"objetivo_agua\":%d,\"objetivo_arena\":%d,\"objetivo_espuma\":%d,"
    "\"objetivo_agua_ml\":%.0f,\"objetivo_arena_ml\":%.0f,\"objetivo_espuma_ml\":%.0f,"
    "\"unidad_agua\":%d,\"unidad_arena\":%d,\"unidad_espuma\":%d,"
    "\"modo_agua\":%d,\"modo_arena\":%d,\"modo_espuma\":%d,"
    "\"simulacion\":%d"
    "}",
    dA, dAr, dE,
    pA, pAr, pE, vA, vAr, vE, hA, hAr, hE,
    estaLleno(AGUA)?1:0, estaLleno(ARENA)?1:0, estaLleno(ESPUMA)?1:0,
    bombaActiva?1:0, arenaAbierta?1:0, espumaActiva?1:0,
    retornoAguaActivo?1:0, retornoJabonActivo?1:0,
    oA, oAr, oE,
    objetivoMl[AGUA], objetivoMl[ARENA], objetivoMl[ESPUMA],
    unidadMl[AGUA]?1:0, unidadMl[ARENA]?1:0, unidadMl[ESPUMA]?1:0,
    modoTermostato[AGUA]?1:0, modoTermostato[ARENA]?1:0, modoTermostato[ESPUMA]?1:0,
    simulacionActiva?1:0
  );
  servidor.send(200, "application/json", json);
}

// Redirige al index todo lo desconocido (captive portal)
void manejarNoEncontrado() {
  servidor.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  servidor.send(302, "text/plain", "");
}

// ============ SETUP ============
void setup() {
  Serial.begin(115200);

  pinMode(PIN_BOMBA, OUTPUT);         digitalWrite(PIN_BOMBA, RELE_INACTIVO);
  pinMode(PIN_RELE_ESPUMA, OUTPUT);   digitalWrite(PIN_RELE_ESPUMA, RELE_INACTIVO);
  pinMode(PIN_RETORNO_AGUA, OUTPUT);  digitalWrite(PIN_RETORNO_AGUA, RELE_INACTIVO);
  pinMode(PIN_RETORNO_JABON, OUTPUT); digitalWrite(PIN_RETORNO_JABON, RELE_INACTIVO);

  memoria.begin("habitabot", false);
  const char* cV[3] = { "aguaVacia", "arenaVacia", "espumaVacia" };
  for (int i = 0; i < 3; i++) {
    float g = memoria.getFloat(cV[i], distanciaVacio[i]);
    if (g >= 2.0f && g <= 40.0f) distanciaVacio[i] = g;
  }
  const char* kP[3] = { "objAguaPct", "objArenaPct", "objEspumaPct" };
  const char* kM[3] = { "objAguaMl", "objArenaMl", "objEspumaMl" };
  const char* kU[3] = { "uniAgua", "uniArena", "uniEspuma" };
  const char* kMo[3] = { "modoAgua", "modoArena", "modoEspuma" };
  for (int i = 0; i < 3; i++) {
    objetivoPct[i] = memoria.getInt(kP[i], 100);
    objetivoMl[i] = memoria.getFloat(kM[i], volumenMaximo[i]);
    unidadMl[i] = memoria.getBool(kU[i], false);
    modoTermostato[i] = memoria.getBool(kMo[i], false);
  }

  for (int i = 0; i < 3; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }
  pinMode(ECHO_AGUA, INPUT_PULLUP);

  ESP32PWM::allocateTimer(0);
  servoArena.setPeriodHertz(50);
  servoArena.attach(PIN_SERVO_ARENA, 500, 2400);
  if (servoArena.attached()) servoArena.write(SERVO_CERRADO);
  arenaAbierta = false;

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_NOMBRE, WIFI_CLAVE);
  IPAddress ip = WiFi.softAPIP();

  // DNS captive portal
  dnsServer.start(DNS_PORT, "*", ip);

  // Servidor web
  servidor.on("/", HTTP_GET, []() { servidor.send_P(200, "text/html; charset=utf-8", PAGINA); });
  servidor.on("/datos", HTTP_GET, manejarDatos);
  servidor.on("/control", HTTP_GET, manejarControl);
  servidor.on("/objetivo", HTTP_GET, manejarObjetivo);
  servidor.on("/modo", HTTP_GET, manejarModo);
  servidor.on("/simulacion", HTTP_GET, manejarSimulacion);

  // Endpoints de detección de captive portal
  servidor.on("/generate_204", HTTP_GET, manejarNoEncontrado);
  servidor.on("/gen_204", HTTP_GET, manejarNoEncontrado);
  servidor.on("/hotspot-detect.html", HTTP_GET, manejarNoEncontrado);
  servidor.on("/library/test/success.html", HTTP_GET, manejarNoEncontrado);
  servidor.on("/connecttest.txt", HTTP_GET, manejarNoEncontrado);
  servidor.on("/ncsi.txt", HTTP_GET, manejarNoEncontrado);
  servidor.on("/redirect", HTTP_GET, manejarNoEncontrado);
  servidor.on("/canonical.html", HTTP_GET, manejarNoEncontrado);
  servidor.on("/success.txt", HTTP_GET, manejarNoEncontrado);

  servidor.onNotFound(manejarNoEncontrado);
  servidor.begin();

  Serial.println("HABITABOT v2.4 INICIADO");
  Serial.print("IP AP: "); Serial.println(ip);
  Serial.println("Portal cautivo activo");
}

// ============ LOOP ============
void loop() {
  dnsServer.processNextRequest();
  servidor.handleClient();
  actualizarSensores();

  unsigned long ahora = millis();
  int pA = obtenerPorcentaje(AGUA), pAr = obtenerPorcentaje(ARENA), pE = obtenerPorcentaje(ESPUMA);
  int oA = objetivoEfectivoPct(AGUA), oAr = objetivoEfectivoPct(ARENA), oE = objetivoEfectivoPct(ESPUMA);

  if (bombaActiva && pA >= oA) controlarBomba(false);
  if (arenaAbierta && pAr >= oAr) controlarArena(false);
  if (espumaActiva && pE >= oE) controlarEspuma(false);

  if (modoTermostato[AGUA]) {
    if (!bombaActiva && !retornoAguaActivo && pA < oA - 2) controlarBomba(true);
    if (bombaActiva && pA >= oA) controlarBomba(false);
  }
  if (modoTermostato[ARENA]) {
    if (!arenaAbierta && pAr < oAr - 2) controlarArena(true);
    if (arenaAbierta && pAr >= oAr) controlarArena(false);
  }
  if (modoTermostato[ESPUMA]) {
    if (!espumaActiva && !retornoJabonActivo && pE < oE - 2) controlarEspuma(true);
    if (espumaActiva && pE >= oE) controlarEspuma(false);
  }

  if (retornoAguaActivo && pA <= NIVEL_VACIO) controlarRetornoAgua(false);
  if (retornoJabonActivo && pE <= NIVEL_VACIO) controlarRetornoJabon(false);

  if (bombaActiva && ahora - inicioBomba >= LIMITE_BOMBA) controlarBomba(false);
  if (arenaAbierta && ahora - inicioArena >= LIMITE_ARENA) controlarArena(false);
  if (espumaActiva && ahora - inicioEspuma >= LIMITE_ESPUMA) controlarEspuma(false);
  if (retornoAguaActivo && ahora - inicioRetornoAgua >= LIMITE_RETORNO) controlarRetornoAgua(false);
  if (retornoJabonActivo && ahora - inicioRetornoJabon >= LIMITE_RETORNO) controlarRetornoJabon(false);

  static unsigned long ultimoSerial = 0;
  if (ahora - ultimoSerial >= 2000UL) {
    ultimoSerial = ahora;
    Serial.printf("[%s] AGUA:%d%% ARENA:%d%% ESP:%d%%\n",
      simulacionActiva ? "SIM" : "REAL", pA, pAr, pE);
  }
}