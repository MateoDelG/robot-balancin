#include "mpu9250_9dof_test.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <MPU9250_asukiaaa.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

#include "../../include/config.h"
#include "imu9250_calibration.h"
#include "imu9250_filter_settings.h"
#include "ota_manager.h"

namespace {

constexpr uint8_t AK8963_ADDRESS = 0x0C;
constexpr uint8_t AK8963_WHO_AM_I = 0x00;
constexpr uint8_t MAG_MODE_CONTINUOUS_100HZ_16BIT = MAG_MODE_CONTINUOUS_100HZ | 0x10;
constexpr float MAG_16BIT_UT_PER_COUNT = 0.15f;
constexpr float RAD_TO_DEG_F = 57.2957795f;
constexpr float DEG_TO_RAD_F = 0.0174532925f;
constexpr uint8_t ACCEL_POSE_COUNT = 6;

const char *ACCEL_POSE_NAMES[ACCEL_POSE_COUNT] = {
    "+X hacia arriba", "-X hacia arriba", "+Y hacia arriba",
    "-Y hacia arriba", "+Z hacia arriba", "-Z hacia arriba"};

MPU9250_asukiaaa imuAt68(MPU9250_ADDRESS_AD0_LOW);
MPU9250_asukiaaa imuAt69(MPU9250_ADDRESS_AD0_HIGH);
MPU9250_asukiaaa *imu = nullptr;
WebServer server(80);
WebSocketsServer webSocket(81);

bool imuReady = false;
bool accelReady = false;
bool gyroReady = false;
bool magReady = false;
bool freshAccel = false;
bool freshGyro = false;
bool freshMag = false;
uint8_t imuId = 0;
uint8_t imuAddress = 0;
uint8_t magId = 0;

float accelRaw[3] = {};
float accelCorrected[3] = {};
float gyroRaw[3] = {};
float gyroCorrected[3] = {};
float magRaw[3] = {};
float magCorrectedUt[3] = {};
float accelNorm = 0.0f;
float magneticNormUt = 0.0f;
float accelRollDeg = 0.0f;
float accelPitchDeg = 0.0f;
float filteredRollDeg = 0.0f;
float filteredPitchDeg = 0.0f;
float headingDeg = 0.0f;
bool filterInitialized = false;
unsigned long lastFilterUs = 0;
float filterAlpha = Config::SENSOR_COMPLEMENTARY_ALPHA;

bool traceActive = false;
bool tracePaused = false;
unsigned long traceStartMs = 0;
unsigned long tracePauseStartMs = 0;
unsigned long tracePausedTotalMs = 0;
unsigned long traceDurationMs = 30000;
unsigned long lastTraceSendMs = 0;
unsigned long lastGyroIntegrationUs = 0;
float gyroOnlyPitchDeg = 0.0f;
uint32_t traceSequence = 0;

uint32_t accelReads = 0;
uint32_t gyroReads = 0;
uint32_t magReads = 0;
uint32_t previousAccelReads = 0;
uint32_t previousGyroReads = 0;
uint32_t previousMagReads = 0;
uint32_t accelRateHz = 0;
uint32_t gyroRateHz = 0;
uint32_t magRateHz = 0;
unsigned long lastRateMs = 0;
unsigned long lastSerialPrintMs = 0;
unsigned long lastStateSendMs = 0;

enum class CalibrationMode { None, Gyro, AccelPose, Magnetometer };
CalibrationMode calibrationMode = CalibrationMode::None;
String calibrationStatus = "Sin calibracion activa";
unsigned long calibrationStartMs = 0;
uint32_t calibrationSamples = 0;

double sampleSum[3] = {};
double sampleSquareSum[3] = {};
bool automaticGyroCalibration = false;

bool accelWizardActive = false;
uint8_t accelPoseIndex = 0;
float accelPoseMean[ACCEL_POSE_COUNT][3] = {};

float magMinimum[3] = {};
float magMaximum[3] = {};
unsigned long lastMagSampleMs = 0;

const char DASHBOARD[] PROGMEM = R"rawliteral(
<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Laboratorio MPU9250</title><style>
:root{color-scheme:dark;--bg:#071014;--panel:#0c1c21;--line:#26464e;--text:#eff9fa;--muted:#91b0b7;--cyan:#38d6d1;--amber:#ffbd59;--purple:#bca7ff;--green:#66e3a4;--red:#ff7070}*{box-sizing:border-box}body{margin:0;background:radial-gradient(circle at 15% 0,#133841 0,transparent 36%),var(--bg);color:var(--text);font-family:Arial,sans-serif}header{display:flex;justify-content:space-between;gap:18px;align-items:center;padding:20px clamp(14px,4vw,44px);border-bottom:1px solid var(--line)}h1{margin:0;font-size:clamp(22px,4vw,34px)}h1 span{color:var(--cyan)}.sub,.muted{color:var(--muted);font-size:13px}.connection{display:flex;align-items:center;gap:8px;color:var(--muted);font-size:13px}.dot{width:10px;height:10px;border-radius:50%;background:var(--red)}.dot.on{background:var(--green);box-shadow:0 0 12px var(--green)}main{max-width:1300px;margin:auto;padding:20px clamp(12px,3vw,30px) 45px}.summary,.grid,.cal-grid{display:grid;gap:13px}.summary{grid-template-columns:repeat(5,1fr)}.grid{grid-template-columns:repeat(3,1fr);margin-top:13px}.cal-grid{grid-template-columns:repeat(2,1fr);margin-top:13px}.card,.panel{background:linear-gradient(145deg,#10262c,#09171b);border:1px solid var(--line);border-radius:14px}.card{padding:14px}.card label,.axis th{color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.1em}.card strong{display:block;font:700 22px Consolas,monospace;margin-top:7px}.panel{padding:17px}.panel h2{margin:0 0 13px;font-size:16px;display:flex;justify-content:space-between}.unit{font-size:11px;color:var(--muted);font-weight:normal}.axis{width:100%;border-collapse:collapse}.axis th,.axis td{text-align:right;padding:9px 5px;border-top:1px solid #19353c}.axis th:first-child,.axis td:first-child{text-align:left}.axis td{font:700 17px Consolas,monospace}.raw{color:var(--muted)}.corrected{color:var(--cyan)}.actions{display:flex;flex-wrap:wrap;gap:8px;margin-top:12px}button{border:1px solid #3b626b;background:#17343b;color:var(--text);padding:10px 13px;border-radius:9px;font-weight:700;cursor:pointer}button.primary{background:#087f7d;border-color:#24bebb}button.warn{border-color:#9c6135}button:disabled{opacity:.4;cursor:not-allowed}.status{padding:11px;border-radius:9px;background:#081518;color:var(--amber);min-height:40px}.checks{display:grid;grid-template-columns:1fr 1fr;gap:8px}.check{padding:9px;border-radius:8px;background:#081518;color:var(--muted)}.pass{color:var(--green)}.fail{color:var(--red)}.values{font:13px Consolas,monospace;color:var(--muted);line-height:1.6;white-space:pre-line}.step{font-size:20px;color:var(--purple);font-weight:700;margin:8px 0}footer{text-align:center;color:var(--muted);font-size:12px;margin-top:18px}@media(max-width:900px){.summary{grid-template-columns:repeat(2,1fr)}.grid,.cal-grid{grid-template-columns:1fr}}@media(max-width:480px){header{align-items:flex-start;flex-direction:column}.summary{grid-template-columns:1fr 1fr}.axis td{font-size:14px}}
.trace-panel{margin-top:14px}.trace-controls{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:10px}.trace-controls label{font-size:12px;color:var(--muted)}.trace-controls input{display:block;width:100%;margin-top:5px;padding:9px;border-radius:8px;border:1px solid var(--line);background:#081518;color:var(--text)}.series{display:flex;flex-wrap:wrap;gap:14px;margin:12px 0;color:var(--muted);font-size:13px}.series input{accent-color:var(--cyan)}canvas{display:block;width:100%;height:300px;background:#071316;border:1px solid var(--line);border-radius:10px;margin-top:10px}.rate-chart{height:190px}.trace-stats{width:100%;border-collapse:collapse;margin-top:12px;font-size:12px}.trace-stats th,.trace-stats td{padding:7px;text-align:right;border-top:1px solid var(--line)}.trace-stats th:first-child,.trace-stats td:first-child{text-align:left}.trace-badge{color:var(--amber)}@media(max-width:600px){canvas{height:240px}.rate-chart{height:160px}}
</style></head><body>
<header><div><h1>Laboratorio <span>MPU9250</span></h1><div class="sub">Medicion, orientacion y calibracion persistente</div></div><div class="connection"><span id="dot" class="dot"></span><span id="connection">Conectando</span></div></header><main>
<div class="summary"><div class="card"><label>Roll filtrado</label><strong><span id="roll">--</span> deg</strong></div><div class="card"><label>Pitch filtrado</label><strong><span id="pitch">--</span> deg</strong></div><div class="card"><label>Pitch relativo</label><strong><span id="relativePitch">--</span> deg</strong></div><div class="card"><label>Rumbo magnetico</label><strong><span id="heading">--</span> deg</strong></div><div class="card"><label>Norma magnetica</label><strong><span id="magNorm">--</span> uT</strong></div></div>
<div class="grid"><section class="panel"><h2>Acelerometro <span class="unit">g</span></h2><table class="axis"><tr><th>Eje</th><th>Crudo</th><th>Corregido</th></tr><tr><td>X</td><td class="raw" id="ar0">--</td><td class="corrected" id="ac0">--</td></tr><tr><td>Y</td><td class="raw" id="ar1">--</td><td class="corrected" id="ac1">--</td></tr><tr><td>Z</td><td class="raw" id="ar2">--</td><td class="corrected" id="ac2">--</td></tr></table><p class="muted">Norma: <b id="accelNorm">--</b> g | <span id="accelRate">--</span> Hz</p></section>
<section class="panel"><h2>Giroscopio <span class="unit">deg/s</span></h2><table class="axis"><tr><th>Eje</th><th>Crudo</th><th>Corregido</th></tr><tr><td>X</td><td class="raw" id="gr0">--</td><td class="corrected" id="gc0">--</td></tr><tr><td>Y</td><td class="raw" id="gr1">--</td><td class="corrected" id="gc1">--</td></tr><tr><td>Z</td><td class="raw" id="gr2">--</td><td class="corrected" id="gc2">--</td></tr></table><p class="muted"><span id="gyroRate">--</span> Hz</p></section>
<section class="panel"><h2>Magnetometro <span class="unit">uT</span></h2><table class="axis"><tr><th>Eje</th><th>Crudo</th><th>Corregido</th></tr><tr><td>X</td><td class="raw" id="mr0">--</td><td class="corrected" id="mc0">--</td></tr><tr><td>Y</td><td class="raw" id="mr1">--</td><td class="corrected" id="mc1">--</td></tr><tr><td>Z</td><td class="raw" id="mr2">--</td><td class="corrected" id="mc2">--</td></tr></table><p class="muted"><span id="magRate">--</span> Hz, crudo en cuentas ajustadas de fabrica</p></section></div>
<div class="cal-grid"><section class="panel"><h2>Calibraciones <span class="unit" id="calState">--</span></h2><div id="message" class="status">Esperando estado</div><div class="actions"><button id="gyroCal" class="primary">Calibrar gyro</button><button id="magCal">Calibrar magnetometro 30 s</button><button id="setVertical">Guardar vertical</button></div><p class="muted">Gyro: mantenga el modulo inmovil. Magnetometro: gire lentamente sobre los tres ejes.</p></section>
<section class="panel"><h2>Asistente acelerometro <span class="unit">6 posiciones</span></h2><div class="step" id="accelStep">No iniciado</div><p class="muted" id="accelHelp">Inicie el asistente y siga cada orientacion.</p><div class="actions"><button id="accelStart">Iniciar / reiniciar</button><button id="accelCapture" class="primary">Capturar posicion</button></div></section>
<section class="panel"><h2>Calibracion guardada <span class="unit">NVS</span></h2><div class="checks"><div class="check">Almacenamiento <b id="stored">--</b></div><div class="check">Acelerometro <b id="accelValid">--</b></div><div class="check">Giroscopio <b id="gyroValid">--</b></div><div class="check">Magnetometro <b id="magValid">--</b></div><div class="check">Vertical <b id="verticalValid">--</b></div><div class="check">Muestras <b id="samples">0</b></div></div><div id="calValues" class="values"></div><div class="actions"><button id="clearCal" class="warn">Borrar toda la calibracion</button></div></section>
<section class="panel"><h2>Diagnostico</h2><div class="checks"><div class="check">MPU9250 <b id="imuCheck">--</b></div><div class="check">AK8963 <b id="magCheck">--</b></div><div class="check">Direccion I2C <b id="imuAddress">--</b></div><div class="check">OTA <b id="ota">--</b></div></div><div class="actions"><button id="scan">Escanear I2C</button></div><p class="muted" id="scanResult">Sin escanear</p></section></div>
<section class="panel trace-panel"><h2>Analizador de filtro <span class="unit trace-badge" id="traceStatus">DETENIDO</span></h2><div class="trace-controls"><label>Duracion de captura (s)<input id="traceDuration" type="number" min="5" max="300" value="30"></label><label>Alpha complementario<input id="filterAlpha" type="number" min="0.80" max="0.999" step="0.001" value="0.980"></label><div><span class="muted">Configuracion</span><div class="actions"><button id="applyAlpha">Aplicar</button><button id="saveAlpha" class="primary">Guardar NVS</button><button id="resetAlpha">Restaurar</button></div></div><div><span class="muted">Captura</span><div class="actions"><button id="traceStart" class="primary">Iniciar</button><button id="tracePause">Pausar</button><button id="traceStop">Detener</button><button id="traceClear">Limpiar</button><button id="traceCsv">Exportar CSV</button></div></div></div><div class="checks" style="margin-top:12px"><div class="check">Muestras <b id="traceSamples">0</b></div><div class="check">Frecuencia recibida <b id="traceRate">0 Hz</b></div><div class="check">Alpha activo <b id="activeAlpha">--</b></div><div class="check">Persistencia <b id="alphaStored">--</b></div></div><div class="series"><label><input id="showAccel" type="checkbox" checked> Acelerometro sin filtro</label><label><input id="showGyro" type="checkbox" checked> Gyro integrado</label><label><input id="showFiltered" type="checkbox" checked> Complementario</label></div><canvas id="angleChart"></canvas><canvas id="rateChart" class="rate-chart"></canvas><table class="trace-stats"><thead><tr><th>Serie</th><th>Media</th><th>Std dev</th><th>Min</th><th>Max</th><th>Pico-pico</th><th>Deriva</th></tr></thead><tbody id="statsBody"><tr><td colspan="7">Sin muestras</td></tr></tbody></table></section>
<footer>El magnetometro se usa para rumbo; el equilibrio depende principalmente de acelerometro y giroscopio.</footer></main><script>
const $=id=>document.getElementById(id);
function val(id,v,n=2){$(id).textContent=typeof v==='number'?v.toFixed(n):v}
function check(id,ok,text){const e=$(id);e.textContent=text;e.className=ok?'pass':'fail'}
let alphaDirty=false,traceActive=false,tracePaused=false,traceSamples=[],drawPending=false;
const series=[{key:'accel',name:'Acelerometro',color:'#38d6d1',toggle:'showAccel'},{key:'gyro',name:'Gyro integrado',color:'#ffbd59',toggle:'showGyro'},{key:'filtered',name:'Complementario',color:'#bca7ff',toggle:'showFiltered'}];
function render(d){
  for(let i=0;i<3;i++){val('ar'+i,d.accelRaw[i],3);val('ac'+i,d.accelCorrected[i],3);val('gr'+i,d.gyroRaw[i],2);val('gc'+i,d.gyroCorrected[i],2);val('mr'+i,d.magRaw[i],1);val('mc'+i,d.magCorrected[i],2)}
  val('roll',d.roll,1);val('pitch',d.pitch,1);val('relativePitch',d.relativePitch,1);val('heading',d.heading,1);val('magNorm',d.magNorm,1);val('accelNorm',d.accelNorm,3);val('accelRate',d.accelRate,0);val('gyroRate',d.gyroRate,0);val('magRate',d.magRate,0);val('calState',d.calibrationMode,0);val('message',d.calibrationStatus,0);val('samples',d.calibrationSamples,0);
  check('stored',d.calibrationStored,d.calibrationStored?'CARGADA':'VACIA');check('accelValid',d.accelCalibrated,d.accelCalibrated?'VALIDA':'PENDIENTE');check('gyroValid',d.gyroCalibrated,d.gyroCalibrated?'VALIDA':'PENDIENTE');check('magValid',d.magCalibrated,d.magCalibrated?'VALIDA':'PENDIENTE');check('verticalValid',d.verticalCalibrated,d.verticalCalibrated?'VALIDA':'PENDIENTE');check('imuCheck',d.imuReady,d.imuId);check('magCheck',d.magReady,d.magId);val('imuAddress',d.imuAddress,0);check('ota',d.otaAvailable,d.otaUpdating?'ACTUALIZANDO':(d.otaAvailable?'LISTO':'NO'));
  val('accelStep',d.accelWizardActive?(d.accelPoseIndex+1)+'/6: '+d.accelPoseName:'No iniciado',0);val('accelHelp',d.accelWizardActive?'Coloque esa cara hacia arriba, inmovilice y pulse Capturar.':'Inicie el asistente y siga cada orientacion.',0);
  $('calValues').textContent=`A off: ${d.accelOffset.map(v=>v.toFixed(4)).join(', ')}\nA scale: ${d.accelScale.map(v=>v.toFixed(4)).join(', ')}\nG off: ${d.gyroOffset.map(v=>v.toFixed(3)).join(', ')}\nM off: ${d.magOffset.map(v=>v.toFixed(1)).join(', ')}\nM scale: ${d.magScale.map(v=>v.toFixed(3)).join(', ')}\nVertical: roll ${d.verticalRoll.toFixed(2)}, pitch ${d.verticalPitch.toFixed(2)}`;
  const busy=d.calibrationMode!=='idle';$('gyroCal').disabled=busy;$('magCal').disabled=busy||!d.magReady;$('accelStart').disabled=busy;$('accelCapture').disabled=busy||!d.accelWizardActive;$('setVertical').disabled=busy||!d.filterReady;$('clearCal').disabled=busy;
  traceActive=d.traceActive;tracePaused=d.tracePaused;val('traceStatus',traceActive?(tracePaused?'PAUSADA':'CAPTURANDO'):'DETENIDA',0);val('activeAlpha',d.filterAlpha,3);val('alphaStored',d.filterAlphaStored?'NVS':'TEMPORAL',0);if(!alphaDirty)$('filterAlpha').value=d.filterAlpha.toFixed(3);$('traceStart').disabled=traceActive||busy;$('tracePause').disabled=!traceActive;$('tracePause').textContent=tracePaused?'Continuar':'Pausar';$('traceStop').disabled=!traceActive;
}
function sendPayload(payload){if(!ws||ws.readyState!==WebSocket.OPEN){val('message','WebSocket no conectado',0);return false}ws.send(JSON.stringify(payload));return true}
function send(type){if(sendPayload({type}))val('message','Comando enviado: '+type,0)}
function addTrace(d){traceSamples.push(d);val('traceSamples',traceSamples.length,0);const elapsed=d.t/1000;val('traceRate',elapsed>0?(traceSamples.length/elapsed).toFixed(1)+' Hz':'0 Hz',0);scheduleDraw();if(traceSamples.length%25===0)updateStats()}
function scheduleDraw(){if(drawPending)return;drawPending=true;requestAnimationFrame(()=>{drawPending=false;drawCharts()})}
function prepareCanvas(canvas){const ratio=window.devicePixelRatio||1,w=Math.max(320,canvas.clientWidth),h=canvas.clientHeight;canvas.width=w*ratio;canvas.height=h*ratio;const ctx=canvas.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);return{ctx,w,h}}
function drawAxes(ctx,w,h,minY,maxY,maxX,title){ctx.clearRect(0,0,w,h);ctx.strokeStyle='#26464e';ctx.fillStyle='#91b0b7';ctx.font='11px Arial';ctx.lineWidth=1;const l=48,r=12,t=24,b=28;for(let i=0;i<=4;i++){const y=t+(h-t-b)*i/4;ctx.beginPath();ctx.moveTo(l,y);ctx.lineTo(w-r,y);ctx.stroke();const v=maxY-(maxY-minY)*i/4;ctx.fillText(v.toFixed(1),4,y+4)}for(let i=0;i<=5;i++){const x=l+(w-l-r)*i/5;ctx.beginPath();ctx.moveTo(x,t);ctx.lineTo(x,h-b);ctx.stroke();ctx.fillText((maxX*i/5).toFixed(1),x-8,h-8)}ctx.fillStyle='#eff9fa';ctx.fillText(title,l,15);return{l,r,t,b}}
function drawLine(ctx,points,key,color,bounds,w,h,maxX,minY,maxY){if(points.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.beginPath();points.forEach((p,i)=>{const x=bounds.l+(w-bounds.l-bounds.r)*(p.t/1000)/maxX;const y=bounds.t+(h-bounds.t-bounds.b)*(maxY-p[key])/(maxY-minY);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});ctx.stroke()}
function drawCharts(){const visible=series.filter(s=>$(s.toggle).checked),angleValues=[];for(const p of traceSamples)for(const s of visible)angleValues.push(p[s.key]);let min=Math.min(...angleValues),max=Math.max(...angleValues);if(!isFinite(min)){min=-1;max=1}if(max-min<0.5){const c=(max+min)/2;min=c-.25;max=c+.25}else{const pad=(max-min)*.08;min-=pad;max+=pad}const maxX=Math.max(1,traceSamples.length?traceSamples[traceSamples.length-1].t/1000:1);let c=prepareCanvas($('angleChart')),b=drawAxes(c.ctx,c.w,c.h,min,max,maxX,'Pitch (deg)');for(const s of visible)drawLine(c.ctx,traceSamples,s.key,s.color,b,c.w,c.h,maxX,min,max);const rates=traceSamples.map(p=>p.gyroRate);let rmin=Math.min(...rates),rmax=Math.max(...rates);if(!isFinite(rmin)){rmin=-1;rmax=1}if(rmax-rmin<1){const m=(rmax+rmin)/2;rmin=m-.5;rmax=m+.5}c=prepareCanvas($('rateChart'));b=drawAxes(c.ctx,c.w,c.h,rmin,rmax,maxX,'GY corregido (deg/s)');drawLine(c.ctx,traceSamples,'gyroRate','#66e3a4',b,c.w,c.h,maxX,rmin,rmax)}
function stats(key){const values=traceSamples.map(p=>p[key]);if(!values.length)return null;const mean=values.reduce((a,b)=>a+b,0)/values.length;const variance=values.reduce((a,b)=>a+(b-mean)**2,0)/values.length;return{mean,std:Math.sqrt(variance),min:Math.min(...values),max:Math.max(...values),drift:values[values.length-1]-values[0]}}
function updateStats(){$('statsBody').innerHTML=series.map(s=>{const x=stats(s.key);return x?`<tr><td style="color:${s.color}">${s.name}</td><td>${x.mean.toFixed(3)}</td><td>${x.std.toFixed(3)}</td><td>${x.min.toFixed(3)}</td><td>${x.max.toFixed(3)}</td><td>${(x.max-x.min).toFixed(3)}</td><td>${x.drift.toFixed(3)}</td></tr>`:''}).join('')||'<tr><td colspan="7">Sin muestras</td></tr>'}
function clearTrace(){traceSamples=[];val('traceSamples',0,0);val('traceRate','0 Hz',0);updateStats();drawCharts()}
function exportCsv(){if(!traceSamples.length)return;const rows=['time_ms,accel_pitch_deg,gyro_pitch_deg,filtered_pitch_deg,gyro_y_dps,accel_norm_g,alpha',...traceSamples.map(p=>[p.t,p.accel,p.gyro,p.filtered,p.gyroRate,p.accelNorm,p.alpha].join(','))];const blob=new Blob([rows.join('\n')],{type:'text/csv'}),a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='mpu9250_filtro_'+Date.now()+'.csv';a.click();URL.revokeObjectURL(a.href)}
let ws;function connect(){ws=new WebSocket('ws://'+location.hostname+':81/');ws.onopen=()=>{$('dot').className='dot on';val('connection','WebSocket conectado',0)};ws.onclose=()=>{$('dot').className='dot';val('connection','Reconectando...',0);setTimeout(connect,1000)};ws.onerror=()=>val('message','Error de WebSocket',0);ws.onmessage=e=>{const d=JSON.parse(e.data);if(d.type==='state')render(d);else if(d.type==='trace')addTrace(d);else if(d.type==='trace_end'){traceActive=false;val('traceStatus','COMPLETA',0);updateStats();scheduleDraw()}else if(d.type==='i2c_scan')val('scanResult',d.devices.length?'Encontrados: '+d.devices.join(', '):'Ningun dispositivo',0);else if(d.type==='ack'||d.type==='error')val('message',d.message,0)}}
$('gyroCal').addEventListener('click',()=>send('calibrate_gyro'));$('magCal').addEventListener('click',()=>send('calibrate_mag'));$('accelStart').addEventListener('click',()=>send('accel_start'));$('accelCapture').addEventListener('click',()=>send('accel_capture'));$('setVertical').addEventListener('click',()=>send('set_vertical'));$('clearCal').addEventListener('click',()=>send('clear_calibration'));$('scan').addEventListener('click',()=>send('scan_i2c'));
$('filterAlpha').addEventListener('input',()=>alphaDirty=true);$('applyAlpha').addEventListener('click',()=>{sendPayload({type:'set_filter_alpha',alpha:Number($('filterAlpha').value)});alphaDirty=false});$('saveAlpha').addEventListener('click',()=>{sendPayload({type:'save_filter_alpha',alpha:Number($('filterAlpha').value)});alphaDirty=false});$('resetAlpha').addEventListener('click',()=>{send('reset_filter_alpha');alphaDirty=false});
$('traceStart').addEventListener('click',()=>{const seconds=Math.min(300,Math.max(5,Number($('traceDuration').value)||30));$('traceDuration').value=seconds;clearTrace();sendPayload({type:'trace_start',durationMs:seconds*1000})});$('tracePause').addEventListener('click',()=>send(tracePaused?'trace_resume':'trace_pause'));$('traceStop').addEventListener('click',()=>{send('trace_stop');updateStats();scheduleDraw()});$('traceClear').addEventListener('click',clearTrace);$('traceCsv').addEventListener('click',exportCsv);series.forEach(s=>$(s.toggle).addEventListener('change',scheduleDraw));window.addEventListener('resize',scheduleDraw);drawCharts();connect();
</script></body></html>)rawliteral";

void disableMotorOutputs() {
  const int pins[] = {Config::PIN_MOTOR_LEFT_ENABLE_PWM, Config::PIN_MOTOR_LEFT_IN1,
                      Config::PIN_MOTOR_LEFT_IN2, Config::PIN_MOTOR_RIGHT_ENABLE_PWM,
                      Config::PIN_MOTOR_RIGHT_IN1, Config::PIN_MOTOR_RIGHT_IN2};
  for (int pin : pins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
}

uint8_t readI2cRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1) return 0;
  return Wire.read();
}

String hexByte(uint8_t value) {
  String text = "0x";
  if (value < 0x10) text += '0';
  text += String(value, HEX);
  return text;
}

void initializeSensor() {
  imuAt68.setWire(&Wire);
  imuAt69.setWire(&Wire);
  uint8_t idAt68 = 0;
  uint8_t idAt69 = 0;
  const bool foundAt68 = imuAt68.readId(&idAt68) == 0;
  const bool foundAt69 = imuAt69.readId(&idAt69) == 0;
  if (foundAt68 && (idAt68 == 0x71 || idAt68 == 0x73)) {
    imu = &imuAt68; imuId = idAt68; imuAddress = MPU9250_ADDRESS_AD0_LOW;
  } else if (foundAt69 && (idAt69 == 0x71 || idAt69 == 0x73)) {
    imu = &imuAt69; imuId = idAt69; imuAddress = MPU9250_ADDRESS_AD0_HIGH;
  }
  imuReady = imu != nullptr;
  if (!imuReady) {
    calibrationStatus = "MPU9250 no detectado";
    return;
  }
  imu->beginAccel(ACC_FULL_SCALE_4_G);
  imu->beginGyro(GYRO_FULL_SCALE_500_DPS);
  imu->beginMag(MAG_MODE_CONTINUOUS_100HZ_16BIT);
  magId = readI2cRegister(AK8963_ADDRESS, AK8963_WHO_AM_I);
  magReady = magId == 0x48 && imu->magUpdate() == 0;
  Serial.printf("MPU9250 %s at %s, AK8963 %s\n", hexByte(imuId).c_str(),
                hexByte(imuAddress).c_str(), hexByte(magId).c_str());
}

void applyCalibration() {
  const Imu9250Calibration::Data &data = Imu9250Calibration::get();
  for (uint8_t axis = 0; axis < 3; ++axis) {
    accelCorrected[axis] = (accelRaw[axis] - data.accelOffset[axis]) * data.accelScale[axis];
    gyroCorrected[axis] = gyroRaw[axis] - data.gyroOffset[axis];
    magCorrectedUt[axis] = (magRaw[axis] - data.magOffset[axis]) * data.magScale[axis] * MAG_16BIT_UT_PER_COUNT;
  }
  accelNorm = sqrtf(accelCorrected[0] * accelCorrected[0] + accelCorrected[1] * accelCorrected[1] + accelCorrected[2] * accelCorrected[2]);
  magneticNormUt = sqrtf(magCorrectedUt[0] * magCorrectedUt[0] + magCorrectedUt[1] * magCorrectedUt[1] + magCorrectedUt[2] * magCorrectedUt[2]);
}

void resetFilterState() {
  filterInitialized = false;
  lastFilterUs = micros();
}

void updateOrientation() {
  if (!accelReady || !gyroReady) return;
  accelRollDeg = atan2f(accelCorrected[1], accelCorrected[2]) * RAD_TO_DEG_F;
  accelPitchDeg = atan2f(-accelCorrected[0], sqrtf(accelCorrected[1] * accelCorrected[1] + accelCorrected[2] * accelCorrected[2])) * RAD_TO_DEG_F;
  const unsigned long nowUs = micros();
  if (!filterInitialized) {
    filteredRollDeg = accelRollDeg;
    filteredPitchDeg = accelPitchDeg;
    filterInitialized = true;
  } else {
    const float dt = constrain((nowUs - lastFilterUs) / 1000000.0f, 0.0005f, 0.05f);
    filteredRollDeg = filterAlpha * (filteredRollDeg + gyroCorrected[0] * dt) + (1.0f - filterAlpha) * accelRollDeg;
    filteredPitchDeg = filterAlpha * (filteredPitchDeg + gyroCorrected[1] * dt) + (1.0f - filterAlpha) * accelPitchDeg;
  }
  lastFilterUs = nowUs;

  if (traceActive && !tracePaused) {
    if (lastGyroIntegrationUs != 0) {
      const float traceDt = constrain((nowUs - lastGyroIntegrationUs) / 1000000.0f, 0.0005f, 0.05f);
      gyroOnlyPitchDeg += gyroCorrected[1] * traceDt;
    }
    lastGyroIntegrationUs = nowUs;
  }

  if (magReady) {
    const float roll = filteredRollDeg * DEG_TO_RAD_F;
    const float pitch = filteredPitchDeg * DEG_TO_RAD_F;
    const float horizontalX = magCorrectedUt[0] * cosf(pitch) + magCorrectedUt[2] * sinf(pitch);
    const float horizontalY = magCorrectedUt[0] * sinf(roll) * sinf(pitch) + magCorrectedUt[1] * cosf(roll) - magCorrectedUt[2] * sinf(roll) * cosf(pitch);
    headingDeg = atan2f(horizontalY, horizontalX) * RAD_TO_DEG_F;
    if (headingDeg < 0.0f) headingDeg += 360.0f;
  }
}

void updateSensor() {
  if (imu == nullptr) return;
  freshAccel = imu->accelUpdate() == 0;
  freshGyro = imu->gyroUpdate() == 0;
  freshMag = magId == 0x48 && imu->magUpdate() == 0;
  if (freshAccel) {
    accelRaw[0] = imu->accelX(); accelRaw[1] = imu->accelY(); accelRaw[2] = imu->accelZ();
    accelReady = true; ++accelReads;
  }
  if (freshGyro) {
    gyroRaw[0] = imu->gyroX(); gyroRaw[1] = imu->gyroY(); gyroRaw[2] = imu->gyroZ();
    gyroReady = true; ++gyroReads;
  }
  if (freshMag) {
    magRaw[0] = imu->magX(); magRaw[1] = imu->magY(); magRaw[2] = imu->magZ();
    magReady = true; ++magReads;
  }
  applyCalibration();
  updateOrientation();
}

void resetSampleAccumulator() {
  calibrationSamples = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) sampleSum[axis] = sampleSquareSum[axis] = 0.0;
}

void startGyroCalibration(bool automatic) {
  if (calibrationMode != CalibrationMode::None || !imuReady) return;
  calibrationMode = CalibrationMode::Gyro;
  automaticGyroCalibration = automatic;
  calibrationStartMs = millis();
  resetSampleAccumulator();
  calibrationStatus = automatic ? "Verificando bias del gyro" : "Gyro: mantenga el sensor inmovil";
}

void finishGyroCalibration() {
  calibrationMode = CalibrationMode::None;
  if (calibrationSamples < 100) {
    calibrationStatus = "Gyro FAIL: pocas muestras";
    return;
  }
  float mean[3];
  float maximumDeviation = 0.0f;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    mean[axis] = sampleSum[axis] / calibrationSamples;
    const float variance = max(0.0, sampleSquareSum[axis] / calibrationSamples - mean[axis] * mean[axis]);
    maximumDeviation = max(maximumDeviation, sqrtf(variance));
  }
  if (maximumDeviation > 1.5f) {
    calibrationStatus = automaticGyroCalibration ? "Bias guardado conservado: sensor en movimiento" : "Gyro FAIL: sensor en movimiento";
    return;
  }
  Imu9250Calibration::Data data = Imu9250Calibration::get();
  for (uint8_t axis = 0; axis < 3; ++axis) data.gyroOffset[axis] = mean[axis];
  data.validFlags |= Imu9250Calibration::VALID_GYRO;
  calibrationStatus = Imu9250Calibration::save(data) ? "Gyro PASS y guardado" : "Gyro PASS, error al guardar";
}

void startAccelWizard() {
  if (calibrationMode != CalibrationMode::None) return;
  accelWizardActive = true;
  accelPoseIndex = 0;
  calibrationStatus = String("Acelerometro: coloque ") + ACCEL_POSE_NAMES[0];
}

void startAccelPoseCapture() {
  if (!accelWizardActive || calibrationMode != CalibrationMode::None || !accelReady) return;
  calibrationMode = CalibrationMode::AccelPose;
  calibrationStartMs = millis();
  resetSampleAccumulator();
  calibrationStatus = String("Capturando ") + ACCEL_POSE_NAMES[accelPoseIndex];
}

bool accelPoseIsValid(const float mean[3]) {
  const uint8_t axis = accelPoseIndex / 2;
  const float expectedSign = accelPoseIndex % 2 == 0 ? 1.0f : -1.0f;
  if (mean[axis] * expectedSign < 0.7f) return false;
  for (uint8_t other = 0; other < 3; ++other) {
    if (other != axis && fabsf(mean[other]) > 0.55f) return false;
  }
  return true;
}

void finishAccelPoseCapture() {
  calibrationMode = CalibrationMode::None;
  if (calibrationSamples < 100) {
    calibrationStatus = "Acelerometro FAIL: pocas muestras";
    return;
  }
  float mean[3];
  for (uint8_t axis = 0; axis < 3; ++axis) mean[axis] = sampleSum[axis] / calibrationSamples;
  if (!accelPoseIsValid(mean)) {
    calibrationStatus = String("Posicion incorrecta: ") + ACCEL_POSE_NAMES[accelPoseIndex];
    return;
  }
  for (uint8_t axis = 0; axis < 3; ++axis) accelPoseMean[accelPoseIndex][axis] = mean[axis];
  ++accelPoseIndex;
  if (accelPoseIndex < ACCEL_POSE_COUNT) {
    calibrationStatus = String("Posicion guardada. Siguiente: ") + ACCEL_POSE_NAMES[accelPoseIndex];
    return;
  }

  Imu9250Calibration::Data data = Imu9250Calibration::get();
  for (uint8_t axis = 0; axis < 3; ++axis) {
    const float positive = accelPoseMean[axis * 2][axis];
    const float negative = accelPoseMean[axis * 2 + 1][axis];
    const float span = positive - negative;
    if (span < 1.5f) {
      calibrationStatus = "Acelerometro FAIL: rango insuficiente";
      accelWizardActive = false;
      return;
    }
    data.accelOffset[axis] = (positive + negative) * 0.5f;
    data.accelScale[axis] = 2.0f / span;
  }
  data.validFlags |= Imu9250Calibration::VALID_ACCEL;
  accelWizardActive = false;
  calibrationStatus = Imu9250Calibration::save(data) ? "Acelerometro PASS y guardado" : "Acelerometro PASS, error al guardar";
}

void startMagCalibration() {
  if (calibrationMode != CalibrationMode::None || !magReady) return;
  calibrationMode = CalibrationMode::Magnetometer;
  calibrationStartMs = millis();
  lastMagSampleMs = 0;
  calibrationSamples = 0;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    magMinimum[axis] = INFINITY;
    magMaximum[axis] = -INFINITY;
  }
  calibrationStatus = "Mag 0%: gire sobre X, Y y Z";
}

void finishMagCalibration() {
  calibrationMode = CalibrationMode::None;
  float halfRange[3];
  for (uint8_t axis = 0; axis < 3; ++axis) halfRange[axis] = (magMaximum[axis] - magMinimum[axis]) * 0.5f;
  if (calibrationSamples < 100 || halfRange[0] < 20.0f || halfRange[1] < 20.0f || halfRange[2] < 20.0f) {
    calibrationStatus = "Mag FAIL: movimiento insuficiente en los 3 ejes";
    return;
  }
  const float averageRange = (halfRange[0] + halfRange[1] + halfRange[2]) / 3.0f;
  Imu9250Calibration::Data data = Imu9250Calibration::get();
  for (uint8_t axis = 0; axis < 3; ++axis) {
    data.magOffset[axis] = (magMaximum[axis] + magMinimum[axis]) * 0.5f;
    data.magScale[axis] = averageRange / halfRange[axis];
  }
  data.validFlags |= Imu9250Calibration::VALID_MAG;
  calibrationStatus = Imu9250Calibration::save(data) ? "Mag PASS y guardado" : "Mag PASS, error al guardar";
}

void updateCalibration() {
  const unsigned long now = millis();
  if (calibrationMode == CalibrationMode::Gyro) {
    if (freshGyro) {
      for (uint8_t axis = 0; axis < 3; ++axis) {
        sampleSum[axis] += gyroRaw[axis];
        sampleSquareSum[axis] += gyroRaw[axis] * gyroRaw[axis];
      }
      ++calibrationSamples;
    }
    if (now - calibrationStartMs >= Config::SENSOR_GYRO_CALIBRATION_MS) finishGyroCalibration();
  } else if (calibrationMode == CalibrationMode::AccelPose) {
    if (freshAccel) {
      for (uint8_t axis = 0; axis < 3; ++axis) sampleSum[axis] += accelRaw[axis];
      ++calibrationSamples;
    }
    if (now - calibrationStartMs >= Config::SENSOR_ACCEL_POSE_CAPTURE_MS) finishAccelPoseCapture();
  } else if (calibrationMode == CalibrationMode::Magnetometer) {
    const unsigned long elapsed = now - calibrationStartMs;
    if (freshMag && now - lastMagSampleMs >= 20) {
      lastMagSampleMs = now;
      for (uint8_t axis = 0; axis < 3; ++axis) {
        magMinimum[axis] = min(magMinimum[axis], magRaw[axis]);
        magMaximum[axis] = max(magMaximum[axis], magRaw[axis]);
      }
      ++calibrationSamples;
    }
    if (elapsed >= Config::SENSOR_MAG_CALIBRATION_MS) finishMagCalibration();
    else calibrationStatus = String("Mag ") + String(elapsed * 100UL / Config::SENSOR_MAG_CALIBRATION_MS) + "%: gire sobre X, Y y Z";
  }
}

const char *calibrationModeName() {
  if (calibrationMode == CalibrationMode::Gyro) return "gyro";
  if (calibrationMode == CalibrationMode::AccelPose) return "accelerometer";
  if (calibrationMode == CalibrationMode::Magnetometer) return "magnetometer";
  return "idle";
}

String stateAsJson() {
  const Imu9250Calibration::Data &data = Imu9250Calibration::get();
  JsonDocument doc;
  doc["type"] = "state";
  doc["imuReady"] = imuReady; doc["accelReady"] = accelReady; doc["gyroReady"] = gyroReady; doc["magReady"] = magReady;
  doc["imuId"] = imuReady ? hexByte(imuId) : "--"; doc["magId"] = magReady ? hexByte(magId) : "--"; doc["imuAddress"] = imuReady ? hexByte(imuAddress) : "--";
  JsonArray ar = doc["accelRaw"].to<JsonArray>(); JsonArray ac = doc["accelCorrected"].to<JsonArray>();
  JsonArray gr = doc["gyroRaw"].to<JsonArray>(); JsonArray gc = doc["gyroCorrected"].to<JsonArray>();
  JsonArray mr = doc["magRaw"].to<JsonArray>(); JsonArray mc = doc["magCorrected"].to<JsonArray>();
  JsonArray ao = doc["accelOffset"].to<JsonArray>(); JsonArray as = doc["accelScale"].to<JsonArray>();
  JsonArray go = doc["gyroOffset"].to<JsonArray>(); JsonArray mo = doc["magOffset"].to<JsonArray>(); JsonArray ms = doc["magScale"].to<JsonArray>();
  for (uint8_t axis = 0; axis < 3; ++axis) {
    ar.add(accelRaw[axis]); ac.add(accelCorrected[axis]); gr.add(gyroRaw[axis]); gc.add(gyroCorrected[axis]);
    mr.add(magRaw[axis]); mc.add(magCorrectedUt[axis]); ao.add(data.accelOffset[axis]); as.add(data.accelScale[axis]);
    go.add(data.gyroOffset[axis]); mo.add(data.magOffset[axis]); ms.add(data.magScale[axis]);
  }
  doc["accelNorm"] = accelNorm; doc["magNorm"] = magneticNormUt; doc["roll"] = filteredRollDeg; doc["pitch"] = filteredPitchDeg;
  doc["relativePitch"] = filteredPitchDeg - data.verticalPitchDeg; doc["heading"] = headingDeg; doc["filterReady"] = filterInitialized;
  doc["accelRate"] = accelRateHz; doc["gyroRate"] = gyroRateHz; doc["magRate"] = magRateHz;
  doc["calibrationMode"] = calibrationModeName(); doc["calibrationStatus"] = calibrationStatus; doc["calibrationSamples"] = calibrationSamples;
  doc["calibrationStored"] = Imu9250Calibration::wasLoaded();
  doc["accelCalibrated"] = Imu9250Calibration::isValid(Imu9250Calibration::VALID_ACCEL);
  doc["gyroCalibrated"] = Imu9250Calibration::isValid(Imu9250Calibration::VALID_GYRO);
  doc["magCalibrated"] = Imu9250Calibration::isValid(Imu9250Calibration::VALID_MAG);
  doc["verticalCalibrated"] = Imu9250Calibration::isValid(Imu9250Calibration::VALID_VERTICAL);
  doc["verticalRoll"] = data.verticalRollDeg; doc["verticalPitch"] = data.verticalPitchDeg;
  doc["accelWizardActive"] = accelWizardActive; doc["accelPoseIndex"] = accelPoseIndex;
  doc["accelPoseName"] = accelWizardActive && accelPoseIndex < ACCEL_POSE_COUNT ? ACCEL_POSE_NAMES[accelPoseIndex] : "--";
  doc["filterAlpha"] = filterAlpha;
  doc["filterAlphaStored"] = Imu9250FilterSettings::wasLoaded();
  doc["traceActive"] = traceActive;
  doc["tracePaused"] = tracePaused;
  doc["traceDurationMs"] = traceDurationMs;
  doc["otaAvailable"] = ota_isAvailable(); doc["otaUpdating"] = ota_isUpdating();
  String json; serializeJson(doc, json); return json;
}

String scanAsJson() {
  JsonDocument doc; doc["type"] = "i2c_scan"; JsonArray devices = doc["devices"].to<JsonArray>();
  for (uint8_t address = 1; address < 127; ++address) { Wire.beginTransmission(address); if (Wire.endTransmission() == 0) devices.add(hexByte(address)); }
  String json; serializeJson(doc, json); return json;
}

void sendMessage(uint8_t clientId, const char *type, const String &message) {
  JsonDocument doc; doc["type"] = type; doc["message"] = message; String json; serializeJson(doc, json); webSocket.sendTXT(clientId, json);
}

void saveVertical() {
  Imu9250Calibration::Data data = Imu9250Calibration::get();
  data.verticalRollDeg = filteredRollDeg; data.verticalPitchDeg = filteredPitchDeg;
  data.validFlags |= Imu9250Calibration::VALID_VERTICAL;
  calibrationStatus = Imu9250Calibration::save(data) ? "Vertical guardada" : "Error guardando vertical";
}

void startTrace(unsigned long requestedDurationMs) {
  traceDurationMs = constrain(requestedDurationMs, Config::SENSOR_TRACE_MIN_DURATION_MS,
                              Config::SENSOR_TRACE_MAX_DURATION_MS);
  traceActive = true;
  tracePaused = false;
  traceStartMs = millis();
  tracePausedTotalMs = 0;
  tracePauseStartMs = 0;
  lastTraceSendMs = 0;
  traceSequence = 0;
  gyroOnlyPitchDeg = filteredPitchDeg;
  lastGyroIntegrationUs = micros();
}

void pauseTrace() {
  if (!traceActive || tracePaused) return;
  tracePaused = true;
  tracePauseStartMs = millis();
}

void resumeTrace() {
  if (!traceActive || !tracePaused) return;
  tracePausedTotalMs += millis() - tracePauseStartMs;
  tracePaused = false;
  lastGyroIntegrationUs = micros();
}

void stopTrace() {
  traceActive = false;
  tracePaused = false;
}

void handleWebSocketText(uint8_t clientId, const uint8_t *payload, size_t length) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, length)) { sendMessage(clientId, "error", "JSON invalido"); return; }
  const char *type = doc["type"] | "";
  if (strcmp(type, "scan_i2c") == 0) { String json = scanAsJson(); webSocket.sendTXT(clientId, json); return; }
  if (strcmp(type, "trace_start") == 0) {
    if (calibrationMode != CalibrationMode::None || !filterInitialized) { sendMessage(clientId, "error", "Sensor no disponible para captura"); return; }
    startTrace(doc["durationMs"] | 30000UL); sendMessage(clientId, "ack", "Captura iniciada"); return;
  }
  if (strcmp(type, "trace_pause") == 0) { pauseTrace(); sendMessage(clientId, "ack", "Captura pausada"); return; }
  if (strcmp(type, "trace_resume") == 0) { resumeTrace(); sendMessage(clientId, "ack", "Captura reanudada"); return; }
  if (strcmp(type, "trace_stop") == 0) { stopTrace(); sendMessage(clientId, "ack", "Captura detenida"); return; }
  if (strcmp(type, "set_filter_alpha") == 0 || strcmp(type, "save_filter_alpha") == 0) {
    const float requestedAlpha = doc["alpha"] | NAN;
    if (!isfinite(requestedAlpha) || requestedAlpha < Config::SENSOR_FILTER_ALPHA_MIN || requestedAlpha > Config::SENSOR_FILTER_ALPHA_MAX) {
      sendMessage(clientId, "error", "Alpha fuera de rango 0.80-0.999"); return;
    }
    filterAlpha = requestedAlpha;
    resetFilterState();
    if (strcmp(type, "save_filter_alpha") == 0 && !Imu9250FilterSettings::save(filterAlpha)) {
      sendMessage(clientId, "error", "No se pudo guardar alpha"); return;
    }
    sendMessage(clientId, "ack", strcmp(type, "save_filter_alpha") == 0 ? "Alpha aplicado y guardado" : "Alpha aplicado temporalmente");
    return;
  }
  if (strcmp(type, "reset_filter_alpha") == 0) {
    Imu9250FilterSettings::clear(Config::SENSOR_COMPLEMENTARY_ALPHA);
    filterAlpha = Config::SENSOR_COMPLEMENTARY_ALPHA;
    resetFilterState();
    sendMessage(clientId, "ack", "Alpha predeterminado restaurado"); return;
  }
  if (calibrationMode != CalibrationMode::None) { sendMessage(clientId, "error", "Calibracion en curso"); return; }
  if (strcmp(type, "calibrate_gyro") == 0) startGyroCalibration(false);
  else if (strcmp(type, "calibrate_mag") == 0) startMagCalibration();
  else if (strcmp(type, "accel_start") == 0) startAccelWizard();
  else if (strcmp(type, "accel_capture") == 0) startAccelPoseCapture();
  else if (strcmp(type, "set_vertical") == 0) saveVertical();
  else if (strcmp(type, "clear_calibration") == 0) {
    Imu9250Calibration::clear(); accelWizardActive = false; calibrationStatus = "Calibracion NVS borrada";
  } else { sendMessage(clientId, "error", "Comando desconocido"); return; }
  sendMessage(clientId, "ack", calibrationStatus);
}

void onWebSocketEvent(uint8_t clientId, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) { String json = stateAsJson(); webSocket.sendTXT(clientId, json); }
  else if (type == WStype_TEXT) handleWebSocketText(clientId, payload, length);
}

void startNetwork() {
  WiFi.mode(WIFI_STA); WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
  const unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < Config::WIFI_CONNECT_TIMEOUT_MS) delay(250);
  if (WiFi.status() == WL_CONNECTED) { Serial.print(F("Dashboard: http://")); Serial.println(WiFi.localIP()); ota_begin(Config::OTA_HOSTNAME, Config::OTA_PASSWORD); }
  else { WiFi.mode(WIFI_AP); WiFi.softAP("robot-balancin-sensor"); Serial.print(F("Dashboard AP: http://")); Serial.println(WiFi.softAPIP()); }
  server.on(F("/"), HTTP_GET, []() { server.send_P(200, PSTR("text/html"), DASHBOARD); });
  server.begin(); webSocket.begin(); webSocket.onEvent(onWebSocketEvent);
}

void updateRates() {
  const unsigned long now = millis(); if (now - lastRateMs < 1000) return;
  const unsigned long elapsed = lastRateMs == 0 ? 1000 : now - lastRateMs;
  accelRateHz = (accelReads - previousAccelReads) * 1000UL / elapsed;
  gyroRateHz = (gyroReads - previousGyroReads) * 1000UL / elapsed;
  magRateHz = (magReads - previousMagReads) * 1000UL / elapsed;
  previousAccelReads = accelReads; previousGyroReads = gyroReads; previousMagReads = magReads; lastRateMs = now;
}

void sendStateIfDue() {
  const unsigned long now = millis();
  const unsigned long intervalMs = traceActive ? 500UL : Config::WS_STATE_INTERVAL_MS;
  if (now - lastStateSendMs < intervalMs) return;
  lastStateSendMs = now; String json = stateAsJson(); webSocket.broadcastTXT(json);
}

void sendTraceIfDue() {
  if (!traceActive || tracePaused || !filterInitialized) return;
  const unsigned long now = millis();
  const unsigned long elapsed = now - traceStartMs - tracePausedTotalMs;
  if (elapsed >= traceDurationMs) {
    stopTrace();
    JsonDocument endDoc;
    endDoc["type"] = "trace_end";
    endDoc["samples"] = traceSequence;
    String json;
    serializeJson(endDoc, json);
    webSocket.broadcastTXT(json);
    return;
  }
  if (now - lastTraceSendMs < Config::SENSOR_TRACE_INTERVAL_MS) return;
  lastTraceSendMs = now;
  const Imu9250Calibration::Data &data = Imu9250Calibration::get();
  JsonDocument doc;
  doc["type"] = "trace";
  doc["seq"] = traceSequence++;
  doc["t"] = elapsed;
  doc["accel"] = accelPitchDeg - data.verticalPitchDeg;
  doc["gyro"] = gyroOnlyPitchDeg - data.verticalPitchDeg;
  doc["filtered"] = filteredPitchDeg - data.verticalPitchDeg;
  doc["gyroRate"] = gyroCorrected[1];
  doc["accelNorm"] = accelNorm;
  doc["alpha"] = filterAlpha;
  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
}

void printIfDue() {
  const unsigned long now = millis(); if (now - lastSerialPrintMs < 500) return; lastSerialPrintMs = now;
  Serial.printf("A %.3f %.3f %.3f | G %.2f %.2f %.2f | M %.1f %.1f %.1f uT | R %.1f P %.1f H %.1f\n",
                accelCorrected[0], accelCorrected[1], accelCorrected[2], gyroCorrected[0], gyroCorrected[1], gyroCorrected[2],
                magCorrectedUt[0], magCorrectedUt[1], magCorrectedUt[2], filteredRollDeg, filteredPitchDeg, headingDeg);
}

}  // namespace

namespace Mpu9250NineAxisTest {

void begin() {
  disableMotorOutputs();
  Serial.begin(Config::SERIAL_BAUD_RATE);
  delay(300);
  Imu9250Calibration::begin();
  Imu9250FilterSettings::begin(Config::SENSOR_COMPLEMENTARY_ALPHA);
  filterAlpha = Imu9250FilterSettings::alpha();
  Wire.begin(Config::PIN_I2C_SDA, Config::PIN_I2C_SCL);
  Wire.setClock(Config::I2C_CLOCK_HZ);
  initializeSensor();
  startNetwork();
  lastRateMs = millis();
  if (Imu9250Calibration::isValid(Imu9250Calibration::VALID_GYRO)) startGyroCalibration(true);
  else calibrationStatus = "Sin bias de gyro guardado: calibre manualmente";
}

void update() {
  disableMotorOutputs();
  server.handleClient();
  webSocket.loop();
  ota_handle();
  updateSensor();
  updateCalibration();
  updateRates();
  sendStateIfDue();
  sendTraceIfDue();
  if (!traceActive) printIfDue();
  delay(traceActive ? 1 : 5);
}

}  // namespace Mpu9250NineAxisTest
