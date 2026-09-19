#include <math.h>
#include <Arduino.h> 
#include <WiFi.h>
#include <WebServer.h>

// --- CONFIGURACIÓN WIFI ---
const char* ssid = "iPhone de María";
const char* password = "OnX9-P97t-IiTH-Q84f";

// --- CONFIGURACIÓN IP ---
IPAddress local_IP(172, 20, 10, 5);  
IPAddress gateway(172, 20, 10, 1);   
IPAddress subnet(255, 255, 255, 240); 
IPAddress primaryDNS(8, 8, 8, 8); 

WebServer server(80);

// --- VARIABLES CONTROL ---
bool usar_sensores = false; 
bool modo_manual = false; 
const int MAX_VELOCIDAD_MANUAL = 600; 

int pwm_manual_izq = 0;
int pwm_manual_der = 0;

// --- POSICIÓN ROBOT ---
double pos_x_robot = 0.0;
double pos_y_robot = 0.0;
double angulo_theta_robot = 0.0;

// --- ESTRUCTURA MOTORES ---
struct InfoMotor {
  int pin_fase_A; int pin_fase_B; int pin_sensor_A; int pin_sensor_B;    
  volatile long ticks_actuales; long ticks_anteriores_vel; long ticks_anteriores_odom;
  double velocidad_bruta; double velocidad_suavizada;  
  double error_anterior; double error_acumulado; double derivada_suavizada; 
  double potencia_pwm;         
  double ganancia_Kp; double ganancia_Ki; double ganancia_Kd;
  bool esta_bloqueado;
};

// --- CONSTANTES ---
const double RADIO_RUEDA = 0.0335;      
const double DISTANCIA_ENTRE_RUEDAS = 0.2175;   
const double TICKS_POR_VUELTA = 1500.0;
const double TIEMPO_MUESTREO = 0.01; 

const double LONGITUD_RECTA_T5 = 0.5; 
const double RADIO_CURVA_T5 = 0.25; 
double objetivo_vel_motor1 = 0; double objetivo_vel_motor2 = 0; 
double comando_lineal_v = 0.0; double comando_angular_w = 0.0; 
int paso_secuencia = 0; 
int fase_tarea5 = 0; 

unsigned long tiempo_inicio_estado = 0;
unsigned long tiempo_acumulado_tarea = 0; 
bool en_pausa_obstaculo = false;
bool esperando_reinicio = false;
double start_x = 0; double start_y = 0; double theta_referencia = 0; 
bool estado_recien_iniciado = true; 

// --- PINES ---
const int PIN_M1_PWM_A = 32; const int PIN_M1_PWM_B = 33; const int PIN_M1_SENSOR_A = 25; const int PIN_M1_SENSOR_B = 26; 
const int PIN_M2_PWM_A = 19; const int PIN_M2_PWM_B = 18; const int PIN_M2_SENSOR_A = 27; const int PIN_M2_SENSOR_B = 14;
const int PIN_SENSOR_1 = 34; const int PIN_SENSOR_2 = 35; const int PIN_SENSOR_3 = 39; 
const int UMBRAL_OBSTACULO = 300; 

// PID
const double VALOR_KP = 0.17661; const double VALOR_KI = 5.0; const double VALOR_KD = 0.005;
const double FACTOR_SUAVIZADO_DERIVADA = 0.85;

InfoMotor motor_derecho   = {PIN_M1_PWM_A, PIN_M1_PWM_B, PIN_M1_SENSOR_A, PIN_M1_SENSOR_B, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, VALOR_KP, VALOR_KI, VALOR_KD, false};
InfoMotor motor_izquierdo = {PIN_M2_PWM_A, PIN_M2_PWM_B, PIN_M2_SENSOR_A, PIN_M2_SENSOR_B, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, VALOR_KP, VALOR_KI, VALOR_KD, false};

const int frecuencia_pwm = 1000; const int resolucion_pwm = 10;
double tiempo_muestreo_seg = 0.01; double factor_suavizado_vel = 0.7; 
hw_timer_t *temporizador = NULL;
volatile bool hay_datos_nuevos = false;

// --- INTERRUPCIONES ---
void IRAM_ATTR ISR_MD_A() { if (digitalRead(motor_derecho.pin_sensor_A) == digitalRead(motor_derecho.pin_sensor_B)) motor_derecho.ticks_actuales--; else motor_derecho.ticks_actuales++; }
void IRAM_ATTR ISR_MD_B() { if (digitalRead(motor_derecho.pin_sensor_A) == digitalRead(motor_derecho.pin_sensor_B)) motor_derecho.ticks_actuales++; else motor_derecho.ticks_actuales--; }
void IRAM_ATTR ISR_MI_A() { if (digitalRead(motor_izquierdo.pin_sensor_A) == digitalRead(motor_izquierdo.pin_sensor_B)) motor_izquierdo.ticks_actuales++; else motor_izquierdo.ticks_actuales--; }
void IRAM_ATTR ISR_MI_B() { if (digitalRead(motor_izquierdo.pin_sensor_A) == digitalRead(motor_izquierdo.pin_sensor_B)) motor_izquierdo.ticks_actuales--; else motor_izquierdo.ticks_actuales++; }

double normalizarAngulo(double angle) { while (angle > PI) angle -= 2 * PI; while (angle < -PI) angle += 2 * PI; return angle; }

// --- PÁGINA WEB ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=0">
  <title>Control Pingüino</title>
  <style>
    body { font-family: 'Verdana', sans-serif; margin: 0; padding: 0; background: linear-gradient(to right, #E0F7FA, #B3E5FC); color: #01579B; overflow: hidden; height: 100vh; display: flex; flex-direction: column; }
    header { height: 10%; display: flex; align-items: center; justify-content: center; background: rgba(255,255,255,0.6); backdrop-filter: blur(5px); }
    h2 { margin: 0; font-size: 22px; }
    .main-layout { display: flex; flex: 1; height: 90%; }
    .map-panel { width: 60%; display: flex; justify-content: center; align-items: center; position: relative; padding: 10px; }
    .controls-panel { width: 40%; display: flex; flex-direction: column; justify-content: space-evenly; align-items: center; background: rgba(255,255,255,0.3); border-left: 2px solid rgba(255,255,255,0.8); padding: 5px; }
    #map-container { background: white; width: 95%; height: 95%; border: 6px solid #FFF; border-radius: 20px; box-shadow: 0 10px 30px rgba(0,20,60,0.15); }
    canvas { width: 100%; height: 100%; border-radius: 14px; }
    .btn { display: inline-block; width: 90%; margin: 4px; padding: 12px; font-size: 14px; font-weight: bold; color: white; border: none; border-radius: 15px; cursor: pointer; box-shadow: 0 4px #004c8c; transition: transform 0.1s; }
    .btn:active { transform: translateY(3px); box-shadow: 0 1px #004c8c; }
    .t1 { background-color: #29B6F6; } .t2 { background-color: #039BE5; } .t3 { background-color: #0288D1; } .t4 { background-color: #0277BD; } .t5 { background-color: #01579B; }
    .stop { background-color: #D32F2F; margin-top: 10px; border: 2px solid white; }
    .sensor-btn { background-color: #FFB300; color: #333; margin-bottom: 10px; border: 2px solid white; }
    .active { background-color: #7CB342; color: white; } .inactive { background-color: #E53935; color: white; }
    .joystick-wrapper { margin-top: 5px; display: flex; flex-direction: column; align-items: center; }
    #joystick-container { position: relative; width: 140px; height: 140px; background: rgba(255, 255, 255, 0.5); border-radius: 50%; border: 3px solid #FFF; box-shadow: inset 0 0 15px rgba(0,200,255,0.2); touch-action: none; }
    #joystick { position: absolute; width: 50px; height: 50px; background: radial-gradient(circle at 30% 30%, #4FC3F7, #0288D1); border-radius: 50%; top: 45px; left: 45px; box-shadow: 0 4px 10px rgba(0,0,0,0.2); }
  </style>
</head>
<body>
  <header><h2>🐧 Panel de Control</h2></header>
  <div class="main-layout">
    <div class="map-panel"><div id="map-container"><canvas id="robotMap" width="600" height="600"></canvas></div></div>
    <div class="controls-panel">
      <div style="width: 100%; text-align: center;">
        <button id="btnSens" class="btn sensor-btn" onclick="toggleSensor()">👀 Ojos: ...</button>
        <button class="btn stop" onclick="sendCommand(0)">🛑 STOP</button>
        <div style="margin: 10px 0; height: 2px; background: rgba(255,255,255,0.6); width: 80%; margin-left: auto; margin-right: auto;"></div>
        <button class="btn t1" onclick="sendCommand(1)">❄️ 1. Recta 20s</button>
        <button class="btn t2" onclick="sendCommand(2)">🌨️ 2. Distancia 2m</button>
        <button class="btn t3" onclick="sendCommand(3)">🧊 3. Círculo 10s</button>
        <button class="btn t4" onclick="sendCommand(4)">⛸️ 4. Corregir 2m</button>
        <button class="btn t5" onclick="sendCommand(5)">🎢 5. Ruta en U</button>
      </div>
      <div class="joystick-wrapper"><div id="joystick-container"><div id="joystick"></div></div></div>
    </div>
  </div>
<script>
  var MAX_VEL = 600; 
  var canvas = document.getElementById("robotMap");
  var ctx = canvas.getContext("2d");
  var scale = 150; 
  var robotX = 0, robotY = 0, robotTh = 0;
  var drawingTask = 0;
  
  // VARIABLES DE OFFSET PARA QUE NO SALTE AL 0,0
  var wOX = 0, wOY = 0, wOTh = 0; 

  setInterval(fetchTelemetry, 50);

  function fetchTelemetry() {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/telemetry", true);
    xhr.onload = function() {
      if (xhr.status == 200) {
        var data = JSON.parse(xhr.responseText);
        robotX = data.x; robotY = data.y; robotTh = data.th;
        drawMap();
      }
    };
    xhr.send();
  }

  function drawMap() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    var originX = canvas.width / 2; 
    var originY = canvas.height / 2; 

    // SUELO
    ctx.strokeStyle = "rgba(0,0,0,0.05)"; ctx.lineWidth = 1;
    for(var i=0; i<canvas.width; i+=40) { ctx.beginPath(); ctx.moveTo(i,0); ctx.lineTo(i,canvas.height); ctx.stroke(); }
    for(var i=0; i<canvas.height; i+=40) { ctx.beginPath(); ctx.moveTo(0,i); ctx.lineTo(canvas.width,i); ctx.stroke(); }
    ctx.strokeStyle = "#bbb"; ctx.lineWidth = 2; ctx.beginPath();
    ctx.moveTo(0, originY); ctx.lineTo(canvas.width, originY); ctx.moveTo(originX, 0); ctx.lineTo(originX, canvas.height); ctx.stroke();

    // TRAYECTORIA ROJA (FIJA EN EL SUELO USANDO EL OFFSET)
    if (drawingTask > 0) {
        ctx.save();
        // Usamos el offset guardado (wOX, wOY) como punto de inicio de la línea roja
        var startScreenX = originX + (wOX * scale);
        var startScreenY = originY - (wOY * scale);
        
        ctx.translate(startScreenX, startScreenY);
        ctx.rotate(-wOTh); 

        ctx.strokeStyle = "rgba(255, 0, 0, 0.5)"; ctx.lineWidth = 4; ctx.beginPath();
        if(drawingTask == 1 || drawingTask == 2 || drawingTask == 4) { ctx.moveTo(0, 0); ctx.lineTo(2.0*scale, 0); } 
        else if (drawingTask == 3) { var radioPix = 0.4 * scale; ctx.beginPath(); ctx.arc(0, -radioPix, radioPix, 0, 2*Math.PI); } 
        else if (drawingTask == 5) { ctx.moveTo(0, 0); ctx.lineTo(0.5*scale, 0); ctx.arc(0.5*scale, -0.25*scale, 0.25*scale, Math.PI/2, -Math.PI/2, true); ctx.lineTo(0, -0.5*scale); ctx.lineTo(0, 0); }
        ctx.stroke();
        ctx.restore();
    }

    // PINGÜINO (CALCULADO CON OFFSET PARA QUE FLUYA)
    // Posicion Global = Offset + (Posicion Local Rotada por Offset)
    var globalX = wOX + (robotX * Math.cos(wOTh) - robotY * Math.sin(wOTh));
    var globalY = wOY + (robotX * Math.sin(wOTh) + robotY * Math.cos(wOTh));
    var globalTh = wOTh + robotTh;

    var screenX = originX + (globalX * scale);
    var screenY = originY - (globalY * scale);

    ctx.save();
    ctx.translate(screenX, screenY);
    ctx.rotate(-globalTh); 

    // Cuerpo Negro
    ctx.fillStyle = "#111"; ctx.beginPath(); ctx.ellipse(0, 0, 24, 16, 0, 0, 2*Math.PI); ctx.fill();
    
    // Aletas (Más adelante)
    ctx.beginPath(); ctx.ellipse(5, 16, 12, 5, Math.PI/2, 0, 2*Math.PI); ctx.fill();
    ctx.beginPath(); ctx.ellipse(5, -16, 12, 5, Math.PI/2, 0, 2*Math.PI); ctx.fill();
    
    // Pies (Trapecios naranjas atras)
    ctx.fillStyle = "#FFB300"; 
    ctx.beginPath(); ctx.moveTo(-20, -5); ctx.lineTo(-28, -8); ctx.lineTo(-28, 8); ctx.lineTo(-20, 5); ctx.fill();

    // Pico (Largo y sobresaliendo)
    ctx.beginPath(); ctx.moveTo(18, -4); ctx.lineTo(32, 0); ctx.lineTo(18, 4); ctx.fill();
    
    ctx.restore();
  }

  var container = document.getElementById("joystick-container");
  var stick = document.getElementById("joystick");
  var active = false; var lastSent = 0;
  var joyRadius = 70; var stickRadius = 25; var maxDist = 45;

  container.addEventListener('touchstart', startDrag); container.addEventListener('touchmove', drag); container.addEventListener('touchend', endDrag);
  container.addEventListener('mousedown', startDrag); document.addEventListener('mousemove', drag); document.addEventListener('mouseup', endDrag);

  function startDrag(e) { active = true; drag(e); }
  function drag(e) {
    if (!active) return;
    e.preventDefault();
    var clientX = e.touches ? e.touches[0].clientX : e.clientX;
    var clientY = e.touches ? e.touches[0].clientY : e.clientY;
    var rect = container.getBoundingClientRect();
    var x = clientX - rect.left - joyRadius; var y = clientY - rect.top - joyRadius;
    var dist = Math.sqrt(x*x + y*y);
    if (dist > maxDist) { x = (x/dist)*maxDist; y = (y/dist)*maxDist; }
    stick.style.left = (joyRadius - stickRadius + x) + 'px'; stick.style.top = (joyRadius - stickRadius + y) + 'px';
    var valY = -1 * (y / maxDist) * MAX_VEL; var valX = (x / maxDist) * MAX_VEL;
    if (Date.now() - lastSent > 80) { sendJoystick(Math.round(valY + valX), Math.round(valY - valX)); lastSent = Date.now(); }
  }
  function endDrag() { active = false; stick.style.left = (joyRadius - stickRadius) + 'px'; stick.style.top = (joyRadius - stickRadius) + 'px'; sendJoystick(0, 0); }
  function sendJoystick(l, r) { var xhr = new XMLHttpRequest(); xhr.open("GET", "/manual?l=" + l + "&r=" + r, true); xhr.send(); }
  
  function sendCommand(id) { 
    // ACTUALIZAMOS EL OFFSET PARA QUE EL MAPA SEA CONTINUO
    // Calculamos la posición global actual antes de que el robot se resetee a 0
    var currentGlobalX = wOX + (robotX * Math.cos(wOTh) - robotY * Math.sin(wOTh));
    var currentGlobalY = wOY + (robotX * Math.sin(wOTh) + robotY * Math.cos(wOTh));
    var currentGlobalTh = wOTh + robotTh;

    // Guardamos esa posición como el nuevo origen
    wOX = currentGlobalX;
    wOY = currentGlobalY;
    wOTh = currentGlobalTh;

    drawingTask = id; 
    var xhr = new XMLHttpRequest(); xhr.open("GET", "/set_task?id=" + id, true); xhr.send(); 
  }
  
  function toggleSensor() { var xhr = new XMLHttpRequest(); xhr.open("GET", "/toggle_sensor", true); xhr.onload = function() { location.reload(); }; xhr.send(); }
  window.onload = function() { var xhr = new XMLHttpRequest(); xhr.open("GET", "/status", true); xhr.onload = function() { var btn = document.getElementById("btnSens"); if(this.responseText == "1") { btn.className="btn sensor-btn active"; btn.innerHTML="👀 Ojos: ON"; } else { btn.className="btn sensor-btn inactive"; btn.innerHTML="🙈 Ojos: OFF"; } }; xhr.send(); }
</script></body></html>
)rawliteral";

// --- HANDLERS WEB ---
void handleRoot() { server.send(200, "text/html; charset=utf-8", index_html); }
void handleTelemetry() { String json = "{\"x\":" + String(pos_x_robot) + ",\"y\":" + String(pos_y_robot) + ",\"th\":" + String(angulo_theta_robot) + "}"; server.send(200, "application/json", json); }

void handleSetTask() {
  if (server.hasArg("id")) {
    int id = server.arg("id").toInt();
    modo_manual = false; pwm_manual_izq = 0; pwm_manual_der = 0;
    motor_derecho.potencia_pwm = 0; motor_izquierdo.potencia_pwm = 0;
    ledcWrite(motor_derecho.pin_fase_A, 0); ledcWrite(motor_derecho.pin_fase_B, 0); ledcWrite(motor_izquierdo.pin_fase_A, 0); ledcWrite(motor_izquierdo.pin_fase_B, 0);
    motor_derecho.error_acumulado = 0; motor_derecho.error_anterior = 0; motor_derecho.derivada_suavizada = 0; motor_derecho.velocidad_suavizada = 0;
    motor_izquierdo.error_acumulado = 0; motor_izquierdo.error_anterior = 0; motor_izquierdo.derivada_suavizada = 0; motor_izquierdo.velocidad_suavizada = 0;
    objetivo_vel_motor1 = 0; objetivo_vel_motor2 = 0;
    // RESETEAMOS SIEMPRE LA COORDENADA LOCAL, PERO EL JS SE ENCARGA DEL OFFSET VISUAL
    if (id != 0) { noInterrupts(); pos_x_robot = 0; pos_y_robot = 0; angulo_theta_robot = 0; interrupts(); } else { comando_lineal_v = 0; comando_angular_w = 0; }
    paso_secuencia = id; estado_recien_iniciado = true; 
    server.send(200, "text/plain", "OK");
  } else { server.send(400, "text/plain", "Bad Request"); }
}

void handleManual() {
  if (server.hasArg("l") && server.hasArg("r")) {
    modo_manual = true; paso_secuencia = 0; 
    pwm_manual_izq = server.arg("l").toInt(); pwm_manual_der = server.arg("r").toInt();
    server.send(200, "text/plain", "OK");
  } else { server.send(400, "text/plain", "Fail"); }
}

void handleToggleSensor() { usar_sensores = !usar_sensores; server.send(200, "text/plain", usar_sensores ? "ON" : "OFF"); }
void handleStatus() { server.send(200, "text/plain", usar_sensores ? "1" : "0"); }

void calcular_PID(InfoMotor &m, int vel_obj) {
  if(m.esta_bloqueado) return;
  double error = vel_obj - m.velocidad_suavizada;
  double deriv = (error - m.error_anterior) / tiempo_muestreo_seg;
  m.derivada_suavizada = FACTOR_SUAVIZADO_DERIVADA * m.derivada_suavizada + (1.0 - FACTOR_SUAVIZADO_DERIVADA) * deriv;
  m.error_acumulado += error * tiempo_muestreo_seg;
  if (m.error_acumulado > 4000) m.error_acumulado = 4000; else if (m.error_acumulado < -4000) m.error_acumulado = -4000;
  double out = m.ganancia_Kp * error + m.ganancia_Ki * m.error_acumulado + m.ganancia_Kd * m.derivada_suavizada;
  m.error_anterior = error;
  if (out > 1023) out = 1023; else if (out < -1023) out = -1023;
  m.potencia_pwm = out;
}

void mover_motor(InfoMotor &m) {
  if(m.esta_bloqueado) { ledcWrite(m.pin_fase_A, 0); ledcWrite(m.pin_fase_B, 0); return; }
  if (m.potencia_pwm >= 0) { ledcWrite(m.pin_fase_A, m.potencia_pwm); ledcWrite(m.pin_fase_B, 0); } 
  else { ledcWrite(m.pin_fase_B, -1 * m.potencia_pwm); ledcWrite(m.pin_fase_A, 0); }
}

void medir_velocidad(InfoMotor &m) {
  double dif = m.ticks_actuales - m.ticks_anteriores_vel;
  m.velocidad_bruta = dif / tiempo_muestreo_seg;
  m.velocidad_suavizada = factor_suavizado_vel * m.velocidad_suavizada + (1.0 - factor_suavizado_vel) * m.velocidad_bruta;
  m.ticks_anteriores_vel = m.ticks_actuales;
}

void IRAM_ATTR Interrupcion_Temporizador() {
  medir_velocidad(motor_derecho); medir_velocidad(motor_izquierdo); 
  if (!modo_manual) { calcular_PID(motor_derecho, objetivo_vel_motor1); mover_motor(motor_derecho); calcular_PID(motor_izquierdo, objetivo_vel_motor2); mover_motor(motor_izquierdo); }
  hay_datos_nuevos = true; 
}

void setup() {
  Serial.begin(115200);
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS)) Serial.println("Fallo IP");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("❄️"); }
  Serial.println("\nCONECTADO: http://172.20.10.5");

  server.on("/", handleRoot);
  server.on("/set_task", handleSetTask);
  server.on("/manual", handleManual); 
  server.on("/telemetry", handleTelemetry); 
  server.on("/toggle_sensor", handleToggleSensor);
  server.on("/status", handleStatus);
  server.begin();
  
  ledcAttach(motor_derecho.pin_fase_A, frecuencia_pwm, resolucion_pwm); ledcAttach(motor_derecho.pin_fase_B, frecuencia_pwm, resolucion_pwm);
  ledcAttach(motor_izquierdo.pin_fase_A, frecuencia_pwm, resolucion_pwm); ledcAttach(motor_izquierdo.pin_fase_B, frecuencia_pwm, resolucion_pwm);
  
  pinMode(motor_derecho.pin_sensor_A, INPUT_PULLUP); pinMode(motor_derecho.pin_sensor_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_derecho.pin_sensor_A), ISR_MD_A, CHANGE); attachInterrupt(digitalPinToInterrupt(motor_derecho.pin_sensor_B), ISR_MD_B, CHANGE);
  pinMode(motor_izquierdo.pin_sensor_A, INPUT_PULLUP); pinMode(motor_izquierdo.pin_sensor_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_izquierdo.pin_sensor_A), ISR_MI_A, CHANGE); attachInterrupt(digitalPinToInterrupt(motor_izquierdo.pin_sensor_B), ISR_MI_B, CHANGE);
  pinMode(PIN_SENSOR_1, INPUT); pinMode(PIN_SENSOR_2, INPUT); pinMode(PIN_SENSOR_3, INPUT);

  temporizador = timerBegin(1000000);
  timerAttachInterrupt(temporizador, &Interrupcion_Temporizador);
  timerAlarm(temporizador, tiempo_muestreo_seg * 1000000, true, 0);
}

void loop() {
  if (hay_datos_nuevos) {
    noInterrupts();
    long t_der = motor_derecho.ticks_actuales; long t_izq = motor_izquierdo.ticks_actuales;
    hay_datos_nuevos = false;
    interrupts();

    long d_t_der = t_der - motor_derecho.ticks_anteriores_odom;
    long d_t_izq = t_izq - motor_izquierdo.ticks_anteriores_odom;
    motor_derecho.ticks_anteriores_odom = t_der; motor_izquierdo.ticks_anteriores_odom = t_izq;

    double m_tick = (2.0 * PI * RADIO_RUEDA) / TICKS_POR_VUELTA;
    double d_cen = ((d_t_der + d_t_izq) / 2.0) * m_tick;
    double d_theta = ((d_t_der - d_t_izq) * m_tick) / DISTANCIA_ENTRE_RUEDAS;

    pos_x_robot += d_cen * cos(angulo_theta_robot + d_theta / 2.0);
    pos_y_robot += d_cen * sin(angulo_theta_robot + d_theta / 2.0);
    angulo_theta_robot += d_theta;
    if (angulo_theta_robot > PI) angulo_theta_robot -= 2 * PI;
    else if (angulo_theta_robot < -PI) angulo_theta_robot += 2 * PI;
  }

  server.handleClient(); 

  bool obstaculo = false;
  if (usar_sensores) { 
    if (analogRead(PIN_SENSOR_1)<UMBRAL_OBSTACULO || analogRead(PIN_SENSOR_2)<UMBRAL_OBSTACULO || analogRead(PIN_SENSOR_3)<UMBRAL_OBSTACULO) {
       delay(2);
       if (analogRead(PIN_SENSOR_1)<UMBRAL_OBSTACULO || analogRead(PIN_SENSOR_2)<UMBRAL_OBSTACULO || analogRead(PIN_SENSOR_3)<UMBRAL_OBSTACULO) obstaculo = true;
    }
  }

  if (obstaculo) {
     if (!en_pausa_obstaculo) { 
       en_pausa_obstaculo = true; 
       if (paso_secuencia == 1 || paso_secuencia == 3) tiempo_acumulado_tarea += (millis() - tiempo_inicio_estado);
     }
     ledcWrite(PIN_M1_PWM_A, 0); ledcWrite(PIN_M1_PWM_B, 0); ledcWrite(PIN_M2_PWM_A, 0); ledcWrite(PIN_M2_PWM_B, 0);
     return; 
  } else {
    if (en_pausa_obstaculo) {
       en_pausa_obstaculo = false;
       if (paso_secuencia == 1 || paso_secuencia == 3) tiempo_inicio_estado = millis() - tiempo_acumulado_tarea;
    }
  }

  if (modo_manual) {
    if (pwm_manual_der >= 0) { ledcWrite(PIN_M1_PWM_A, pwm_manual_der); ledcWrite(PIN_M1_PWM_B, 0); }
    else { ledcWrite(PIN_M1_PWM_B, -pwm_manual_der); ledcWrite(PIN_M1_PWM_A, 0); }

    if (pwm_manual_izq >= 0) { ledcWrite(PIN_M2_PWM_A, pwm_manual_izq); ledcWrite(PIN_M2_PWM_B, 0); }
    else { ledcWrite(PIN_M2_PWM_B, -pwm_manual_izq); ledcWrite(PIN_M2_PWM_A, 0); }
    return; 
  }

  float Kp_orient = 2.0;

  switch(paso_secuencia) {
    case 0: comando_lineal_v = 0; comando_angular_w = 0; objetivo_vel_motor1 = 0; objetivo_vel_motor2 = 0; break;

    case 1: 
      if (estado_recien_iniciado) { tiempo_inicio_estado = millis(); comando_lineal_v = 0.1; comando_angular_w = 0.0; estado_recien_iniciado = false; }
      if (millis() - tiempo_inicio_estado > 20000) { paso_secuencia = 0; }
      break;

    case 2:
      if (estado_recien_iniciado) { start_x = pos_x_robot; start_y = pos_y_robot; comando_lineal_v = 0.2; comando_angular_w = 0.0; estado_recien_iniciado = false; }
      if (sqrt(pow(pos_x_robot - start_x, 2) + pow(pos_y_robot - start_y, 2)) >= 2.0) { paso_secuencia = 0; }
      break;

    case 3:
      if (estado_recien_iniciado) { tiempo_inicio_estado = millis(); comando_angular_w = (2.0 * PI) / 10.0; comando_lineal_v = comando_angular_w * 0.4; estado_recien_iniciado = false; }
      if (millis() - tiempo_inicio_estado > 10000) { paso_secuencia = 0; }
      break;

    case 4:
      if (estado_recien_iniciado) { start_x = pos_x_robot; start_y = pos_y_robot; theta_referencia = angulo_theta_robot; comando_lineal_v = 0.2; estado_recien_iniciado = false; }
      { double err = normalizarAngulo(theta_referencia - angulo_theta_robot); comando_angular_w = err * 2.0; }
      if (sqrt(pow(pos_x_robot - start_x, 2) + pow(pos_y_robot - start_y, 2)) >= 2.0) { paso_secuencia = 0; }
      break;

    case 5:
      if (estado_recien_iniciado) { noInterrupts(); pos_x_robot=0; pos_y_robot=0; angulo_theta_robot=0; interrupts(); fase_tarea5 = 0; estado_recien_iniciado = false; }
      switch(fase_tarea5) {
        case 0: comando_lineal_v = 0.2; comando_angular_w = -Kp_orient * angulo_theta_robot; if (pos_x_robot >= LONGITUD_RECTA_T5) fase_tarea5 = 1; break;
        case 1: comando_lineal_v = 0.2; comando_angular_w = 0.2 / RADIO_CURVA_T5; if (pos_y_robot >= (RADIO_CURVA_T5 * 2.0 - 0.01) && cos(angulo_theta_robot) < -0.9) fase_tarea5 = 2; break;
        case 2: comando_lineal_v = 0.2; comando_angular_w = Kp_orient * normalizarAngulo(PI - angulo_theta_robot); if (pos_x_robot <= 0.005) { fase_tarea5 = 3; comando_lineal_v = 0; comando_angular_w = 0; } break;
        case 3: comando_lineal_v = 0; { double err = normalizarAngulo(-PI/2 - angulo_theta_robot); comando_angular_w = 1.5 * err; if (abs(err) < 0.02) fase_tarea5 = 4; } break;
        case 4: comando_lineal_v = 0.2; comando_angular_w = Kp_orient * normalizarAngulo(-PI/2 - angulo_theta_robot); if (pos_y_robot <= 0.005) { fase_tarea5 = 5; comando_lineal_v = 0; } break;
        case 5: comando_lineal_v = 0; { double err = normalizarAngulo(0 - angulo_theta_robot); comando_angular_w = 1.5 * err; if (abs(err) < 0.02) paso_secuencia = 0; } break;
      }
      break;
  }

  if (!modo_manual && paso_secuencia != 0) {
      double v_der = comando_lineal_v + (comando_angular_w * DISTANCIA_ENTRE_RUEDAS / 2.0); 
      double v_izq = comando_lineal_v - (comando_angular_w * DISTANCIA_ENTRE_RUEDAS / 2.0); 
      objetivo_vel_motor1 = (v_der / (2.0 * PI * RADIO_RUEDA)) * TICKS_POR_VUELTA;
      objetivo_vel_motor2 = (v_izq / (2.0 * PI * RADIO_RUEDA)) * TICKS_POR_VUELTA;
  }
}