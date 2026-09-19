#include <math.h>
#include <Arduino.h> 

// --- VARIABLES DE POSICIÓN ---
double pos_x_robot = 0.0;
double pos_y_robot = 0.0;
double angulo_theta_robot = 0.0;

// --- ESTRUCTURA MOTORES ---
struct InfoMotor {
  int pin_fase_A; 
  int pin_fase_B;      
  int pin_sensor_A; 
  int pin_sensor_B;    
  volatile long ticks_actuales; 
  long ticks_anteriores_vel; 
  long ticks_anteriores_odom;
  double velocidad_bruta; 
  double velocidad_suavizada;  
  double error_anterior; 
  double error_acumulado; 
  double derivada_suavizada; 
  double potencia_pwm;         
  double ganancia_Kp; 
  double ganancia_Ki; 
  double ganancia_Kd;
  unsigned long tiempo_inicio_bloqueo; 
  bool esta_bloqueado;
};

// --- CONSTANTES ROBOT ---
const double RADIO_RUEDA = 0.0335;      
const double DISTANCIA_ENTRE_RUEDAS = 0.2175;   
const double TICKS_POR_VUELTA = 1500.0;
const double TIEMPO_MUESTREO = 0.01; // 10ms

// --- CONSTANTES TAREA 5 (Trayectoria en U) ---
const double LONGITUD_RECTA_T5 = 0.5; 
const double RADIO_CURVA_T5 = 0.25; 

// --- VARIABLES DE CONTROL ---
double objetivo_vel_motor1 = 0; 
double objetivo_vel_motor2 = 0; 
double comando_lineal_v = 0.0; 
double comando_angular_w = 0.0; 

// --- GESTIÓN DE SECUENCIA ---
int paso_secuencia = 1; 
// Sub-fase para la Tarea 5 (la máquina de estados interna de la trayectoria)
int fase_tarea5 = 0; 

unsigned long tiempo_inicio_estado = 0;
// Variables para gestionar pausas por obstáculo
unsigned long tiempo_acumulado_tarea = 0; 
unsigned long tiempo_ultimo_obstaculo = 0; 
bool en_pausa_obstaculo = false;
bool esperando_reinicio = false;

double start_x = 0;
double start_y = 0;
double theta_referencia = 0; 
bool estado_recien_iniciado = true; 

// --- PINES MOTORES ---
const int PIN_M1_PWM_A = 32; 
const int PIN_M1_PWM_B = 33;   
const int PIN_M1_SENSOR_A = 25; 
const int PIN_M1_SENSOR_B = 26; 
const int PIN_M2_PWM_A = 19; 
const int PIN_M2_PWM_B = 18;
const int PIN_M2_SENSOR_A = 27; 
const int PIN_M2_SENSOR_B = 14;

// --- PINES SENSORES ---
const int PIN_SENSOR_1 = 34; 
const int PIN_SENSOR_2 = 35; 
const int PIN_SENSOR_3 = 39; 
const int UMBRAL_OBSTACULO = 300; 

// PID
const double VALOR_KP = 0.17661; 
const double VALOR_KI = 5.0; 
const double VALOR_KD = 0.005;
const double FACTOR_SUAVIZADO_DERIVADA = 0.85;

InfoMotor motor_derecho   = {PIN_M1_PWM_A, PIN_M1_PWM_B, PIN_M1_SENSOR_A, PIN_M1_SENSOR_B, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, VALOR_KP, VALOR_KI, VALOR_KD, 0, false};
InfoMotor motor_izquierdo = {PIN_M2_PWM_A, PIN_M2_PWM_B, PIN_M2_SENSOR_A, PIN_M2_SENSOR_B, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, VALOR_KP, VALOR_KI, VALOR_KD, 0, false};

const int frecuencia_pwm = 1000; 
const int resolucion_pwm = 10;
double tiempo_muestreo_seg = 0.01; 
double factor_suavizado_vel = 0.7; 
hw_timer_t *temporizador = NULL;
volatile bool hay_datos_nuevos = false;

// --- INTERRUPCIONES ---
void IRAM_ATTR ISR_MD_A() { if (digitalRead(motor_derecho.pin_sensor_A) == digitalRead(motor_derecho.pin_sensor_B)) motor_derecho.ticks_actuales--; else motor_derecho.ticks_actuales++; }
void IRAM_ATTR ISR_MD_B() { if (digitalRead(motor_derecho.pin_sensor_A) == digitalRead(motor_derecho.pin_sensor_B)) motor_derecho.ticks_actuales++; else motor_derecho.ticks_actuales--; }
void IRAM_ATTR ISR_MI_A() { if (digitalRead(motor_izquierdo.pin_sensor_A) == digitalRead(motor_izquierdo.pin_sensor_B)) motor_izquierdo.ticks_actuales++; else motor_izquierdo.ticks_actuales--; }
void IRAM_ATTR ISR_MI_B() { if (digitalRead(motor_izquierdo.pin_sensor_A) == digitalRead(motor_izquierdo.pin_sensor_B)) motor_izquierdo.ticks_actuales--; else motor_izquierdo.ticks_actuales++; }

// --- FUNCIONES AUXILIARES ---
double normalizarAngulo(double angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

// --- CONTROL PID ---
void calcular_PID(InfoMotor &m, int vel_obj) {
  if(m.esta_bloqueado) return;
  double error = vel_obj - m.velocidad_suavizada;
  double deriv = (error - m.error_anterior) / tiempo_muestreo_seg;
  m.derivada_suavizada = FACTOR_SUAVIZADO_DERIVADA * m.derivada_suavizada + (1.0 - FACTOR_SUAVIZADO_DERIVADA) * deriv;
  m.error_acumulado += error * tiempo_muestreo_seg;
  if (m.error_acumulado > 4000) m.error_acumulado = 4000;
  else if (m.error_acumulado < -4000) m.error_acumulado = -4000;
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
  medir_velocidad(motor_derecho);   calcular_PID(motor_derecho, objetivo_vel_motor1);   mover_motor(motor_derecho);
  medir_velocidad(motor_izquierdo); calcular_PID(motor_izquierdo, objetivo_vel_motor2); mover_motor(motor_izquierdo);
  hay_datos_nuevos = true; 
}

// --- FUNCIÓN DE SEGURIDAD OBSTÁCULOS ---
bool hay_obstaculo() {
  int val1 = analogRead(PIN_SENSOR_1);
  int val2 = analogRead(PIN_SENSOR_2);
  int val3 = analogRead(PIN_SENSOR_3);
  
  if (val1 < UMBRAL_OBSTACULO || val2 < UMBRAL_OBSTACULO || val3 < UMBRAL_OBSTACULO) {
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial.println("--- INICIANDO SISTEMA INTEGRADO (TAREAS 1-5) ---");
  
  ledcAttach(motor_derecho.pin_fase_A, frecuencia_pwm, resolucion_pwm); ledcAttach(motor_derecho.pin_fase_B, frecuencia_pwm, resolucion_pwm);
  ledcAttach(motor_izquierdo.pin_fase_A, frecuencia_pwm, resolucion_pwm); ledcAttach(motor_izquierdo.pin_fase_B, frecuencia_pwm, resolucion_pwm);
  
  pinMode(motor_derecho.pin_sensor_A, INPUT_PULLUP); pinMode(motor_derecho.pin_sensor_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_derecho.pin_sensor_A), ISR_MD_A, CHANGE); attachInterrupt(digitalPinToInterrupt(motor_derecho.pin_sensor_B), ISR_MD_B, CHANGE);
  
  pinMode(motor_izquierdo.pin_sensor_A, INPUT_PULLUP); pinMode(motor_izquierdo.pin_sensor_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_izquierdo.pin_sensor_A), ISR_MI_A, CHANGE); attachInterrupt(digitalPinToInterrupt(motor_izquierdo.pin_sensor_B), ISR_MI_B, CHANGE);

  pinMode(PIN_SENSOR_1, INPUT);
  pinMode(PIN_SENSOR_2, INPUT);
  pinMode(PIN_SENSOR_3, INPUT);

  temporizador = timerBegin(1000000);
  timerAttachInterrupt(temporizador, &Interrupcion_Temporizador);
  timerAlarm(temporizador, tiempo_muestreo_seg * 1000000, true, 0);

  delay(3000); 
}

void loop() {
  double error_ang = 0.0; // Variable auxiliar para cálculos angulares
  float Kp_orient = 2.0;

  // 1. CHEQUEO DE SEGURIDAD CONSTANTE
  bool obstaculo_presente = hay_obstaculo();

  if (obstaculo_presente) {
    if (!en_pausa_obstaculo) {
      Serial.println("!!! OBSTÁCULO DETECTADO - PARANDO !!!");
      en_pausa_obstaculo = true;
      esperando_reinicio = false;
      
      // Guardamos tiempo parcial si la tarea es por tiempo (Tarea 1 y 3)
      if (paso_secuencia == 1 || paso_secuencia == 3) {
        tiempo_acumulado_tarea += (millis() - tiempo_inicio_estado);
      }
    }
    objetivo_vel_motor1 = 0;
    objetivo_vel_motor2 = 0;
    return; 
  } 
  else {
    if (en_pausa_obstaculo && !esperando_reinicio) {
      Serial.println("... Obstáculo libre. Esperando 3s ...");
      tiempo_ultimo_obstaculo = millis();
      esperando_reinicio = true;
    }

    if (esperando_reinicio) {
      objetivo_vel_motor1 = 0;
      objetivo_vel_motor2 = 0;
      if (millis() - tiempo_ultimo_obstaculo > 3000) {
        Serial.println(">>> REANUDANDO MARCHA");
        esperando_reinicio = false;
        en_pausa_obstaculo = false;
        
        // Al reanudar, restauramos el tiempo de inicio
        if (paso_secuencia == 1 || paso_secuencia == 3) {
          tiempo_inicio_estado = millis() - tiempo_acumulado_tarea;
        }
      } else {
        return; 
      }
    }
  }

  // --- MÁQUINA DE ESTADOS PRINCIPAL ---

  switch(paso_secuencia) {
    
    // --- TAREA 1: RECTA 20 SEGUNDOS ---
    case 1: 
      if (estado_recien_iniciado) {
        Serial.println(">>> TAREA 1: Recta 20s");
        tiempo_inicio_estado = millis();
        tiempo_acumulado_tarea = 0; 
        comando_lineal_v = 0.1; comando_angular_w = 0.0;
        estado_recien_iniciado = false;
      }
      if (millis() - tiempo_inicio_estado > 20000) { 
        paso_secuencia = 101; 
        estado_recien_iniciado = true;
      }
      break;

    // --- PAUSA 1 (10 seg) ---
    case 101: 
      if (estado_recien_iniciado) {
        Serial.println(">>> PAUSA (10s)...");
        comando_lineal_v = 0; comando_angular_w = 0; 
        tiempo_inicio_estado = millis();
        estado_recien_iniciado = false;
      }
      if (millis() - tiempo_inicio_estado > 10000) { 
        paso_secuencia = 2; 
        estado_recien_iniciado = true;
      }
      break;

    // --- TAREA 2: RECORRER 2 METROS ---
    case 2:
      if (estado_recien_iniciado) {
        Serial.println(">>> TAREA 2: Distancia 2m");
        start_x = pos_x_robot; start_y = pos_y_robot;
        comando_lineal_v = 0.2; comando_angular_w = 0.0;
        estado_recien_iniciado = false;
      }
      if (sqrt(pow(pos_x_robot - start_x, 2) + pow(pos_y_robot - start_y, 2)) >= 2.0) {
        paso_secuencia = 102; 
        estado_recien_iniciado = true;
      }
      break;

    // --- PAUSA 2 (10 seg) ---
    case 102:
      if (estado_recien_iniciado) {
        Serial.println(">>> PAUSA (10s)...");
        comando_lineal_v = 0; comando_angular_w = 0;
        tiempo_inicio_estado = millis();
        estado_recien_iniciado = false;
      }
      if (millis() - tiempo_inicio_estado > 10000) { 
        paso_secuencia = 3; 
        estado_recien_iniciado = true;
      }
      break;

    // --- TAREA 3: CÍRCULO 10 SEGUNDOS ---
    case 3:
      if (estado_recien_iniciado) {
        Serial.println(">>> TAREA 3: Circulo 10s");
        tiempo_inicio_estado = millis();
        tiempo_acumulado_tarea = 0; 
        comando_angular_w = (2.0 * PI) / 10.0;
        comando_lineal_v = comando_angular_w * 0.4;
        estado_recien_iniciado = false;
      }
      if (millis() - tiempo_inicio_estado > 10000) {
        paso_secuencia = 103; 
        estado_recien_iniciado = true;
      }
      break;

    // --- PAUSA 3 (10 seg) ---
    case 103:
      if (estado_recien_iniciado) {
        Serial.println(">>> PAUSA (10s)...");
        comando_lineal_v = 0; comando_angular_w = 0;
        tiempo_inicio_estado = millis();
        estado_recien_iniciado = false;
      }
      if (millis() - tiempo_inicio_estado > 10000) { 
        paso_secuencia = 4; 
        estado_recien_iniciado = true;
      }
      break;

    // --- TAREA 4: 2 METROS CON CORRECCIÓN ---
    case 4:
      if (estado_recien_iniciado) {
        Serial.println(">>> TAREA 4: 2m Corrigiendo Angulo");
        start_x = pos_x_robot; start_y = pos_y_robot;
        theta_referencia = angulo_theta_robot;
        comando_lineal_v = 0.2;
        estado_recien_iniciado = false;
      }

      error_ang = theta_referencia - angulo_theta_robot;
      while (error_ang > PI) error_ang -= 2*PI;
      while (error_ang < -PI) error_ang += 2*PI;
      comando_angular_w = error_ang * 2.0; 

      if (sqrt(pow(pos_x_robot - start_x, 2) + pow(pos_y_robot - start_y, 2)) >= 2.0) {
        paso_secuencia = 104; 
        estado_recien_iniciado = true;
      }
      break;
    
    // --- PAUSA 4 (10 seg) ---
    case 104:
      if (estado_recien_iniciado) {
        Serial.println(">>> PAUSA (10s)...");
        comando_lineal_v = 0; comando_angular_w = 0;
        tiempo_inicio_estado = millis();
        estado_recien_iniciado = false;
      }
      if (millis() - tiempo_inicio_estado > 10000) { 
        paso_secuencia = 5; // VAMOS A LA TAREA 5
        estado_recien_iniciado = true;
      }
      break;

    // --- TAREA 5: TRAYECTORIA EN U ---
    case 5:
      // INICIALIZACIÓN DE TAREA 5
      if (estado_recien_iniciado) {
        Serial.println(">>> TAREA 5: Trayectoria Compleja (U)");
        // RESET DE COORDENADAS: Vital para que la tarea 5 (que empieza en 0,0) funcione bien
        noInterrupts();
        pos_x_robot = 0.0;
        pos_y_robot = 0.0;
        angulo_theta_robot = 0.0;
        interrupts();
        
        fase_tarea5 = 0; // Empezamos en la sub-fase 0 (Recta Ida)
        estado_recien_iniciado = false;
      }

      // SUB-MÁQUINA DE ESTADOS PARA LA TRAYECTORIA DE LA TAREA 5
      switch(fase_tarea5) {
        // FASE 0: RECTA IDA
        case 0:
          comando_lineal_v = 0.2; 
          comando_angular_w = -Kp_orient * angulo_theta_robot; 
          if (pos_x_robot >= LONGITUD_RECTA_T5) fase_tarea5 = 1;
          break;

        // FASE 1: SEMICÍRCULO
        case 1:
          comando_lineal_v = 0.2;
          comando_angular_w = 0.2 / RADIO_CURVA_T5; 
          // Condición salida: Girado 180º (cos < -0.9) y subido en Y
          if (pos_y_robot >= (RADIO_CURVA_T5 * 2.0 - 0.01) && cos(angulo_theta_robot) < -0.9) {
            fase_tarea5 = 2; 
          }
          break;

        // FASE 2: RECTA VUELTA (Hasta X=0)
        case 2:
          comando_lineal_v = 0.2;
          {
            double err = normalizarAngulo(PI - angulo_theta_robot);
            comando_angular_w = Kp_orient * err;
          }
          if (pos_x_robot <= 0.005) { 
            fase_tarea5 = 3; 
            // Parada breve para girar
            comando_lineal_v = 0;
            comando_angular_w = 0;
          }
          break;

        // FASE 3: GIRO DE 90º
        case 3:
          comando_lineal_v = 0.0;
          {
            double err = normalizarAngulo(-PI/2 - angulo_theta_robot);
            comando_angular_w = 1.5 * err;
            if (abs(err) < 0.02) fase_tarea5 = 4;
          }
          break;

        // FASE 4: RECTA CIERRE (Bajar hasta Y=0)
        case 4:
          comando_lineal_v = 0.2;
          {
            double err = normalizarAngulo(-PI/2 - angulo_theta_robot);
            comando_angular_w = Kp_orient * err;
          }
          if (pos_y_robot <= 0.005) { 
             fase_tarea5 = 5;
             comando_lineal_v = 0;
             comando_angular_w = 0;
          }
          break;

        // FASE 5: GIRO FINAL Y FIN
        case 5:
          comando_lineal_v = 0.0; 
          {
            double err = normalizarAngulo(0.0 - angulo_theta_robot);
            comando_angular_w = 1.5 * err; 
            if (abs(err) < 0.02) {
              paso_secuencia = 0; // FIN DE TODAS LAS TAREAS
              estado_recien_iniciado = true;
            }
          }
          break;
      }
      break;

    // --- FIN DEL PROGRAMA ---
    case 0:
      comando_lineal_v = 0; 
      comando_angular_w = 0;
      // Forzar 0 a los motores para evitar pitidos
      objetivo_vel_motor1 = 0;
      objetivo_vel_motor2 = 0;
      
      if (estado_recien_iniciado) {
        Serial.println("--- MISIÓN COMPLETADA ---");
        estado_recien_iniciado = false;
      }
      break;
  }

  // --- CÁLCULO MOTORES ---
  double v_der = comando_lineal_v + (comando_angular_w * DISTANCIA_ENTRE_RUEDAS / 2.0); 
  double v_izq = comando_lineal_v - (comando_angular_w * DISTANCIA_ENTRE_RUEDAS / 2.0); 
  objetivo_vel_motor1 = (v_der / (2.0 * PI * RADIO_RUEDA)) * TICKS_POR_VUELTA;
  objetivo_vel_motor2 = (v_izq / (2.0 * PI * RADIO_RUEDA)) * TICKS_POR_VUELTA;

  // --- ODOMETRÍA ---
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
}
