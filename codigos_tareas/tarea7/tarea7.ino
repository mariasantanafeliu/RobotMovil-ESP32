#include <math.h>
#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// ==========================================
// 1. CONFIGURACIÓN WIFI Y ROS
// ==========================================
char ssid[] = "iPhone de María";
char password[] = "OnX9-P97t-IiTH-Q84f";

// IP del PC (Agente Micro-ROS)
char agent_ip[] = "172.20.10.4"; 
size_t agent_port = 8888;

// ==========================================
// 2. OBJETOS MICRO-ROS
// ==========================================
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_twist;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

// ==========================================
// 3. VARIABLES DE NAVEGACIÓN (TAREA 5)
// ==========================================
// Fase 0: Recta Ida
// Fase 1: Semicírculo
// Fase 2: Recta Vuelta
// Fase 3: Giro 90
// Fase 4: Recta Cierre
// Fase 5: Giro Final
// Fase 6: TERMINADO
int fase_trayectoria = 0; 
const double LONGITUD_RECTA = 0.5; 
const double RADIO_CURVA = 0.25;

bool mision_activa = true; // Empieza automáticamente

// ==========================================
// 4. ESTRUCTURA Y VARIABLES GLOBALES ROBOT
// ==========================================
double pos_x_robot = 0.0;
double pos_y_robot = 0.0;
double angulo_theta_robot = 0.0;

struct InfoMotor {
  int pin_fase_A; 
  int pin_fase_B;      
  int pin_sensor_A; 
  int pin_sensor_B;
  int canal_pwm_A;
  int canal_pwm_B;
      
  volatile long ticks_actuales; 
  long ticks_anteriores_vel; 
  long ticks_anteriores_odom;
  
  double velocidad_bruta; 
  double velocidad_suavizada;  
  
  // PID
  double error_anterior; 
  double error_acumulado; 
  double derivada_filtrada; 
  double potencia_pwm;         
  
  double ganancia_Kp; 
  double ganancia_Ki; 
  double ganancia_Kd;
};

const double RADIO_RUEDA = 0.0335;      
const double DISTANCIA_ENTRE_RUEDAS = 0.2175;   
const double TICKS_POR_VUELTA = 1500.0;
const double TIEMPO_MUESTREO = 0.01; // 10ms

// PID
const double VALOR_KP = 0.17661; 
const double VALOR_KI = 5.0; 
const double VALOR_KD = 0.005;
const float ALPHA = 0.85; 

double factor_suavizado_vel = 0.7; 

// Variables de Control
double input_v = 0.0; 
double input_w = 0.0; 

// ==========================================
// 5. PINES Y SENSORES
// ==========================================
InfoMotor motor_derecho   = {32, 33, 25, 26, 0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, VALOR_KP, VALOR_KI, VALOR_KD};
InfoMotor motor_izquierdo = {19, 18, 27, 14, 2, 3, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, VALOR_KP, VALOR_KI, VALOR_KD};

const int PIN_SENSOR_1 = 34; 
const int PIN_SENSOR_2 = 35; 
const int PIN_SENSOR_3 = 39; 
const int UMBRAL_OBSTACULO = 300; 

const int frecuencia_pwm = 1000; 
const int resolucion_pwm = 10; 

hw_timer_t *timer = NULL;
volatile bool nueva_medida = false;

// ==========================================
// 6. FUNCIONES AUXILIARES
// ==========================================
void IRAM_ATTR ISR_MD_A() { if (digitalRead(motor_derecho.pin_sensor_A) == digitalRead(motor_derecho.pin_sensor_B)) motor_derecho.ticks_actuales--; else motor_derecho.ticks_actuales++; }
void IRAM_ATTR ISR_MD_B() { if (digitalRead(motor_derecho.pin_sensor_A) == digitalRead(motor_derecho.pin_sensor_B)) motor_derecho.ticks_actuales++; else motor_derecho.ticks_actuales--; }
void IRAM_ATTR ISR_MI_A() { if (digitalRead(motor_izquierdo.pin_sensor_A) == digitalRead(motor_izquierdo.pin_sensor_B)) motor_izquierdo.ticks_actuales++; else motor_izquierdo.ticks_actuales--; }
void IRAM_ATTR ISR_MI_B() { if (digitalRead(motor_izquierdo.pin_sensor_A) == digitalRead(motor_izquierdo.pin_sensor_B)) motor_izquierdo.ticks_actuales--; else motor_izquierdo.ticks_actuales++; }

double normalizarAngulo(double angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle < -PI) angle += 2 * PI;
  return angle;
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){ while(1){ delay(100); } }

// Callback: Si mandas algo por teclado, abortamos la misión automática y obedecemos al teclado
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Si recibimos una orden manual distinta de cero, paramos la tarea automática
  if (abs(msg->linear.x) > 0.01 || abs(msg->angular.z) > 0.01) {
    mision_activa = false; // DESACTIVAR AUTOMÁTICO
    input_v = msg->linear.x;
    input_w = msg->angular.z;
  } else {
    // Si mandan 0, paramos motores
    if (!mision_activa) {
       input_v = 0;
       input_w = 0;
    }
  }
}

bool hay_obstaculo() {
  if (analogRead(PIN_SENSOR_1) < UMBRAL_OBSTACULO || 
      analogRead(PIN_SENSOR_2) < UMBRAL_OBSTACULO || 
      analogRead(PIN_SENSOR_3) < UMBRAL_OBSTACULO) return true;
  return false;
}

void calcular_PID(InfoMotor &m, int vel_obj) {
  double error = vel_obj - m.velocidad_suavizada;
  double deriv = (error - m.error_anterior) / TIEMPO_MUESTREO;
  m.derivada_filtrada = ALPHA * m.derivada_filtrada + (1.0 - ALPHA) * deriv;
  m.error_acumulado += error * TIEMPO_MUESTREO;
  
  if (m.error_acumulado > 4000) m.error_acumulado = 4000;
  else if (m.error_acumulado < -4000) m.error_acumulado = -4000;
  
  double out = m.ganancia_Kp * error + m.ganancia_Ki * m.error_acumulado + m.ganancia_Kd * m.derivada_filtrada;
  m.error_anterior = error;
  
  if (out > 1023) out = 1023; else if (out < -1023) out = -1023;
  m.potencia_pwm = out;
}

void mover_motor(InfoMotor &m) {
  if (m.potencia_pwm >= 0) { 
    ledcWrite(m.canal_pwm_A, m.potencia_pwm); 
    ledcWrite(m.canal_pwm_B, 0); 
  } else { 
    ledcWrite(m.canal_pwm_B, -1 * m.potencia_pwm); 
    ledcWrite(m.canal_pwm_A, 0); 
  }
}

void procesar_velocidad(InfoMotor &m) {
  double dif = m.ticks_actuales - m.ticks_anteriores_vel;
  m.velocidad_bruta = dif / TIEMPO_MUESTREO;
  m.velocidad_suavizada = factor_suavizado_vel * m.velocidad_suavizada + (1.0 - factor_suavizado_vel) * m.velocidad_bruta;
  m.ticks_anteriores_vel = m.ticks_actuales;
}

// --- ISR DEL TIMER: AQUÍ ESTÁ EL CEREBRO ---
void IRAM_ATTR ontimerISR() {
  
  // 1. Calcular V y W según: ¿Misión automática o Manual?
  if (mision_activa) {
    // --- LÓGICA DE LA TAREA 5 ---
    float Kp_orient = 2.0;

    switch(fase_trayectoria) {
      case 0: // Recta Ida
        input_v = 0.2; 
        input_w = -Kp_orient * angulo_theta_robot; 
        if (pos_x_robot >= LONGITUD_RECTA) fase_trayectoria = 1;
        break;

      case 1: // Semicírculo
        input_v = 0.2;
        // Compensamos radio para el giro (truco de calibración anterior)
        input_w = 0.2 / (RADIO_CURVA + 0.015); 
        if (pos_y_robot >= (RADIO_CURVA * 2.0 - 0.01) && cos(angulo_theta_robot) < -0.9) fase_trayectoria = 2;
        break;

      case 2: // Recta Vuelta
        input_v = 0.2;
        {
          double err = normalizarAngulo(PI - angulo_theta_robot);
          input_w = Kp_orient * err;
        }
        if (pos_x_robot <= 0.005) { fase_trayectoria = 3; input_v=0; input_w=0;}
        break;

      case 3: // Giro 90
        input_v = 0.0;
        {
           double err = normalizarAngulo(-PI/2 - angulo_theta_robot);
           input_w = 1.5 * err;
           if (abs(err) < 0.02) fase_trayectoria = 4;
        }
        break;

      case 4: // Cierre
         input_v = 0.2;
         {
           double err = normalizarAngulo(-PI/2 - angulo_theta_robot);
           input_w = Kp_orient * err;
         }
         if (pos_y_robot <= 0.005) { fase_trayectoria = 5; input_v=0; input_w=0;}
         break;

      case 5: // Giro Final
         input_v = 0.0;
         {
           double err = normalizarAngulo(0.0 - angulo_theta_robot);
           input_w = 1.5 * err;
           if (abs(err) < 0.02) {
             fase_trayectoria = 6; // FIN
             input_v = 0; input_w = 0;
           }
         }
         break;
         
      case 6: // Terminado
         input_v = 0; input_w = 0;
         break;
    }
  } 
  // Si mision_activa es false, input_v e input_w vienen del teclado (callback)

  // 2. Cinemática Inversa
  double v_der = input_v + (input_w * DISTANCIA_ENTRE_RUEDAS / 2.0); 
  double v_izq = input_v - (input_w * DISTANCIA_ENTRE_RUEDAS / 2.0); 

  double ref_ticks_D = (v_der / (2.0 * PI * RADIO_RUEDA)) * TICKS_POR_VUELTA;
  double ref_ticks_I = (v_izq / (2.0 * PI * RADIO_RUEDA)) * TICKS_POR_VUELTA;

  // 3. PID
  procesar_velocidad(motor_derecho);   calcular_PID(motor_derecho, ref_ticks_D);   mover_motor(motor_derecho);
  procesar_velocidad(motor_izquierdo); calcular_PID(motor_izquierdo, ref_ticks_I); mover_motor(motor_izquierdo);
  
  nueva_medida = true; // Avisar para publicar odometría
}

// ==========================================
// 7. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // 1. WiFi
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  // 2. Pines Motores
  ledcSetup(motor_derecho.canal_pwm_A, frecuencia_pwm, resolucion_pwm);
  ledcSetup(motor_derecho.canal_pwm_B, frecuencia_pwm, resolucion_pwm);
  ledcSetup(motor_izquierdo.canal_pwm_A, frecuencia_pwm, resolucion_pwm);
  ledcSetup(motor_izquierdo.canal_pwm_B, frecuencia_pwm, resolucion_pwm);

  ledcAttachPin(motor_derecho.pin_fase_A, motor_derecho.canal_pwm_A);
  ledcAttachPin(motor_derecho.pin_fase_B, motor_derecho.canal_pwm_B);
  ledcAttachPin(motor_izquierdo.pin_fase_A, motor_izquierdo.canal_pwm_A);
  ledcAttachPin(motor_izquierdo.pin_fase_B, motor_izquierdo.canal_pwm_B);
  
  pinMode(motor_derecho.pin_sensor_A, INPUT_PULLUP); pinMode(motor_derecho.pin_sensor_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_derecho.pin_sensor_A), ISR_MD_A, CHANGE); attachInterrupt(digitalPinToInterrupt(motor_derecho.pin_sensor_B), ISR_MD_B, CHANGE);
  
  pinMode(motor_izquierdo.pin_sensor_A, INPUT_PULLUP); pinMode(motor_izquierdo.pin_sensor_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motor_izquierdo.pin_sensor_A), ISR_MI_A, CHANGE); attachInterrupt(digitalPinToInterrupt(motor_izquierdo.pin_sensor_B), ISR_MI_B, CHANGE);

  pinMode(PIN_SENSOR_1, INPUT); pinMode(PIN_SENSOR_2, INPUT); pinMode(PIN_SENSOR_3, INPUT);

  delay(2000); 

  // 3. Iniciar Micro-ROS (SI ESTO FALLA, SE QUEDA AQUÍ Y NO ARRANCA MOTORES)
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_robot", "", &support));

  RCCHECK(rclc_publisher_init(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom",
    &rmw_qos_profile_default));

  static char frame_id[] = "odom";
  static char child_frame_id[] = "base_link";
  odom_msg.header.frame_id.data = frame_id;
  odom_msg.header.frame_id.size = strlen(frame_id);
  odom_msg.header.frame_id.capacity = strlen(frame_id) + 1;
  odom_msg.child_frame_id.data = child_frame_id;
  odom_msg.child_frame_id.size = strlen(child_frame_id);
  odom_msg.child_frame_id.capacity = strlen(child_frame_id) + 1;

  RCCHECK(rclc_subscription_init(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel",
    &rmw_qos_profile_default));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_twist, &subscription_callback, ON_NEW_DATA));

  // 4. ENCENDER MOTORES (Solo llegamos aquí si hay conexión)
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &ontimerISR, true);
  timerAlarmWrite(timer, TIEMPO_MUESTREO * 1000000, true); 
  timerAlarmEnable(timer);
}

// ==========================================
// 8. LOOP
// ==========================================
void loop() {
  // 1. Escuchar órdenes (Siempre)
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); 

  // 2. Publicar Odometría (CONTROLADO: Solo cada 100ms)
  static unsigned long ultimo_envio = 0;
  
  // Solo entramos si hay dato nuevo Y han pasado 100ms desde el último envío
  if (nueva_medida && (millis() - ultimo_envio > 100)) { 
    ultimo_envio = millis(); // Reseteamos el reloj
    
    noInterrupts();
    long t_der = motor_derecho.ticks_actuales; 
    long t_izq = motor_izquierdo.ticks_actuales;
    // Cálculos de velocidad real
    double vel_lin = (motor_derecho.velocidad_suavizada + motor_izquierdo.velocidad_suavizada) / 2.0 * ((2.0 * PI * RADIO_RUEDA) / TICKS_POR_VUELTA);
    double vel_ang = (motor_derecho.velocidad_suavizada - motor_izquierdo.velocidad_suavizada) * ((2.0 * PI * RADIO_RUEDA) / TICKS_POR_VUELTA) / DISTANCIA_ENTRE_RUEDAS;
    nueva_medida = false;
    interrupts();

    // -- Cálculo de Posición (Odometría) --
    long d_t_der = t_der - motor_derecho.ticks_anteriores_odom;
    long d_t_izq = t_izq - motor_izquierdo.ticks_anteriores_odom;
    motor_derecho.ticks_anteriores_odom = t_der; 
    motor_izquierdo.ticks_anteriores_odom = t_izq;

    double m_tick = (2.0 * PI * RADIO_RUEDA) / TICKS_POR_VUELTA;
    double d_cen = ((d_t_der + d_t_izq) / 2.0) * m_tick;
    double d_theta = ((d_t_der - d_t_izq) * m_tick) / DISTANCIA_ENTRE_RUEDAS;

    pos_x_robot += d_cen * cos(angulo_theta_robot + d_theta / 2.0);
    pos_y_robot += d_cen * sin(angulo_theta_robot + d_theta / 2.0);
    angulo_theta_robot += d_theta;

    while (angulo_theta_robot > PI) angulo_theta_robot -= 2 * PI;
    while (angulo_theta_robot < -PI) angulo_theta_robot += 2 * PI;

    // -- Rellenar mensaje ROS --
    odom_msg.pose.pose.position.x = pos_x_robot;
    odom_msg.pose.pose.position.y = pos_y_robot;
    odom_msg.pose.pose.orientation.z = sin(angulo_theta_robot / 2.0);
    odom_msg.pose.pose.orientation.w = cos(angulo_theta_robot / 2.0);
    odom_msg.twist.twist.linear.x = vel_lin;
    odom_msg.twist.twist.angular.z = vel_ang;

    // Publicar
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
  }
}
