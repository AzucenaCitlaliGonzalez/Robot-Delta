#include <math.h>   // Funciones matemáticas

// ================== PARÁMETROS DEL ROBOT DELTA (en metros) ==================
const float L1 = 0.17;        // brazo superior
const float L2 = 0.32513;     // brazo inferior
const float ra = 0.116;       // radio base
const float rb = 0.05713;     // radio plataforma
const float r  = ra - rb;     // diferencia de radios
const float theta[3] = {0.0, 2.0 * PI / 3.0, -2.0 * PI / 3.0}; // 0°, 120°, -120°

// ================== PINES (ajústalos si usas otros) ==================
#define PUL_1 18
#define DIR_1 19
#define PUL_2 26
#define DIR_2 27
#define PUL_3 33
#define DIR_3 25

// LEDs (1 = blanca -> VERDE ; 2 = amarilla -> AMARILLO)
#define LED_VERDE    5
#define LED_AMARILLO 4

// ================== CONFIGURACIÓN DE MOTOR ==================
const int steps_per_revolution = 1600; // depende del microstepping
const int pulse_delay = 200;           // us entre pasos (velocidad)

// ================== ESTADO ==================
float current_phi[3] = {0,0,0};  // ángulos actuales (rad)

// ================== HOME “SOFTWARE” ==================
const float HOME_POS[3] = {0.0, 0.0, 0.2309};  // X,Y,Z en metros
float phi_home[3];                              // ángulos que corresponden al HOME_POS

long steps_prev_M1 = 0;
long steps_prev_M2 = 0;
long steps_prev_M3 = 0;

void setup() {
  Serial.begin(9600);

  pinMode(PUL_1, OUTPUT); pinMode(DIR_1, OUTPUT);
  pinMode(PUL_2, OUTPUT); pinMode(DIR_2, OUTPUT);
  pinMode(PUL_3, OUTPUT); pinMode(DIR_3, OUTPUT);

  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_AMARILLO, OUTPUT);

  digitalWrite(PUL_1, LOW);
  digitalWrite(PUL_2, LOW);
  digitalWrite(PUL_3, LOW);
  digitalWrite(LED_VERDE, LOW);
  digitalWrite(LED_AMARILLO, LOW);

  // “SOFTWARE HOME”: se calcula el ángulo equivalente a HOME_POS
  CI(HOME_POS, phi_home);
  for (int i = 0; i < 3; i++) current_phi[i] = phi_home[i];
}

void loop() {
  if (!Serial.available()) return;

  String data = Serial.readStringUntil('\n');
  data.trim();
  if (data.length() < 3) return;

  int idx1 = data.indexOf(',');
  int idx2 = data.indexOf(',', idx1 + 1);
  int idx3 = data.indexOf(',', idx2 + 1);
  if (idx1 <= 0 || idx2 <= idx1 || idx3 <= idx2) return;

  float X = data.substring(0, idx1).toFloat();
  float Y = data.substring(idx1 + 1, idx2).toFloat();
  float Z = data.substring(idx2 + 1, idx3).toFloat();
  int clase = data.substring(idx3 + 1).toInt();

  // LEDs según clase
  if (clase == 1) {
    digitalWrite(LED_VERDE, HIGH);
    digitalWrite(LED_AMARILLO, LOW);
  } else if (clase == 2) {
    digitalWrite(LED_VERDE, LOW);
    digitalWrite(LED_AMARILLO, HIGH);
  } else {
    digitalWrite(LED_VERDE, LOW);
    digitalWrite(LED_AMARILLO, LOW);
  }

  float Pd[3] = {X, Y, Z};
  float phi_target[3];
  CI(Pd, phi_target);

  float tiempo_s = millis() / 1000.0;
  Phi_to_STEP_DIR_M1(tiempo_s, phi_target[0], steps_prev_M1);
  Phi_to_STEP_DIR_M2(tiempo_s, phi_target[1], steps_prev_M2);
  Phi_to_STEP_DIR_M3(tiempo_s, phi_target[2], steps_prev_M3);
}

void CI(const float Pd[3], float phi[3]) {
  const float Xp = Pd[0];
  const float Yp = Pd[1];
  const float Zp = Pd[2];

  for (int i = 0; i < 3; i++) {
    float E = 2 * L1 * (r - Xp*cos(theta[i]) - Yp*sin(theta[i]));
    float F = -2 * L1 * Zp;
    float G = Xp*Xp + Yp*Yp + Zp*Zp + r*r + L1*L1 - L2*L2 - 2*r*(Xp*cos(theta[i]) + Yp*sin(theta[i]));
    float D = sqrt(E*E + F*F - G*G);
    phi[i] = 2 * atan((-F - D) / (G - E));
  }
}

void Phi_to_STEP_DIR_M1(float tiempo, float phi, long &steps_prev_M1) {
  const float max_steps_per_second = 1000.0;
  const float Ts = 1e-3;
  int iterations_per_step = round((1.0 / Ts) / max_steps_per_second);
  long t1 = round(tiempo / Ts);
  if (iterations_per_step < 1) iterations_per_step = 1;

  const float deg_per_step = 360.0 / 1600.0;
  long steps_target = floor((180*phi/PI) / deg_per_step);

  int DIR = (steps_target > steps_prev_M1) ? LOW : HIGH;
  digitalWrite(DIR_1, DIR);

  // Pulso STEP
  digitalWrite(PUL_1, LOW);
  if ((steps_target != steps_prev_M1) && (t1 % iterations_per_step == 0)) {
    digitalWrite(PUL_1, HIGH);
    delayMicroseconds(5); // único delay por pulso
    if (DIR == LOW) steps_prev_M1++; else steps_prev_M1--;
  }
}

void Phi_to_STEP_DIR_M2(float tiempo, float phi, long &steps_prev_M2) {
  const float max_steps_per_second = 1000.0;
  const float Ts = 1e-3;
  int iterations_per_step = round((1.0 / Ts) / max_steps_per_second);
  long t2 = round(tiempo / Ts);
  if (iterations_per_step < 1) iterations_per_step = 1;

  const float deg_per_step = 360.0 / 1600.0;
  long steps_target = floor((180*phi/PI) / deg_per_step);

  int DIR = (steps_target > steps_prev_M2) ? LOW : HIGH;
  digitalWrite(DIR_2, DIR);

  digitalWrite(PUL_2, LOW);
  if ((steps_target != steps_prev_M2) && (t2 % iterations_per_step == 0)) {
    digitalWrite(PUL_2, HIGH);
    delayMicroseconds(5);
    if (DIR == LOW) steps_prev_M2++; else steps_prev_M2--;
  }
}

void Phi_to_STEP_DIR_M3(float tiempo, float phi, long &steps_prev_M3) {
  const float max_steps_per_second = 1000.0;
  const float Ts = 1e-3;
  int iterations_per_step = round((1.0 / Ts) / max_steps_per_second);
  long t3 = round(tiempo / Ts);
  if (iterations_per_step < 1) iterations_per_step = 1;

  const float deg_per_step = 360.0 / 1600.0;
  long steps_target = floor((180*phi/PI) / deg_per_step);

  int DIR = (steps_target > steps_prev_M3) ? LOW : HIGH;
  digitalWrite(DIR_3, DIR);

  digitalWrite(PUL_3, LOW);
  if ((steps_target != steps_prev_M3) && (t3 % iterations_per_step == 0)) {
    digitalWrite(PUL_3, HIGH);
    delayMicroseconds(5);
    if (DIR == LOW) steps_prev_M3++; else steps_prev_M3--;
  }
}
