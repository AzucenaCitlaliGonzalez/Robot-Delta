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
// Coloca el robot físicamente en esta postura ANTES de encender,
// o ajusta estos valores a tu postura real de referencia.
const float HOME_POS[3] = {0.0, 0.0, 0.2309};  // X,Y,Z en metros (ej. 23.09 cm de altura)
float phi_home[3];                            // ángulos que corresponden al HOME_POS

// ================== FUNCIONES DELTA ==================
float disc2(float Xp, float Yp, float Zp, float th) {
  float term1 = 2 * L1 * (r - Xp*cos(th) - Yp*sin(th));
  float term2 = -2 * L1 * Zp;
  float term3 = Xp*Xp + Yp*Yp + Zp*Zp + r*r + L1*L1 - L2*L2 - 2*r*(Xp*cos(th) + Yp*sin(th));
  return term1*term1 + term2*term2 - term3*term3;
}

// Cinemática inversa: posición Pd -> ángulos phi
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

// Movimiento a ángulos objetivos
void moveToAngles(const float phi_target[3]) {
  int  steps[3];
  bool dir[3];

  for (int i = 0; i < 3; i++) {
    float delta = phi_target[i] - current_phi[i];
    dir[i]   = (delta >= 0) ? LOW : HIGH;  // depende del cableado/driver
    steps[i] = (int) (fabs(delta) * steps_per_revolution / (2.0f * PI));
  }

  int max_steps = max(steps[0], max(steps[1], steps[2]));

  for (int i = 0; i < max_steps; i++) {
    digitalWrite(DIR_1, dir[0]);
    digitalWrite(DIR_2, dir[1]);
    digitalWrite(DIR_3, dir[2]);
    delayMicroseconds(10);

    if (i < steps[0]) digitalWrite(PUL_1, HIGH);
    if (i < steps[1]) digitalWrite(PUL_2, HIGH);
    if (i < steps[2]) digitalWrite(PUL_3, HIGH);

    delayMicroseconds(10);

    digitalWrite(PUL_1, LOW);
    digitalWrite(PUL_2, LOW);
    digitalWrite(PUL_3, LOW);

    delayMicroseconds(pulse_delay);
  }

  for (int i = 0; i < 3; i++) current_phi[i] = phi_target[i];
}

// ================== SETUP ==================
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

  // ====== “SOFTWARE HOME”: se calcula el ángulo equivalente a HOME_POS
  // y se asume que el robot ya está físicamente ahí al arrancar.
  CI(HOME_POS, phi_home);
  for (int i = 0; i < 3; i++) current_phi[i] = phi_home[i];

  // Señal breve de listo
  digitalWrite(LED_VERDE, HIGH);
  delay(150);
  digitalWrite(LED_VERDE, LOW);
}

// ================== LOOP ==================
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

  // Si clase == 9 -> señal de trayectoria (opcional): no cambiar LEDs
  if (clase == 1) {           // Blanca
    digitalWrite(LED_VERDE, HIGH);
    digitalWrite(LED_AMARILLO, LOW);
  } else if (clase == 2) {    // Amarilla
    digitalWrite(LED_VERDE, LOW);
    digitalWrite(LED_AMARILLO, HIGH);
  } else if (clase == 0) {    // Ninguna/otro
    digitalWrite(LED_VERDE, LOW);
    digitalWrite(LED_AMARILLO, LOW);
  } // si clase==9 no tocamos LEDs (permanece como está)

  // Validar razonable de límites (opcional)
  if (isnan(X) || isnan(Y) || isnan(Z)) return;

  float Pd[3] = {X, Y, Z};
  float phi_target[3];
  CI(Pd, phi_target);
  moveToAngles(phi_target);
}
