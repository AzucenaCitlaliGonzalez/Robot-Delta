#include <AccelStepper.h>
#include <MultiStepper.h>

#define PUL_1 18
#define DIR_1 19
#define PUL_2 26
#define DIR_2 27
#define PUL_3 33
#define DIR_3 25

// Configuración de movimiento
const int STEPS_PER_REV =3200;  // Exactamente 1600 pasos por revolución
// Creamos objetos para cada motor con sus respectivos pines
AccelStepper stepper1(AccelStepper::DRIVER, PUL_1, DIR_1);
AccelStepper stepper2(AccelStepper::DRIVER, PUL_2, DIR_2);
AccelStepper stepper3(AccelStepper::DRIVER, PUL_3, DIR_3);
MultiStepper steppers;

long movimiento[3];
int values[5];
int velocidad=1000;
float phi_1;
float phi_2;
float phi_3;
float phid[3];
float Pd[3];


void setup() {
  // Configuramos los parámetros de los motores
stepper1.setMaxSpeed(velocidad);
stepper2.setMaxSpeed(velocidad);
stepper3.setMaxSpeed(velocidad);

Serial.begin(9600);
delay(30);
steppers.addStepper(stepper1);
steppers.addStepper(stepper2);
steppers.addStepper(stepper3);

int posicion_init=round(((-90.0) / 360.0) * STEPS_PER_REV);
stepper1.setCurrentPosition(posicion_init);
stepper2.setCurrentPosition(posicion_init);
stepper3.setCurrentPosition(posicion_init);
}

void loop() {
  float t = millis() / 1000.0;
  Plan_tray(t,Pd);
  CI(Pd, phid);
  float phid_1 = -phid[0]*180.0/PI; // ±90° a 0.5 Hz
  float phid_2 = -phid[1]*180.0/PI; // ±90° a 0.5 Hz
  float phid_3 = -phid[2]*180.0/PI;

  Serial.println(phid_1);
  Serial.println("\t");
  //Serial.println(phid_2);
  //Serial.println("\t");
  //Serial.println(phid_3);
  //Serial.println("\t");
  
  // Pasos a mover
  int steps1 = round(((phid_1) / 360.0) * STEPS_PER_REV);
  int steps2 = round(((phid_2) / 360.0) * STEPS_PER_REV);
  int steps3 = round(((phid_3) / 360.0) * STEPS_PER_REV);
  
  movimiento[0] = steps1;
  movimiento[1] = steps2;
  movimiento[2] = steps3;
  steppers.moveTo(movimiento);
  while (stepper1.currentPosition() != movimiento[0] || stepper2.currentPosition() != movimiento[1] || stepper3.currentPosition() != movimiento[2]) {
    steppers.run();
  }
  
}

float posicion(float t){
  float result;
  if (t<=2.0*PI){
    result=-90.0+t*(0+90.0)/(2.0*PI);
    }
   else if (t>2.0*PI){
    result=-abs(25.0 * sin(2.0 * PI * 0.1 * t));
    }
    
   return result;
  }


void CI(float Pd[3], float phid[3]){
  // Posiciones
  float Xp = Pd[0];
  float Yp = Pd[1];
  float Zp = Pd[2];
  // Constantes
  float L1 = 0.17;
  float L2 = 0.315;
  float ra = 0.116;
  float rb = 0.05713;
  float r = ra - rb;
  float theta[3] = {0, 2.0*PI/3.0, 4.0*PI/3.0};
  // Variables
  float E[3];
  float F[3];
  float G[3];
  for (int i = 0; i < 3; i = i + 1) {
      E[i] = 2.0*L1*(r - Xp*cos(theta[i]) - Yp*sin(theta[i]));
      F[i] = -2.0*L1*Zp;
      G[i] = Xp*Xp + Yp*Yp + Zp*Zp + r*r + L1*L1 - L2*L2 - 2*r*(Xp*cos(theta[i]) + Yp*sin(theta[i]));
      phid[i]=2*atan((-F[i]-sqrt(E[i]*E[i]+F[i]*F[i]-G[i]*G[i]))/(G[i]-E[i]));
    }
  }

void Plan_tray(float t,float Pd[3]){
// Descripción
// Parámetros
  float r=0.05; //0.1
// Ecuaciones-
  float xd=r*cos(2.0*PI*0.1*t);
  float yd=r*sin(2.0*PI*0.1*t);
  float zd=0.38;
  Pd[0]=xd;
  Pd[1]=yd;
  Pd[2]=zd;
}
