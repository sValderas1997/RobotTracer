// Sebastian Alexis Valderas Neculqueo
// Ingenieria Civil en Electronica
// UTEM
// Modelo Robot = smartBots (BRCito)

//------------------------------------------------------------------------------------------------------------------------------------------
//////////////////////////////////// Librerias /////////////////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------------------------------------

#include "QTRSensors.h"         // biblioteca necesaria para utilizar los sensores de línea QTRSensors en el programa

//------------------------------------------------------------------------------------------------------------------------------------------

QTRSensors qtr;                       // Creacion un objeto llamado qtr
const uint8_t SensorCount = 6;        // Numero de sensores en QTR  
uint16_t sensorValues[SensorCount];   // Array para guardar los valores de cada sensor qtr

int lastError = 0;      // Variable de Error actual en qtr
boolean onoff = 0;      // Variable para indicar el apagado y encendido del robot
int val, cnt = 0, v[3];

const uint16_t threshold = 500; // Este valor representa el umbral utilizado para determinar si un sensor detecta
                                // una línea o no.

const int maxspeeda = 150;      // PWM maximo para Motor A
const int maxspeedb = 150;      // PWM maximo para Motor B
const int basespeeda = 100;     // PWM base para Motor A
const int basespeedb = 100;     // PWM base para Motor B
const int minspeeda = -100;        // PWM minimo para Motor A
const int minspeedb = -100;        // PWM minimo para Motor B

float Kp = 0;             // Constante Kp
float Ki = 0;             // Constante Ki
float Kd = 0;             // Constante Kd               
uint8_t multiP = 1;       // Factor de multiplicacion para ajustar magnitud de Constante Kp
uint8_t multiI = 1;       // Factor de multiplicacion para ajustar magnitud de Constante Ki
uint8_t multiD = 1;       // Factor de multiplicacion para ajustar magnitud de Constante Kd
int P;                    // Valor para la creacion de la Constante Kp
int I;                    // Valor para la creacion de la Constante Ki
int D;                    // Valor para la creacion de la Constante Kd
float Pvalue;             // Ultimo valor para la creacion de la Constante Kp
float Ivalue;             // Ultimo valor para la creacion de la Constante Ki   
float Dvalue;             // Ultimo valor para la creacion de la Constante Kd                


const int pwmA = 6;       // Derecho
const int Ain2 = 10;
const int Ain1 = 13;
const int Bin1 = 11;      // Izquierda  
const int Bin2 = 12;
const int pwmB = 5;
const int sA = A1;
const int sB = A0;
const int button = 7;
const int led = 9;



void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();      // Configura los sensores de línea para que operen en el modo RC "resistencia/capacitancia"
  qtr.setSensorPins((const uint8_t[]){A2, A3, A4, A5, A6, A7}, SensorCount); // Especificar los pines a los que están conectados los sensores de línea
  //qtr.setEmitterPin(3); // Especificar el pin al que está conectado el emisor de luz de los sensores de línea

//---------------------------------- CONFIGURACION DE PINES --------------------------------------------------------------------------------
  // Pin Puente H
  pinMode(Ain1, OUTPUT); pinMode(Ain2, OUTPUT); pinMode(Bin1, OUTPUT); pinMode(Bin2, OUTPUT); 

  // Led
  pinMode(led, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT); // Pin del LED integrado en la placa como salida

  // Button
  pinMode(button, INPUT);
  

//---------------------------------

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT); // Pin del LED integrado en la placa como salida
  calibration();                // Realizar la calibración de los sensores de línea antes de comenzar a utilizarlos
  
  //---------------------------------- MOTOR APAGADO -----------------------------------------------------------------------------------------

  digitalWrite(Ain1, false); digitalWrite(Ain2, false); digitalWrite(Bin1, false); digitalWrite(Bin2, false); 
}

void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);    // Enciende Led
  
  for (uint16_t i = 0; i < 400; i++)  // Bucle de 400 iteraciones para buscar valores mínimos y 
  {                                   // máximos esperados para cada sensor, los cuales se guardan  
    qtr.calibrate();                  // en los arreglos qtr.calibratedMinimumOn[] y qtr.calibratedMaximumOn[]
  }
  
  digitalWrite(LED_BUILTIN, LOW);     // Apagado Led
}

void loop() {
  if (Serial.available()) {           // Verifica si hay datos disponibles en el puerto serial
    while(Serial.available() == 0);   // Se entra en un bucle hasta recibir datos para asegurar la recepcion
    valuesread();                     // Leer y procesar los valores recibidos
    processing();                     // Procesar los valores recibidos y realizar acciones adicionales según el caso.                           
  }
  
  if(onoff == 1) {
    robot_control();                  // Control del robot está activado y se deben ejecutar las acciones correspondientes 
  }                                   // para controlar el robot.
  
  if(onoff == 0) {
    forward_brake(0, 0);              // Robot está desactivado y se detiene el movimiento del robot.
  }
}

//This void delimits each instruction.
//The Arduino knows that for each instruction it will receive 2 bytes.
void valuesread() {
  val = Serial.read();      // Valor leido se guarda en val              
  cnt++;                    // El contador cnt aumenta +1
  v[cnt] = val;             // v guarda el valor val en el arreglo cnt
  if (cnt == 2)             // Si cnt es igual a 2 se resetea cnt
    cnt = 0;
}

/* En v[1] indica en cual variable (Kp, multiP, Ki, multiI, Kd, multiD, se va guardar el valor de v[2] */
void processing() {
  int a = v[1];
  int value = v[2];

  switch (a) {
    case 1:
      Kp = value;
      break;
    case 2:
      multiP = value;
      break;
    case 3:
      Ki = value;
      break;
    case 4:
      multiI = value;
      break;
    case 5:
      Kd = value;
      break;
    case 6:
      multiD = value;
      break;
    case 7:
      onoff = value;
      break;
    default:
      // Acción para el caso por defecto 
      break;
  }
}

void forward_brake(int posb, int posa) {
  digitalWrite(Bin2, LOW);
  digitalWrite(Bin1, HIGH);
  digitalWrite(Ain2, LOW);  
  digitalWrite(Ain1, HIGH); 
  analogWrite(pwmB, posb);
  analogWrite(pwmA, posa);
}
void left_brake(int posb, int posa) {
  digitalWrite(Bin2, HIGH);  
  digitalWrite(Bin1, LOW);  
  digitalWrite(Ain2, LOW);  
  digitalWrite(Ain1, HIGH);       
  analogWrite(pwmB, posb);
  analogWrite(pwmA, posa);
}
void right_brake(int posb, int posa) {
  digitalWrite(Bin2, LOW);  
  digitalWrite(Bin1, HIGH);  
  digitalWrite(Ain2, HIGH);  
  digitalWrite(Ain1, LOW);       
  analogWrite(pwmB, posb);
  analogWrite(pwmA, posa);
}
void robot_control() {
  uint16_t position = qtr.readLineWhite(sensorValues);  /* Utiliza la función readLineBlack() 
                                                           para leer los valores de los sensores de línea
                                                           y determinar la posición relativa de la línea detectada */
  int error = 2500 - position;              // Entre 2000 (sensor 3) y 3000 (sensor4) debe estar el robot

  int cnt = 0;                              // Contador en cero
  float sum = 0;                            // Sumatoria en cero
  
  for (int i = 0; i < SensorCount; i++) {   // Bucle de 6 iteraciones
    if(sensorValues[i] >= threshold) {      // Se pregunta si un sensor detecto una linea
      cnt++;                                // En caso de que si aumenta el contador +1
      sum = sum + i;                        // Suma va sumando segun la posicion del sensor
    }
  }

   
  if (cnt >= 3) {
    int motorspeeda = 0;
    int motorspeedb = 0;
    int val = sum/cnt;          // Promedio de la suma ponderada de los índices de los sensores que superan o igualan el umbral
    if(val < 2.5) {
      //doblar a la derecha  
      right_brake(basespeedb, basespeeda); 

    }
    if(val > 2.5) {
      //doblar a la izquierda
      left_brake(basespeedb, basespeeda);

    }
    if(val == 2.5) {
      cnt = cnt/2;              // Se reduce la cantidad de sensores considerados en la búsqueda del sensor mínimo
      uint16_t mini = 100;      // Valor del sensor cuando esta cerca de una superficie blanca
      uint8_t minpos = 0;       // Resetea el indice de la posicion del sensor
      for (int i = 3 - cnt; i <= 2 + cnt; i++) {
         if (mini < sensorValues[i]) {
            mini = sensorValues[i];
            minpos = i;
         }
      }
      
      if(minpos < 2.5) {
      //doblar a la derecha  
      right_brake(basespeedb, basespeeda);   
      }
      
      if(minpos > 2.5) {
      //doblar a la izquierda
      left_brake(basespeedb, basespeeda);  
      }
    }
  }
  else {
    PID(error);
  }
}

void PID(int error) {
  int P = error;                // Representa la respuesta del controlador proporcional al error presente en ese momento.
  int I = I + error;            // Representa la acumulación de errores a lo largo del tiempo
  int D = error - lastError;    // Representa la tasa de cambio del error a lo largo del tiempo.
  lastError = error;            // Ultimo error se actualiza con el actual

  /* Calculo del componente (Pvalue, Ivalue, Dvalue) multiplicado por la ganancia (Kp, Ki, Kd) escalada por el factor de 
  escala (1 / 10^multiP) y el valor (P, I, D) de escala (1 / 10^multiP) y el valor del error (P) */
  
  Pvalue = (Kp/pow(10,multiP))*P;     
  Ivalue = (Ki/pow(10,multiI))*I;
  Dvalue = (Kd/pow(10,multiD))*D;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  float motorspeed = Pvalue + Ivalue + Dvalue;    // velocidad deseada de los motores
  
  int motorspeeda = basespeeda + motorspeed;      // Ajuste de velocidad del motor A sumando motorspeed a la velocidad base (basespeeda).
  int motorspeedb = basespeedb - motorspeed;      // Ajuste de velocidad del motor B sumando motorspeed a la velocidad base (basespeedb).

  /* Se asegura que no se superen los limites establecidos */
  motorspeeda = (motorspeeda > maxspeeda) ? maxspeeda : motorspeeda;
  motorspeedb = (motorspeedb > maxspeedb) ? maxspeedb : motorspeedb;
  motorspeeda = (motorspeeda < minspeeda) ? minspeeda : motorspeeda;
  motorspeedb = (motorspeedb < minspeedb) ? minspeedb : motorspeedb;
  
  //Serial.print(motorspeeda); Serial.print(" "); Serial.println(motorspeedb);
  speedcontrol(motorspeedb, motorspeeda );
}

void speedcontrol(int motb, int mota) {
  /* Si ambas velocidades son no negativas, significa que ambos motores deben moverse hacia adelante */
  if (motb >= 0 && mota >= 0) {
    forward_brake(motb, mota);
  }

  /* el motor B sigue hacia adelante mientras que motor A debe girar hacia atrás  */
  if (motb >= 0 && mota < 0) {
    // Girar a la derecha
    right_brake(motb, abs(mota) );
     
  }

  /* el motor B debe girar hacia atrás mientras que el motor A sigue hacia adelante */ 
  if (motb < 0 && mota >= 0) {
    // Girar a la izquierda
    left_brake(abs(motb), mota);
  }
}
