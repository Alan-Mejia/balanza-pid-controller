#include <Arduino.h>
#include <ESP32Servo.h>


Servo myServo;
int servoPin = 19;
int sensorPin = 35;
float sensorAnalogReading = 0;
float sensorVoltage = 0;
float voltageReference=3.3;
float pinResolution = 4096;
float sum= 0.0;
float distance=0;

//Control Variables
int position = 0;
int setPoint = 11;
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError;
double outPut;

// Ganancias PID;
double Kp = 2, Ki = 0.00055, Kd = 1200;


double PID(float input);

void setup() {
  Serial.begin(921600);
  Serial.println("Holaaa");
  myServo.attach(servoPin);
  pinMode(sensorPin,INPUT);
}

void loop() {
  // Valor minimo 10-80cm 
  // Permitted working range from 8-19
  sum = 0.0;
  
  for (int i=0; i<100; i++){
    sum+=float(analogRead(sensorPin));  
  }

  sensorAnalogReading = sum/100;
  // Convertir el valor analógico a voltaje
  sensorVoltage = (sensorAnalogReading * voltageReference) / pinResolution;

  distance =  (-0.7610 * (sensorVoltage * sensorVoltage * sensorVoltage)) + (6.8341 * (sensorVoltage * sensorVoltage)) - ( 22.7881 * sensorVoltage) + 35.9252; 

  Serial.println(distance);
  position = int(PID(distance) + 25);

  // To keep the servo positionition between 0-50 degrees
  if (position >= 50){
    position = 50;
  }else if (position <= 0){
    position = 0;
  }

  Serial.println(position);
  myServo.write(50 - position);
  delay(100);
}






double PID(float input){ 
  // Se guarda el tempo actual 
  currentTime = millis();
  // Se calcula el tiempo transcurrido
  elapsedTime = currentTime - previousTime;

  // Se obtiene el error de posición
  error = setPoint - input;
  // Se calcula la integral del error
  cumError += error * elapsedTime;
  // Se calcula la derivada del error
  rateError = (error - lastError) / elapsedTime;
  // Se calcula la salida del controlador
  outPut = Kp * error + Ki * cumError + Kd * rateError;

  // El error actual se convierte en el error pasado
  lastError = error;
  // El tiempo actual se convierte en el tiempo pasado
  previousTime = currentTime;

  // Se regresa la salida del controlador
  return outPut;
}
