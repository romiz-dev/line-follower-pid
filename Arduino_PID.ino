#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// pin layar
#define lebarlayar 64
#define panjanglayar 128
Adafruit_SSD1306 layar(panjanglayar, lebarlayar, &Wire, -1);


const int sensorPins[8] = {
  13, 14, 27, 26, 25, 33, 32, 35
};

//--------Pin definitions for the TB6612FNG Motor Driver----
#define AIN1 18
#define BIN1 17
#define AIN2 19
#define BIN2 16
#define PWMA 23
#define PWMB 4
//------------------------------------------------------------

//--------Enter Line Details here---------
bool isBlackLine = 1;             //keep 1 in case of black line. In case of white line change this to 0
unsigned int numSensors = 8;
//-----------------------------------------

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 100;
int currentSpeed = 30;
int sensorWeight[8] = { 8, 4, 2, 1, -1, -2, -4, -8 };
int activeSensors;
float Kp = 0.03;
float Kd = 0.2;
float Ki = 0;

int onLine = 1;
int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];

void setup() {
  layar.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  layar.clearDisplay();
  layar.setTextColor(SSD1306_WHITE);
  layar.setCursor(26, 20);
  layar.setTextSize(2);
  for (int i = 0; i < 8; i++){
    layar.print(analogRead(sensorPins[i]));
  }
  
  layar.display();

  // sbi(ADCSRA, ADPS2);
  // cbi(ADCSRA, ADPS1);
  // cbi(ADCSRA, ADPS0);

  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(34, INPUT_PULLUP);  //Pushbutton
  pinMode(13, OUTPUT);        //LED

  pinMode(5, OUTPUT);     //standby for older carrier boards
  digitalWrite(5, HIGH);  //enables the motor driver
  //in case of a newer board with motor enable jumper, you can delete these two lines which frees up digital pin 5

}

void loop() {
  // put your main code here, to run repeatedly:
  // while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  while (digitalRead(34)) {}
  delay(1000);

  while (1) {
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {  //PID LINE FOLLOW
      linefollow();
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
      if (error < -1000) {
        motor1run(0);
        motor2run(200);
      } else if (error > 1000) {
        motor1run(200);
        motor2run(0);
      }
    }
  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  for (int i = 0; i < 8; i++) {
    error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    activeSensors += sensorArray[i];
  }
  error = error / activeSensors;


  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < -70) {
    rsp = -70;
  }
  //Serial.print(rsp);
  //Serial.print("   ");
  //Serial.println(lsp);
  motor1run(rsp);
  motor2run(lsp);
}

void calibrate() {
  for (int i = 0; i < 8; i++) {
    int reading = analogRead(sensorPins[i]);

    minValues[i] = reading;
    maxValues[i] = reading;
  }

  for (int i = 0; i < 10000; i++) {
    motor1run(70);
    motor2run(-70);

    for (int i = 0; i < 8; i++) {
      int reading = analogRead(sensorPins[i]);

      if (reading < minValues[i]) {
        minValues[i] = reading;
      }
      if (reading > maxValues[i]) {
        maxValues[i] = reading;
      }
    }
  }

  for (int i = 0; i < 8; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;

  for (int i = 0; i < 8; i++) {
    int reading = analogRead(sensorPins[i]);

    if (isBlackLine) {
      sensorValue[i] = map(
        reading, minValues[i], maxValues[i], 0, 1000
      );
    } else {
      sensorValue[i] = map(
        reading, minValues[i], maxValues[i], 1000, 0
      );
    }

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;

    if (sensorArray[i]) {
      onLine = 1;
    }
  }
}
//--------Function to run Motor 1-----------------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

//--------Function to run Motor 2-----------------
void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}
