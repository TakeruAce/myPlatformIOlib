#include "pololu.h"

void Pololu::setup() {
    pinMode(pwm,OUTPUT);
    pinMode(out1,OUTPUT);
    pinMode(out2,OUTPUT);
}

void Pololu::update() {
    counter = Enc->read();
    out = calcuratePWMSpeed(currSpeed, targetSpeed, counter ,out1, out2);
    analogWrite(pwm,out);
    return;
}

// need to be threaded
void Pololu::updateSpeed() {
    end_time = micros();
    if(end_time<start_time){
    Serial.println("time measurement exception: end_time is larger than start_time");
    return;
    }
    
    dt = end_time - start_time;
    currSpeed = (counter - prevCounter) / (double)dt * 1000000.0 * DEGREE_PER_COUNT;
    // Serial.println(currSpeed);
    // currSpeed = currSpeed * 0.5 + prevSpeed * 0.5;
    prevSpeed = currSpeed;

    prevCounter = counter;
    start_time = micros();
    return;
}

void Pololu::setTargetSpeed(float target) {
    targetSpeed = target;
}

float Pololu::getAngle() {
    return counter * DEGREE_PER_COUNT;
}

float Pololu::getSpeed() {
    return currSpeed;
}

int Pololu::calcuratePWMSpeed(double currentSpeed, double targetSpeed, int counter, int motorPin1, int motorPin2) {
  if (targetSpeed ==0) {
    return 0;
  }
  double motorSpeedErrorInCPM = targetSpeed - currentSpeed;

  motorSpeedIntegral += motorSpeedErrorInCPM * (double)dt / 1000000.0;
  double motorSpeedDiff = (motorSpeedErrorInCPM - preMotorSpeedErrorInCPM) / (double)dt * 1000000.0;
  preMotorSpeedErrorInCPM = motorSpeedErrorInCPM;

  int motorOut = (int)(motorSpeedErrorInCPM * Kp + motorSpeedIntegral * Ki + motorSpeedDiff * Kd);

  if (motorOut > 0) {  // increase motor count
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else if (motorOut < -0){  // decrease motor count
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }

  return constrain(abs(motorOut), MIN_MOTOR_OUT, MAX_MOTOR_OUT);
}

void Pololu::reset() {
    motorSpeedIntegral = 0;
    preMotorSpeedErrorInCPM = 0;
    Enc->write(0);
    counter = 0;
    prevCounter = -999;
    targetSpeed = 0;
    currSpeed = 0;
    digitalWrite(out1,LOW);
    digitalWrite(out2,LOW);
    digitalWrite(pwm,LOW);
}
