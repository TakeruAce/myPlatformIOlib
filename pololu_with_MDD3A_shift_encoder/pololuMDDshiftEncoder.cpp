#include "pololuMDDshiftEncoder.h"

void PololuMDDshiftEncoder::setup() {
    pinMode(pwm1,OUTPUT);
    pinMode(pwm2,OUTPUT);
    Enc.setReverse(isReversed_);
    // pinMode(enc1,INPUT);
    // pinMode(enc2,INPUT);
}

void PololuMDDshiftEncoder::update(u_int8_t sig) {
    Enc.update(sig);
    counter = Enc.read();
    if (isTorqueMode) {
      calcuratePWMTorque();
    } else {
      if (isAimingAngle) {
        calcuratePWMAngle(counter);
      } else {
        calcuratePWMSpeed(currSpeed, targetSpeed, counter);
      }
    }
    analogWrite(pwm1, pwm1OutVal);
    analogWrite(pwm2, pwm2OutVal);
    return;
}

// need to be threaded
void PololuMDDshiftEncoder::updateSpeed() {
    end_time = micros();
    if(end_time<start_time){
    Serial.println("time measurement exception: end_time is larger than start_time");
    return;
    }
    
    dt = end_time - start_time;
    currSpeed = (counter - prevCounter) / (double)dt * 1000000.0 * DEGREE_PER_COUNT;
    // Serial.println(currSpeed);
    currSpeed = currSpeed * 0.1 + prevSpeed * 0.9;
    prevSpeed = currSpeed;

    prevCounter = counter;
    start_time = micros();
    return;
}

int PololuMDDshiftEncoder::readEncoder() {
  return counter;
}

void PololuMDDshiftEncoder::setTargetSpeed(float target) {
    targetSpeed = target;
}
void PololuMDDshiftEncoder::setTargetSpeedRPS(float targetRPS) {
    setTargetSpeed(targetRPS / PI * 180.0);
}

void PololuMDDshiftEncoder::setTargetAngle(float target) {
    targetSpeed = 0;
    targetAngle = target;
    isAimingAngle = true;
    isTorqueMode = false;
}

void PololuMDDshiftEncoder::setTargetTorque(float target) {
  targetTorque = constrain(target, -MAX_TORQUE,MAX_TORQUE);
}

float PololuMDDshiftEncoder::getAngle() {
    return counter * DEGREE_PER_COUNT;
}

float PololuMDDshiftEncoder::getAngleRadian() {     
    return counter * DEGREE_PER_COUNT / 180.0 * PI;
}

float PololuMDDshiftEncoder::getSpeed() {
    return currSpeed;
}
float PololuMDDshiftEncoder::getSpeedRPS() {
    return currSpeed * PI / 180.0;
}

void PololuMDDshiftEncoder::calcuratePWMSpeed(double currentSpeed, double targetSpeed, int counter) {
  if (targetSpeed ==0) {
    pwm1OutVal = 0;
    pwm2OutVal = 0;
    return;
  }
  double motorSpeedErrorInCPM = (targetSpeed - currentSpeed);

  motorSpeedIntegral += constrain(motorSpeedErrorInCPM * (double)dt / 1000000.0,-10,10);
  // motorSpeedIntegral += motorSpeedErrorInCPM * (double) dt / 1000000.0;
  // if (motorSpeedErrorInCPM < targetSpeed * 0.1) {
  //   motorSpeedIntegral = 0;
  //   lastUpdate_t = micros();
  // }
  double motorSpeedDiff = (motorSpeedErrorInCPM - preMotorSpeedErrorInCPM) / (double)dt * 1000000.0;
  preMotorSpeedErrorInCPM = motorSpeedErrorInCPM;

  int motorOut = (int)(motorSpeedErrorInCPM * Kp + motorSpeedIntegral * Ki + motorSpeedDiff * Kd);

  if (motorOut > 0) {  // increase motor count
    pwm1OutVal = constrain(abs(motorOut),MIN_MOTOR_OUT,MAX_MOTOR_OUT);
    pwm2OutVal = 0;
  } else if (motorOut < -0){  // decrease motor count
    pwm1OutVal = 0;
    pwm2OutVal = constrain(abs(motorOut),MIN_MOTOR_OUT,MAX_MOTOR_OUT);
  } 
}

void PololuMDDshiftEncoder::calcuratePWMAngle(int counter) {
  float errorInCount = targetAngle / DEGREE_PER_COUNT / GEAR_RATIO - counter;
  if (abs(errorInCount) * DEGREE_PER_COUNT < ALLOWABLE_DEGREE) {
    pwm1OutVal = 0;
    pwm2OutVal = 0;
    isAimingAngle = false;
    isTorqueMode = true;
    // if (resetFlag) {
    //   Enc->write(0);
    //   resetFlag = false;
    //   counter = 0;
    //   prevCounter = 0;
    // }
    return;
  }
  int motorDir = (int) errorInCount;
  if (motorDir > 0) {
    pwm1OutVal = constrain(0.5*abs(motorDir),50,100);
    pwm2OutVal = 0;
  } else if (motorDir < -0) {
    pwm1OutVal = 0;
    pwm2OutVal = constrain(0.5*abs(motorDir),50,100);
  }
}

void PololuMDDshiftEncoder::calcuratePWMTorque() {
  int pwmVal = targetTorque / MAX_TORQUE * MAX_MOTOR_OUT;
  
  if (targetTorque > 0) {
    pwm1OutVal = constrain(abs(pwmVal),MIN_MOTOR_OUT,MAX_MOTOR_OUT);
    pwm2OutVal = 0;
  } else if (targetTorque < -0) {
    pwm1OutVal = 0;
    pwm2OutVal = constrain(abs(pwmVal),MIN_MOTOR_OUT,MAX_MOTOR_OUT);
  } else if (targetTorque == 0) {
    pwm1OutVal = 0;
    pwm2OutVal = 0;
  }
}

void PololuMDDshiftEncoder::reset() {
    motorSpeedIntegral = 0;
    preMotorSpeedErrorInCPM = 0;
    resetFlag = true;
    setTargetAngle(getAngle() - (float)((int)getAngle()*100 % 360) / 100.0);
    targetSpeed = 0;
    currSpeed = 0;
    // digitalWrite(pwm1,LOW);
    // digitalWrite(pwm2,LOW);
}

void PololuMDDshiftEncoder::stop() {
    targetSpeed = 0;
    currSpeed = 0;
    digitalWrite(pwm1,LOW);
    digitalWrite(pwm2,LOW);
}

void PololuMDDshiftEncoder::newPosInit() {
    Enc.write(0);
    counter = 0;
    prevCounter = 0;
}