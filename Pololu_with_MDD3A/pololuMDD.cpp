#include "pololuMDD.h"

void PololuMDD::setup() {
    pinMode(pwm1,OUTPUT);
    pinMode(pwm2,OUTPUT);
    // pinMode(enc1,INPUT);
    // pinMode(enc2,INPUT);
}

void PololuMDD::updatePosition() {
  if (!mIsReversed) {
    counter = Enc->read();
  } else {
    counter = -Enc->read();
  }
  return;
}

void PololuMDD::proceedPID() {
  if(is_param_identify) return;
  
  if (isTorqueMode) {
    calcuratePWMTorque();
  } else {
    if (isAimingAngle) {
      calcuratePWMAngle(counter);
    } else {
      // Serial.println("calc speed");
      // calcuratePWMSpeed(currSpeed, targetSpeed, counter);
      calcuratePWMSpeedwithFF();
      // calcuratePWMSpeedwithI_PD();
    }
  }
  // change command process depend on the type of motor driver
  if (is3A) {
    analogWrite(pwm1, constrain(pwm1OutVal*255,0,255));
    analogWrite(pwm2, constrain(pwm2OutVal*255,0,255));
  } else {
    // write 10A drive command (dir and pwm)
    // pwm1 == dir pin , pwm2 == pwm pin
    if (pwm1OutVal == 0) {
      analogWrite(pwm2,constrain(pwm2OutVal,MIN_MOTOR_OUT,MAX_MOTOR_OUT));
      digitalWriteFast(pwm1, HIGH);
    }
    if (pwm2OutVal == 0) {
      analogWrite(pwm2,constrain(pwm1OutVal,MIN_MOTOR_OUT,MAX_MOTOR_OUT));
      digitalWriteFast(pwm1, LOW);
    }
  }
}

// need to be threaded
void PololuMDD::updateSpeed() {
    end_time = micros();
    if(end_time<start_time){
      Serial.println("time measurement exception: end_time is larger than start_time");
      while(1) {
        digitalWrite(13,HIGH);
        delay(500);
        digitalWrite(13,LOW);
        delay(500);
      }
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

int PololuMDD::readEncoder() {
  return counter;
}
double PololuMDD::readEncoderVel() {
  return currSpeed_enc;
}


void PololuMDD::setTargetSpeed(float target) {
    targetSpeed = target;
}
void PololuMDD::setTargetSpeedRPS(float targetRPS) {
    setTargetSpeed(targetRPS / PI * 180.0);
}

void PololuMDD::setTargetAngle(float target) {
  if(isAimingAngle) return;
    targetSpeed = 0;
    targetAngle = (getAngle() - fmod(getAngle(),360)) + target;
    isAimingAngle = true;
    isTorqueMode = false;
    digitalWrite(13,HIGH);
}

void PololuMDD::setTargetTorque(float target) {
  targetTorque = constrain(target, -MAX_TORQUE,MAX_TORQUE);
}

float PololuMDD::getAngle() {
    return counter * DEGREE_PER_COUNT;
}

float PololuMDD::getAngleRadian() {     
    return counter * DEGREE_PER_COUNT / 180.0 * PI;
}

float PololuMDD::getSpeed() {
    return currSpeed;
}
float PololuMDD::getSpeedRPS() {
    return currSpeed * PI / 180.0;
}
float PololuMDD::getTargetAngle() {
    return targetAngle;
}

void PololuMDD::calcuratePWMSpeed(double currentSpeed, double targetSpeed, int counter) {
  if (targetSpeed ==0) {
    pwm1OutVal = 0;
    pwm2OutVal = 0;
    return;
  }
  double motorSpeedErrorInCPM = (targetSpeed - currentSpeed);

  motorSpeedIntegral += constrain(motorSpeedErrorInCPM * (double)dt / 1000000.0,-10,10);

  double motorSpeedDiff = (motorSpeedErrorInCPM - preMotorSpeedErrorInCPM) / (double)dt * 1000000.0;
  preMotorSpeedErrorInCPM = motorSpeedErrorInCPM;

  float motorOut = (motorSpeedErrorInCPM * Kp + motorSpeedIntegral * Ki + motorSpeedDiff * Kd);

  if (motorOut > 0) {  // increase motor count
    pwm1OutVal = constrain(abs(motorOut),0,12);
    pwm2OutVal = 0;
  } else if (motorOut < -0){  // decrease motor count
    pwm1OutVal = 0;
    pwm2OutVal = constrain(abs(motorOut),0,12);
  } 
}

void PololuMDD::calcuratePWMSpeedwithFF() {
  // if (targetSpeed ==0) {
  //   pwm1OutVal = 0;
  //   pwm2OutVal = 0;
  //   return;
  // }
  double motorSpeedErrorInCPM = (targetSpeed - currSpeed);

  motorSpeedIntegral += constrain(((motorSpeedErrorInCPM + preMotorSpeedErrorInCPM)/2 * (double)dt / 1000000.0),-10,10);

  double motorSpeedDiff = (motorSpeedErrorInCPM - preMotorSpeedErrorInCPM) / (double)dt * 1000000.0;
  preMotorSpeedErrorInCPM = motorSpeedErrorInCPM;

  int motorOut = (motorSpeedErrorInCPM * Kp + motorSpeedIntegral * Ki + motorSpeedDiff * Kd + Kf * disturbance);

  if (mIsReversed) {
    if (motorOut > 0) {  // increase motor count
      pwm1OutVal = abs(motorOut);
      pwm2OutVal = 0;
    } else if (motorOut < -0){  // decrease motor count
      pwm1OutVal = 0;
      pwm2OutVal = abs(motorOut);
    } 
  } else {
    if (motorOut < -0) {  // increase motor count
      pwm1OutVal = abs(motorOut);
      pwm2OutVal = 0;
    } else if (motorOut > 0){  // decrease motor count
      pwm1OutVal = 0;
      pwm2OutVal = abs(motorOut);
    } 
  }
}

void PololuMDD::calcuratePWMSpeedwithI_PD() {
  // if (targetSpeed ==0) {
  //   pwm1OutVal = 0;
  //   pwm2OutVal = 0;
  //   return;
  // }
  double motorSpeedErrorInCPM = (targetSpeed - currSpeed);

  motorSpeedIntegral += constrain(((motorSpeedErrorInCPM + preMotorSpeedErrorInCPM)/2 * (double)dt / 1000000.0),-10,10);

  double motorSpeedDiff = (currSpeed - preMotorSpeedErrorInCPM) / (double)dt * 1000000.0;
  preMotorSpeedErrorInCPM = currSpeed;

  int motorOut = (-currSpeed * Kp + motorSpeedIntegral * Ki - motorSpeedDiff * Kd + Kf * disturbance);

  if (motorOut > 0) {  // increase motor count
    pwm1OutVal = abs(motorOut);
    pwm2OutVal = 0;
  } else if (motorOut < -0){  // decrease motor count
    pwm1OutVal = 0;
    pwm2OutVal = abs(motorOut);
  }
}

void PololuMDD::calcuratePWMAngle(int counter) {
  int errorInCount = targetAngle / DEGREE_PER_COUNT - counter;
  if (abs(errorInCount) * DEGREE_PER_COUNT < ALLOWABLE_DEGREE) {
    motorAngleStableMillis += (millis() - prevmotorAngleStableMills);
    if(motorAngleStableMillis > 2000) {
      pwm1OutVal = 0;
      pwm2OutVal = 0;
      isAimingAngle = false;
      motorAngleIntegral = 0;
      motorAngleStableMillis = 0;
      digitalWrite(13,LOW);
      // if (resetFlag) {
      //   Enc->write(0);
      //   resetFlag = false;
      //   counter = 0;
      //   prevCounter = 0;
      // }
      return;
    }
  }
  prevmotorAngleStableMills = millis();
  motorAngleIntegral += errorInCount * (double)dt / 1000000.0;

  int motorDir = 0.5 * errorInCount + 2 *  motorAngleIntegral;
  if (mIsReversed) {
    if (motorDir > 0) {
      pwm1OutVal = constrain(abs(motorDir),10,100);
      pwm2OutVal = 0;
    } else if (motorDir < -0) {
      pwm1OutVal = 0;
      pwm2OutVal = constrain(abs(motorDir),10,100);
    }
  } else {
    if (motorDir < -0) {
      pwm1OutVal = constrain(abs(motorDir),10,100);
      pwm2OutVal = 0;
    } else if (motorDir > 0) {
      pwm1OutVal = 0;
      pwm2OutVal = constrain(abs(motorDir),10,100);
    }
  }

}

void PololuMDD::calcuratePWMTorque() {
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

void PololuMDD::reset() {
    motorSpeedIntegral = 0;
    preMotorSpeedErrorInCPM = 0;
    resetFlag = true;
    setTargetAngle(getAngle() - fmod(getAngle(),1800));
    targetSpeed = 0;
    currSpeed = 0;
    // digitalWrite(pwm1,LOW);
    // digitalWrite(pwm2,LOW);
}

void PololuMDD::stop() {
    targetSpeed = 0;
    currSpeed = 0;
    digitalWrite(pwm1,LOW);
    digitalWrite(pwm2,LOW);
}

void PololuMDD::newPosInit() {
    Enc->write(0);
    counter = 0;
    prevCounter = 0;
}

void PololuMDD::setIsTorqueMode(bool isTorqueMode_) {
  isTorqueMode = isTorqueMode_;
}