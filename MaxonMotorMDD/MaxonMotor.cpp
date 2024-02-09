#include "MaxonMotor.h"

void MaxonMotor::setup() {
    pinMode(pwm1,OUTPUT);
    pinMode(pwm2,OUTPUT);
    // Encoder 系のSetup
    pinMode(ss_pin, OUTPUT);
  
    // Raise select pins
    // Communication begins when you drop the individual select signsl
    digitalWrite(ss_pin,HIGH);
    
    SPI.begin();
    // Initialize encoder 1
    //    Clock division factor: 0
    //    Negative index input
    //    free-running count mode
    //    x4 quatrature count mode (four counts per quadrature cycle)
    // NOTE: For more information on commands, see datasheet
    digitalWrite(ss_pin,LOW);        // Begin SPI conversation
    SPI.transfer(0x88);                       // Write to MDR0
    SPI.transfer(0x03);                       // Configure to 4 byte mode
    digitalWrite(ss_pin,HIGH);       // Terminate SPI conversation 
    clearEncoderCount();
}

void MaxonMotor::update() {
  updatePosition();
  updateSpeed();
}

void MaxonMotor::updatePosition() {
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;  

  digitalWrite(ss_pin,LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(ss_pin,HIGH);     // Terminate SPI conversation 

    // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;

  if (!mIsReversed) {
    counter = count_value;
  } else {
    counter = -count_value;
  }
  return;
}

void MaxonMotor::proceedPID() {
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
      digitalWrite(pwm1, LOW);
    }
    if (pwm2OutVal == 0) {
      analogWrite(pwm2,constrain(pwm1OutVal,MIN_MOTOR_OUT,MAX_MOTOR_OUT));
      digitalWrite(pwm1, HIGH);
    }
  }
}

// need to be threaded
void MaxonMotor::updateSpeed() {
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
    currSpeed = currSpeed * speed_filter_gain + prevSpeed * (1-speed_filter_gain);
    prevSpeed = currSpeed;

    prevCounter = counter;
    start_time = micros();
    return;
}

int MaxonMotor::readEncoder() {
  return counter;
}
double MaxonMotor::readEncoderVel() {
  return currSpeed_enc;
}


void MaxonMotor::setTargetSpeed(float target) {
    targetSpeed = target;
}
void MaxonMotor::setTargetSpeedRPS(float targetRPS) {
    setTargetSpeed(targetRPS / PI * 180.0);
}

void MaxonMotor::setTargetAngle(float target) {
  if(isAimingAngle) return;
    targetSpeed = 0;
    targetAngle = (getAngle() - fmod(getAngle(),360)) + target;
    isAimingAngle = true;
    isTorqueMode = false;
    digitalWrite(13,HIGH);
}

void MaxonMotor::setTargetTorque(float target) {
  targetTorque = constrain(target, -MAX_TORQUE,MAX_TORQUE);
}

float MaxonMotor::getAngle() {
    return counter * DEGREE_PER_COUNT;
}

float MaxonMotor::getAngleRadian() {     
    return counter * DEGREE_PER_COUNT / 180.0 * PI;
}

float MaxonMotor::getSpeed() {
    return currSpeed;
}
float MaxonMotor::getSpeedRPS() {
    return currSpeed * PI / 180.0;
}
float MaxonMotor::getTargetAngle() {
    return targetAngle;
}

void MaxonMotor::calcuratePWMSpeed(double currentSpeed, double targetSpeed, int counter) {
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

void MaxonMotor::calcuratePWMSpeedwithFF() {
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

void MaxonMotor::calcuratePWMSpeedwithI_PD() {
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

void MaxonMotor::calcuratePWMAngle(int counter) {
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

  int motorDir = 0.3 * errorInCount + 1 *  motorAngleIntegral;
  if (mIsReversed) {
    if (motorDir > 0) {
      pwm1OutVal = constrain(abs(motorDir),0,100);
      pwm2OutVal = 0;
    } else if (motorDir < -0) {
      pwm1OutVal = 0;
      pwm2OutVal = constrain(abs(motorDir),0,100);
    }
  } else {
    if (motorDir < -0) {
      pwm1OutVal = constrain(abs(motorDir),0,100);
      pwm2OutVal = 0;
    } else if (motorDir > 0) {
      pwm1OutVal = 0;
      pwm2OutVal = constrain(abs(motorDir),0,100);
    }
  }

}

void MaxonMotor::calcuratePWMTorque() {
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

void MaxonMotor::reset() {
    motorSpeedIntegral = 0;
    preMotorSpeedErrorInCPM = 0;
    resetFlag = true;
    setTargetAngle(getAngle() - fmod(getAngle(),1800));
    targetSpeed = 0;
    currSpeed = 0;
    // digitalWrite(pwm1,LOW);
    // digitalWrite(pwm2,LOW);
}

void MaxonMotor::stop() {
    targetSpeed = 0;
    currSpeed = 0;
    digitalWrite(pwm1,LOW);
    digitalWrite(pwm2,LOW);
}

void MaxonMotor::newPosInit() {
    // Enc.clearCount();
    counter = 0;
    prevCounter = 0;
}

void MaxonMotor::setIsTorqueMode(bool isTorqueMode_) {
  isTorqueMode = isTorqueMode_;
}

void  MaxonMotor::clearEncoderCount() {
    
  // Set encoder1's data register to 0
  digitalWrite(ss_pin,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ss_pin,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(ss_pin,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ss_pin,HIGH);     // Terminate SPI conversation   
}