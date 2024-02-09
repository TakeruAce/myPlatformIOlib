#ifndef BLDC_ESC_hpp
#define BLDC_ESC_hpp

#ifndef Servo_h
#include<Servo.h>
#endif

#ifndef Arduino_h
#include<Arduino.h>
#endif


class BLDC_ESC {
    public:
        BLDC_ESC(int pwm_pin, int pulse_width_for_max_speed, int pulse_width_for_min_speed) {
            mPwmpin = pwm_pin;
            mPulseWidthForMaxSpeed = pulse_width_for_max_speed;
            mPulseWidthForMinSpeed = pulse_width_for_min_speed;
        }
        void setup() {
            setThrottlePin();
            mThrottle.writeMicroseconds(mPulseWidthForMaxSpeed);
            int mil = millis();
            while(millis() - mil < 2000);
            mThrottle.writeMicroseconds(mPulseWidthForMinSpeed);
            mil = millis();
            while(millis() - mil < 2000);
            mIsSetup = true;
        }

        void quickSetup() {
            setThrottlePin();
            mThrottle.writeMicroseconds(mPulseWidthForMinSpeed);
            int mil = millis();
            while(millis() - mil < 2000);
            mIsSetup = true;
        }
        // float 0 - 1
        void setSpeed(float level) {
            if (!mIsSetup) return;
            mThrottle.writeMicroseconds((constrain(level,0.0,1.0) * (mPulseWidthForMaxSpeed - mPulseWidthForMinSpeed) + mPulseWidthForMinSpeed));
        }

        void setThrottleWidth(int width) {
            mThrottle.writeMicroseconds(constrain(width,mPulseWidthForMinSpeed, mPulseWidthForMaxSpeed));
        }

        void setThrottlePin() {
            pinMode(mPwmpin,OUTPUT);
            mThrottle.attach(mPwmpin);
        }

        void setIsSetup(bool isSetUp) {
            mIsSetup = isSetUp;
        }
    private:
        Servo mThrottle;
        int mPwmpin;
        int mPulseWidthForMaxSpeed = 0;
        int mPulseWidthForMinSpeed = 0;
        bool mIsSetup = false;
};

#endif
