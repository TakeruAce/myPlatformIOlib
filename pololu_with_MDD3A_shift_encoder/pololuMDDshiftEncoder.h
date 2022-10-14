#ifndef pololuMDDshiftEncoder_h
#define pololuMDDshiftEncoder_h
#endif
#ifndef FourbitEncoder_h
#define FourbitEncoder_h
#include <4bitEncoder.hpp>
#endif


class PololuMDDshiftEncoder {
    public:
        PololuMDDshiftEncoder(int PWM1, int PWM2, int ENC1, int ENC2, float gear_ratio, float KP, float KI, float KD, bool _isTorqueMode, bool _isReversed) {
            pwm1 = PWM1;
            pwm2 = PWM2;
            enc1 = ENC1;
            enc2 = ENC2;
            GEAR_RATIO = gear_ratio;
            Kp = KP;
            Ki = KI;
            Kd = KD;
            isTorqueMode = _isTorqueMode;
            isReversed_ = _isReversed;
        }
        void setup();
        void update(uint8_t sig);
        void updateSpeed();
        void setTargetSpeed(float targetSpeed);// [degree/s]        
        void setTargetSpeedRPS(float targetSpeedRPS);// [rad/s]
        float getTargetSpeed() {return targetSpeed;}
        float getTargetSpeedRPS() {return targetSpeed * PI / 180.0;}
        void setTargetAngle(float targetAngle);
        void setTargetTorque(float targetTorque);
        int readEncoder();
        float getAngle();
        float getAngleRadian();
        float getSpeed();//[deg/s]
        float getSpeedRPS();//[rad/s]
        void reset();
        void stop();
        float Kp = 0.3;
        float Ki = 0.01;
        float Kd = 0.0;
        int out;
        int pwm1OutVal, pwm2OutVal;
        double getIntegral() {
            return motorSpeedIntegral;
        }
        void newPosInit();

    private:
        int pwm1, pwm2 ,enc1,enc2;
        bool isReversed_;
        int counter,prevCounter;
        FourBitEncoder Enc;
        float targetTorque;
        float currSpeed,prevSpeed,targetSpeed;
        float currAngle,prevAngle,targetAngle;
        bool isAimingAngle = false;
        bool isTorqueMode;
        uint32_t start_time,end_time,dt;
        int lastUpdate_t;
        bool resetFlag = false;
        
        // constrains
        float GEAR_RATIO = 1.0; // output / input
        const int MAX_MOTOR_OUT = 255;
        const int MIN_MOTOR_OUT = 10;
        const float COUNT_PER_REV = 20;
        const float AM_GEAR_RATIO = 125; // pololu #3053
        const float DEGREE_PER_COUNT = 360 / COUNT_PER_REV / AM_GEAR_RATIO;
        const float ALLOWABLE_DEGREE = 1;
        const float MAX_TORQUE = 7.8*9.8/100;//[Nm]
        // for PID 

        double motorSpeedIntegral = 0;
        double preMotorSpeedErrorInCPM = 0;
        void calcuratePWMSpeed(double currentSpeed, double targetSpeed, int counter);
        void calcuratePWMAngle(int counter);
        void calcuratePWMTorque();
};