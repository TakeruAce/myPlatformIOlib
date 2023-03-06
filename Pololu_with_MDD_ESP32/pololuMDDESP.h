#ifndef pololuMDD_h
#define pololuMDD_h
#endif
#ifndef ESP32Encoder_h
#include <ESP32Encoder.h>
#define ESP32Encoder_h
#endif

#include <Arduino.h>


class PololuMDD {
    public:
        PololuMDD(int PWM1, int PWM2, uint8_t ENC1, uint8_t ENC2, float gear_ratio, float KP, float KI, float KD, float KF, bool reversed, bool isTorqueMode_, uint8_t enc_id, bool is3A_,bool is_param_identify_, float count_per_rev) {
            pwm1 = PWM1;//dir if 10A
            pwm2 = PWM2;//pwm if 10A
            enc1 = ENC1;
            enc2 = ENC2;
            AM_GEAR_RATIO = gear_ratio;
            COUNT_PER_REV = count_per_rev;
            DEGREE_PER_COUNT = 360.0 / COUNT_PER_REV / AM_GEAR_RATIO;
            Kp = KP;
            Ki = KI;
            Kd = KD;
            Kf = KF;
            mIsReversed = reversed;
            isTorqueMode = isTorqueMode_;
            is3A = is3A_;
            is_param_identify = is_param_identify_;
        }
        void setup();
        void updatePosition();
        void updateSpeed();
        void proceedPID();
        void setTargetSpeed(float targetSpeed);// [degree/s]        
        void setTargetSpeedRPS(float targetSpeedRPS);// [rad/s]
        float getTargetSpeed() {return targetSpeed;}
        float getTargetSpeedRPS() {return targetSpeed * PI / 180.0;}
        void setTargetAngle(float targetAngle);
        void setTargetTorque(float targetTorque);
        int readEncoder();
        double readEncoderVel();
        float getAngle();
        float getTargetAngle();
        float getAngleRadian();
        float getSpeed();//[deg/s]
        float getSpeedRPS();//[rad/s]
        void reset();
        void stop();
        void setIsTorqueMode(bool isTorqueMode);
        void setPIDGain(float Kp_new, float Ki_new, float Kd_new, float Kf_new) {
            Kp = Kp_new;
            Ki = Ki_new;
            Kd = Kd_new;
            Kf = Kf_new;
        }
        void setKp(double Kp_new) {Kp = Kp_new;}
        void setKi(double Ki_new) {Ki = Ki_new;}
        void setKd(double Kd_new) {Kd = Kd_new;}
        void setPID(double *pid) {
            Kp = pid[0];
            Ki = pid[1];
            Kd = pid[2];
            Kf = pid[3];
        }
        double* getPID() {
            pid[0] = Kp;
            pid[1] = Ki;
            pid[2] = Kd;
            pid[3] = Kf;
            return pid;
        }
        void setDisturbance(double dist) {disturbance = dist;}
        float Kp = 0.3;
        float Ki = 0.01;
        float Kd = 0.0;
        float Kf = 0.0;
        int out;
        float pwm1OutVal, pwm2OutVal;
        double getIntegral() {
            return motorSpeedIntegral;
        }
        void newPosInit();

    private:
        bool is3A = false;
        bool is_param_identify = false;
        int pwm1, pwm2 ,enc1,enc2;
        int counter,prevCounter;
        ESP32Encoder Enc;
        bool mIsReversed;
        float targetTorque;
        float currSpeed,prevSpeed,targetSpeed;
        float currSpeed_enc,prevSpeed_enc;
        float currAngle,prevAngle,targetAngle;
        bool isAimingAngle = false;
        bool isTorqueMode = false;
        uint32_t start_time,end_time,dt;
        int lastUpdate_t;
        bool resetFlag = false;
        
        // constrains
        float GEAR_RATIO = 1.0; // output / input
        const int MAX_MOTOR_OUT = 255;
        const int MIN_MOTOR_OUT = 0;
        float COUNT_PER_REV = 20;
        // const float AM_GEAR_RATIO = 150.5827; // pololu #3053
        float AM_GEAR_RATIO = 25; 
        float DEGREE_PER_COUNT = 1;
        const float ALLOWABLE_DEGREE = 3;
        const float MAX_TORQUE = 7.8*9.8/100;//[Nm]
        // for PID 
        double pid[4];

        double motorSpeedIntegral = 0;
        double preMotorSpeedErrorInCPM = 0;
        double motorAngleIntegral = 0;
        double preMotorAngleErrorInCPM = 0;
        int motorAngleStableMillis = 0;
        int prevmotorAngleStableMills = 0;
        double disturbance = 0;
        void calcuratePWMSpeed(double currentSpeed, double targetSpeed, int counter);
        void calcuratePWMSpeedwithFF();
        void calcuratePWMSpeedwithI_PD();
        void calcuratePWMAngle(int counter);
        void calcuratePWMTorque();
};