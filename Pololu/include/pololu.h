#ifndef pololu_h
#define pololu_h
#endif
#ifndef Encoder_h
#define Encoder_h
#endif

#include <Encoder.h>


class Pololu {
    public:
        Pololu(int PWM, int OUT1, int OUT2, int ENC1, int ENC2) {
            pwm = PWM;
            out1 = OUT1;
            out2 = OUT2;
            Enc = new Encoder(ENC1, ENC2);
        }
        void setup();
        void update();
        void updateSpeed();
        void setTargetSpeed(float targetSpeed);
        void setTargetAngle(float targetAngle);
        int readEncoder();
        float getAngle();
        float getSpeed();
        void reset();
        double Kp = 2.0;
        double Ki = 0.003;
        double Kd = 0.000;
        int out;
    
    private:
        int pwm, out1, out2;
        int counter,prevCounter;
        Encoder *Enc = new Encoder(0,0);
        float currSpeed,prevSpeed,targetSpeed;
        float currAngle,prevAngle;
        uint32_t start_time,end_time,dt;
        
        // constrains
        const int MAX_MOTOR_OUT = 255;
        const int MIN_MOTOR_OUT = 0;
        const float COUNT_PER_REV = 12;
        const float AM_GEAR_RATIO = 150; // pololu #3053
        const float DEGREE_PER_COUNT = 360 / COUNT_PER_REV / AM_GEAR_RATIO;

        // for PID 

        double motorSpeedIntegral = 0;
        double preMotorSpeedErrorInCPM = 0;
        int calcuratePWMSpeed(double currentSpeed, double targetSpeed, int counter, int motorPin1, int motorPin2);
};