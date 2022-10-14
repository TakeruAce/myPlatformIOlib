#include <Arduino.h>

class LPfilter {
    public :
        LPfilter(double a) {
            paramA = a;
        };
        LPfilter(double T_control, double f_cutoff) {
            paramA = 1 / (2 * PI * f_cutoff * T_control + 1);
            T_control_ = T_control;
        }
        double updateVal (double newVal) {
            double outPut = prevVal * paramA + newVal * (1 - paramA);
            prevVal = outPut;
            return outPut;
        }

        void setCutoff(double f) {
            paramA = paramA = 1 / (2 * PI * f * T_control_ + 1);
        }
    private :
        double paramA; // a = 1 / (2pi * fc * dt + 1)
        double prevVal = 0;
        double T_control_;
};