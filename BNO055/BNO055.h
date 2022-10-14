#ifndef BNO055_h
#define BNO055_h
#endif
#ifndef Adafruit_BNO055_h
#define Adafruit_BNO055_h
#endif
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Eigen.h>
#include <Wire.h>
using namespace Eigen;

class BNO055 {
    public:
        int sensor_id;
        Adafruit_BNO055 bno;
        TwoWire *_wire;
        BNO055(int address, TwoWire *theWire) {
            sensor_id = address;
            _wire = theWire;
        };
        void setup();
        void printQuat();
        Vector3f getEulerAngle(); // fuckin stupid
        Vector3f getEulerAngleFromQuat(); // recomended method
        Vector3f getFixedEulerAngleFromQuat(float x_fix,float y_fix,float z_fix);
        Vector3f getGyro();
        Vector3f getAngularAcc();
        Vector3f getAcc();
        void updateAngularAcc(double stabilize_coef);
        void initAnglePosition(Vector3f initAngle);
        void getCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag);
        void displayCalibration();
        void getCalibrationData();
        void QuickCalibration(adafruit_bno055_offsets_t calibData_old);
    private:
        Vector3f currGyro; Vector3f prevGyro;
        Vector3f currAngAcc; Vector3f prevAngAcc;
        int lastUpdatemicros;
        Vector3f INITIAL_ANGLE = Vector3f(0,0,0);
        adafruit_bno055_offsets_t calibData;
};