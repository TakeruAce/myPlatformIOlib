#include <Arduino.h>
#include "BNO055.h"
#include <cmath>
#include <Eigen.h>
#include <Wire.h>


using namespace Eigen;

Vector3f ToVector(imu::Vector<3> vec);
float fixAngleValue(float val, float startval);

void BNO055::setup() {
    bool error_blink = false;
    bno = Adafruit_BNO055(-1, sensor_id, _wire);
    if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
    {

        /* There was a problem detecting the BNO055 ... check your connections */
        while (1){
        Serial.print(sensor_id);
        Serial.println(" Ooops, no BNO055  detected... Check your wiring or I2C ADDR!");
        digitalWrite(13,error_blink == false ? 0 : 1);
        error_blink = !error_blink;
        delay(1000);
        };
    }
    delay(500);
    bno.setExtCrystalUse(true);
    Serial.println("bno" + (String)sensor_id + " setup is finished.");
}

void BNO055::printQuat() {
    imu::Quaternion quat = bno.getQuat();
    Serial.print(quat.w(), 4);
    Serial.print(",");
    Serial.print(quat.x(), 4);
    Serial.print(",");
    Serial.print(quat.y(), 4);
    Serial.print(",");
    Serial.print(quat.z(), 4);
    Serial.println("");
}

Vector3f BNO055::getEulerAngle() {
    sensors_event_t event;
    bno.getEvent(&event);
    return Vector3f(event.orientation.x, event.orientation.y, event.orientation.z);
}

Vector3f BNO055::getEulerAngleFromQuat() {
    imu::Quaternion q = bno.getQuat();
    double w = q.w();double x = q.x();double y = q.y();double z = q.z();
    // double w = (double)((short)q.w() / 16384);double x = (double)((short)q.x() / 16384);double y = (double)((short)q.y() / 16384);double z = (double)((short)q.z() / 16384);

    // calc eular angle from quatanion
    double ysqr = y * y;
    // roll 
    double t0 = 2.0 * (w * x + y * z);
    double t1 = 1.0 - 2.0 * (x * x + ysqr);
    double roll = std::atan2(t0,t1);
    // pitch 
    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    double pitch = std::asin(t2);
    // yaw (z-axis rotation)
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (ysqr + z * z);  
    double yaw = std::atan2(t3, t4);
    
    return Vector3f(roll, pitch, yaw) * 180 / PI;
}

Vector3f BNO055::getAngularAcc() {
    Vector3f v = currAngAcc;
    // v[0] = abs(v[0]) > 100.0 ? v[0] : 0;
    // v[1] = abs(v[1]) > 100.0 ? v[1] : 0;
    // v[2] = abs(v[2]) > 100.0 ? v[2] : 0;
    return v;
}
Vector3f BNO055::getGyro() {
    Vector3f v = ToVector(bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE));
    // v[0] = abs(v[0]) > 1.0 ? v[0] : 0;
    // v[1] = abs(v[1]) > 1.0 ? v[1] : 0;
    // v[2] = abs(v[2]) > 1.0 ? v[2] : 0;
    return v;
}

void BNO055::updateAngularAcc(double stabilize_coef) {
    // get values in current frame
    int curmicros = micros();
    if (curmicros - lastUpdatemicros < 5000) return;
    currGyro = ToVector(bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE)); //[dps]
    
    // calculate angular acceration
    Vector3f rawAngAcc = (currGyro - prevGyro) / (curmicros - lastUpdatemicros) * 1000000; // [dpss]
    currAngAcc = rawAngAcc * stabilize_coef + prevAngAcc * (1 - stabilize_coef);

    // update values for next frame
    prevGyro = currGyro;
    prevAngAcc = currAngAcc;
    lastUpdatemicros = micros();
}

Vector3f BNO055::getFixedEulerAngleFromQuat(float x_fix,float y_fix,float z_fix) {
    Vector3f raw = getEulerAngleFromQuat() - INITIAL_ANGLE;
    return Vector3f(fixAngleValue(raw.x(), x_fix), fixAngleValue(raw.y(), y_fix), fixAngleValue(raw.z(), z_fix));
}
void BNO055::initAnglePosition(Vector3f initAngle) {
    INITIAL_ANGLE = -initAngle;
    for (int i=0;i<10;i++) {
        INITIAL_ANGLE += getEulerAngleFromQuat() / 10.0f;
        delay(100);
        Serial.print("o");
    }
    Serial.println("");
    return;
}
Vector3f ToVector(imu::Vector<3> vec) {
  return Vector3f(vec[0],vec[1],vec[2]);
}

float fixAngleValue(float val, float startval)
{
  if (val <= 0 - startval)
  {
    return ((360 - startval) + (val - (0 - startval)));
  }
  if (val >= 360 - startval)
  {
    return ((0 - startval) + (val - (360 - startval)));
  }
  return val;
}

Vector3f BNO055::getAcc() {
    return ToVector(bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL));
}

void BNO055::getCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag)
{
  bno.getCalibration(sys, gyro, accel, mag);
}

void BNO055::displayCalibration() {
    uint8_t sys =0, gyro=0, accel=0, mag=0;
    getCalibration(&sys,&gyro,&accel,&mag);
    Serial.print(sensor_id);
    Serial.print(" system:");
    Serial.print(sys, DEC);
    Serial.print(",gyro:");
    Serial.print(gyro, DEC);
    Serial.print(",accel:");
    Serial.print(accel, DEC);
    Serial.print(",mag:");
    Serial.println(mag, DEC);
    return;
}

void BNO055::getCalibrationData() {
    if(bno.getSensorOffsets(calibData)) {
            Serial.print("Accelerometer: ");
            Serial.print(calibData.accel_offset_x); Serial.print(" ");
            Serial.print(calibData.accel_offset_y); Serial.print(" ");
            Serial.print(calibData.accel_offset_z); Serial.print(" ");

            Serial.print("\nGyro: ");
            Serial.print(calibData.gyro_offset_x); Serial.print(" ");
            Serial.print(calibData.gyro_offset_y); Serial.print(" ");
            Serial.print(calibData.gyro_offset_z); Serial.print(" ");

            Serial.print("\nMag: ");
            Serial.print(calibData.mag_offset_x); Serial.print(" ");
            Serial.print(calibData.mag_offset_y); Serial.print(" ");
            Serial.print(calibData.mag_offset_z); Serial.print(" ");

            Serial.print("\nAccel Radius: ");
            Serial.print(calibData.accel_radius);

            Serial.print("\nMag Radius: ");
            Serial.print(calibData.mag_radius);

            Serial.println();
    }
}

void BNO055::QuickCalibration(adafruit_bno055_offsets_t calibData_old) {
    bno.setSensorOffsets(calibData_old);
}
