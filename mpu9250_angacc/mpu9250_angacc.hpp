#include <MPU9250.h>
#include <vector>

using namespace std;

class MPU9250_ANGACC : public MPU9250 {

    public :
    float gravx, gravy, gravz;
    float dqw,dqx,dqy,dqz;
    unsigned char _address;
    vector<float> gyro = {0,0,0};
    vector<float> prev_gyro = {0,0,0};
    vector<vector<float>> prev_gyros = {
        {0,0,0},
        {0,0,0},
        {0,0,0},
        {0,0,0}
    };
    vector<float> angular_accel_normal_diff = {0,0,0};
    vector<float> angular_accel_5point = {0,0,0};
    int prev_updated_micros = 0;

    MPU9250_ANGACC(TwoWire &bus,uint8_t address) : MPU9250(bus, address)
    {

    }
    MPU9250_ANGACC(SPIClass &bus,uint8_t csPin) : MPU9250(bus, csPin) {

    }
    
    void computeGravityDirection () {
        // dqw = qToFloat(qw, 30);
        // dqx = qToFloat(qx, 30);
        // dqy = qToFloat(qy, 30);
        // dqz = qToFloat(qz, 30);

        // gravx = 2 * (dqx * dqz - dqw * dqy);
        // gravy = 2 * (dqw * dqx + dqy * dqz);
        // gravz = dqw * dqw - dqx * dqx - dqy * dqy + dqz * dqz;
    }

    void computeAngularAccel() {
        // diff one-dimentional
        gyro = {_gx, _gy ,_gz};
        float dt = (micros() - prev_updated_micros) / 1000000.0;
        angular_accel_normal_diff = {(gyro[0] - prev_gyro[0]) / (float)dt, (gyro[1] - prev_gyro[1]) / float(dt), (gyro[2] - prev_gyro[2]) / float(dt)};
        prev_gyro = gyro;

        // 5 point stencil
        // angular_accel_5point = {
        //     (-gyro[0] + 8 * prev_gyros[3][0] - 8 * prev_gyros[1][0] + prev_gyros[0][0]) / (12 *(float)dt),
        //     (-gyro[1] + 8 * prev_gyros[3][1] - 8 * prev_gyros[1][1] + prev_gyros[0][1]) / (12 *(float)dt),
        //     (-gyro[2] + 8 * prev_gyros[3][2] - 8 * prev_gyros[1][2] + prev_gyros[0][2]) / (12 *(float)dt)
        // };

        prev_gyros.erase(prev_gyros.begin());
        prev_gyros.push_back(gyro);

        prev_updated_micros = micros();
    }

    int setupIMU() {
        int status = begin();
        if (status<0)
        {
            // bool led = false;
            // while(1) {
            //     Serial.println("IMU initialization unsuccessful");
            //     Serial.println("Check IMU wiring or try cycling power");
            //     Serial.print("Status: ");
            //     Serial.println(status);
            //     delay(200);
            //     digitalWrite(13,led);
            //     led = !led;
            // }
            return status;
        }
        else {
            Serial.println("imu connection success");
        }
        // setting of imu can be changed here.
        // calibrateAccel();
        calibrateGyro();
        setAccelRange(ACCEL_RANGE_16G);
        setGyroRange(GYRO_RANGE_2000DPS);
        return status;
    }

    vector<float> getGravityVector() {
        return {gravx, gravy, gravz};
    }


    // vector<float> getLinearAccel() {
    //     float accelX = calcAccel(ax);
    //     float accelY = calcAccel(ay);
    //     float accelZ = calcAccel(az);
    //     return {accelX - gravx, accelY - gravy, accelZ - gravz};
    // }

    vector<float> getAngularAccelRPSS() {
        return angular_accel_normal_diff;
    }
    vector<float> getAngularAccelRPSS_5point() {
        return angular_accel_5point;
    }

    vector<float> getAngularVelRPS() {
        return {_gx,_gy,_gz};
    }

    vector<float> getQuaternion() {
        return {dqw,dqx,dqy,dqz};
    }

    vector<float> getAccel() {
        return {_ax,_ay,_az};
    }
    vector<float> getMag() {
        return {_hx,_hy,_hz};
    }
};