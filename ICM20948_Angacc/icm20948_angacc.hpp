#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <vector>

using namespace std;


class ICM20948_ANGACC : public Adafruit_ICM20948 {

    public:
        float gravx, gravy, gravz;
        float dqw,dqx,dqy,dqz;
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t prev_gyro;
        sensors_event_t mag;
        sensors_event_t temp;
        unsigned char _address;

        vector<float> angular_accel_normal_diff = {0,0,0};
        vector<float> angular_accel_5point = {0,0,0};
        int prev_updated_micros = 0;

        ICM20948_ANGACC() : Adafruit_ICM20948() {}

        void readSensor() {
            getEvent(&accel, &gyro, &temp, &mag);
        }

        void computeAngularAccel() {
            // diff one-dimentional
            float dt = (micros() - prev_updated_micros) / 1000000.0;
            angular_accel_normal_diff = {(gyro.gyro.x - prev_gyro.gyro.x) / (float)dt, (gyro.gyro.y - prev_gyro.gyro.y) / float(dt), (gyro.gyro.z - prev_gyro.gyro.z) / float(dt)};
            prev_gyro = gyro;

            // 5 point stencil
            // angular_accel_5point = {
            //     (-gyro[0] + 8 * prev_gyros[3][0] - 8 * prev_gyros[1][0] + prev_gyros[0][0]) / (12 *(float)dt),
            //     (-gyro[1] + 8 * prev_gyros[3][1] - 8 * prev_gyros[1][1] + prev_gyros[0][1]) / (12 *(float)dt),
            //     (-gyro[2] + 8 * prev_gyros[3][2] - 8 * prev_gyros[1][2] + prev_gyros[0][2]) / (12 *(float)dt)
            // };

            // prev_gyros.erase(prev_gyros.begin());
            // prev_gyros.push_back(gyro);

            prev_updated_micros = micros();
        }

        int checkIMU(int CS_PIN) {
            int status = begin_SPI(CS_PIN);
            Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
                i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, ICM20X_B0_WHOAMI);

            _setBank(0);
            uint8_t chip_id_ = chip_id.read();
            // This returns true when using a 649 lib with a 948
            if ((chip_id_ != ICM20649_CHIP_ID) && (chip_id_ != ICM20948_CHIP_ID)) {
                return -1;
            }
            return 0;
        }

        int setupIMU(int CS_PIN) {
            int status = begin_SPI(CS_PIN);
            Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
                i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, ICM20X_B0_WHOAMI);

            _setBank(0);
            uint8_t chip_id_ = chip_id.read();
            // This returns true when using a 649 lib with a 948
            if ((chip_id_ != ICM20649_CHIP_ID) && (chip_id_ != ICM20948_CHIP_ID)) {
                return -1;
            }
            if (status<0)
            {
                return status;
            }
            else {
                Serial.println("ICM20948 connection success");
            }
            // setting of imu can be changed here.
            // calibrateAccel();
            // calibrateGyro();
            setAccelRange(ICM20948_ACCEL_RANGE_16_G);
            Serial.print("Accelerometer range set to: ");
            switch (getAccelRange()) {
            case ICM20948_ACCEL_RANGE_2_G:
                Serial.println("+-2G");
                break;
            case ICM20948_ACCEL_RANGE_4_G:
                Serial.println("+-4G");
                break;
            case ICM20948_ACCEL_RANGE_8_G:
                Serial.println("+-8G");
                break;
            case ICM20948_ACCEL_RANGE_16_G:
                Serial.println("+-16G");
                break;
            }
            setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
            Serial.print("Gyro range set to: ");
            switch (getGyroRange()) {
            case ICM20948_GYRO_RANGE_250_DPS:
                Serial.println("250 degrees/s");
                break;
            case ICM20948_GYRO_RANGE_500_DPS:
                Serial.println("500 degrees/s");
                break;
            case ICM20948_GYRO_RANGE_1000_DPS:
                Serial.println("1000 degrees/s");
                break;
            case ICM20948_GYRO_RANGE_2000_DPS:
                Serial.println("2000 degrees/s");
                break;
            }           
            setAccelRateDivisor(1);
            uint16_t accel_divisor = getAccelRateDivisor();
            float accel_rate = 1125 / (1.0 + accel_divisor);

            Serial.print("Accelerometer data rate divisor set to: ");
            Serial.println(accel_divisor);
            Serial.print("Accelerometer data rate (Hz) is approximately: ");
            Serial.println(accel_rate);

            setGyroRateDivisor(1);
            uint8_t gyro_divisor = getGyroRateDivisor();
            float gyro_rate = 1100 / (1.0 + gyro_divisor);

            Serial.print("Gyro data rate divisor set to: ");
            Serial.println(gyro_divisor);
            Serial.print("Gyro data rate (Hz) is approximately: ");
            Serial.println(gyro_rate); 
            // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
            Serial.print("Magnetometer data rate set to: ");
            switch (getMagDataRate()) {
            case AK09916_MAG_DATARATE_SHUTDOWN:
                Serial.println("Shutdown");
                break;
            case AK09916_MAG_DATARATE_SINGLE:
                Serial.println("Single/One shot");
                break;
            case AK09916_MAG_DATARATE_10_HZ:
                Serial.println("10 Hz");
                break;
            case AK09916_MAG_DATARATE_20_HZ:
                Serial.println("20 Hz");
                break;
            case AK09916_MAG_DATARATE_50_HZ:
                Serial.println("50 Hz");
                break;
            case AK09916_MAG_DATARATE_100_HZ:
                Serial.println("100 Hz");
                break;
            }
            Serial.println();
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
            return {gyro.gyro.x,gyro.gyro.y,gyro.gyro.z};
        }

        vector<float> getQuaternion() {
            return {dqw,dqx,dqy,dqz};
        }

        vector<float> getAccel() {
            return {accel.acceleration.x,accel.acceleration.y,accel.acceleration.z};
        }
        vector<float> getMag() {
            return {mag.magnetic.x,mag.magnetic.y,mag.magnetic.z};
        }
};