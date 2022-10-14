#include <SparkFunMPU9250-DMP.h>
#include <vector>

using namespace std;

class MPU9250_Qavairable : public MPU9250_DMP {

    public :
    float gravx, gravy, gravz;
    float dqw,dqx,dqy,dqz;
    unsigned char _address;
    vector<float> gyro = {0,0,0};
    vector<vector<float>> prev_gyros = {
        {0,0,0},
        {0,0,0},
        {0,0,0},
        {0,0,0}
    };    
    vector<float> angular_accel = {0,0,0};
    int prev_updated_micros = 0;

    // call update in loop() (highest freqency loop, somethime need to be threaded.
    void updateInfo(void (functionWhenUpdated(void))) {
        int fifobyte = fifoAvailable();
        if (fifobyte > 512) {
            Serial.print("[CAUTION] IMU buffer begin blocking:");
            Serial.println(fifobyte);
        }
        if (fifobyte)
        {
            // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
            // while( dmpUpdateFifo() != INV_SUCCESS);
            int retval = dmpUpdateFifo();
            if ( retval == INV_SUCCESS) {
            // computeEulerAngles can be used -- after updating the
            // quaternion values -- to estimate roll, pitch, and yaw
                computeEulerAngles();
                computeGravityDirection();
                computeAngularAccel();
                functionWhenUpdated();
            } else {
                // Serial.print("fifo update failed:");
                // Serial.println(fifobyte);
            }
        } else {
            // Serial.print("not available:");
            // Serial.println(fifobyte);
        }
    }

    void updateInfo() {
        updateInfo([]{});
    }

    void computeGravityDirection () {
        dqw = qToFloat(qw, 30);
        dqx = qToFloat(qx, 30);
        dqy = qToFloat(qy, 30);
        dqz = qToFloat(qz, 30);

        gravx = 2 * (dqx * dqz - dqw * dqy);
        gravy = 2 * (dqw * dqx + dqy * dqz);
        gravz = dqw * dqw - dqx * dqx - dqy * dqy + dqz * dqz;
    }

    void computeAngularAccel() {
        gyro = {calcGyro(gx), calcGyro(gy) ,calcGyro(gz)};
        float dt = (micros() - prev_updated_micros) / 1000000.0;

        // normal diff
        angular_accel = {
            (gyro[0] - prev_gyros[3][0]) / (float)dt,
            (gyro[1] - prev_gyros[3][1]) / (float)dt,
            (gyro[2] - prev_gyros[3][2]) / (float)dt
        };

        // 5 point stencil
        // angular_accel = {
        //     (-gyro[0] + 8 * prev_gyros[3][0] - 8 * prev_gyros[1][0] + prev_gyros[0][0]) / (12 *(float)dt),
        //     (-gyro[1] + 8 * prev_gyros[3][1] - 8 * prev_gyros[1][1] + prev_gyros[0][1]) / (12 *(float)dt),
        //     (-gyro[2] + 8 * prev_gyros[3][2] - 8 * prev_gyros[1][2] + prev_gyros[0][2]) / (12 *(float)dt)
        // };
        
        prev_gyros.erase(prev_gyros.begin());
        prev_gyros.push_back(gyro);
        prev_updated_micros = micros();

    }

    void setupIMUForDMP(unsigned char address) {
        _address = address;
        if (begin(address) != INV_SUCCESS)
        {
            bool error_blink = false;
            while (1)
            {
            Serial.println("Unable to communicate with MPU-9250");
            Serial.println("Check connections, and try again.");
            Serial.println();
            digitalWrite(13,error_blink == false ? 0 : 1);
            error_blink = !error_blink;
            delay(1000);
            }
        }
        else {
            // Serial.println((String)address + " connection success");
        }
        // Gyro options are +/- 250, 500, 1000, or 2000 dps
        setGyroFSR(2000); // Set gyro to 2000 dps
        // Accel options are +/- 2, 4, 8, or 16 g
        setAccelFSR(16); // Set accel to +/-2g
        setSampleRate(500);
        setCompassSampleRate(10); // Set mag rate to 10Hz
        dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL | 
               DMP_FEATURE_SEND_RAW_ACCEL | 
               DMP_FEATURE_SEND_RAW_GYRO , // Use gyro calibration
               200); // Set DMP FIFO rate to 200 Hz
    }

    vector<float> getGravityVector() {
        return {gravx, gravy, gravz};
    }

    vector<float> getEulerAngles() {
        return {roll, pitch, yaw};
    }

    vector<float> getLinearAccel() {
        float accelX = calcAccel(ax);
        float accelY = calcAccel(ay);
        float accelZ = calcAccel(az);
        return {accelX - gravx, accelY - gravy, accelZ - gravz};
    }

    vector<float> getAngularAccel() {
        return angular_accel;
    }
    
    vector<float> getAngularAccelRPSS() {
        return {angular_accel[0] * (float)(PI / 180.0), angular_accel[1] * (float)(PI / 180.0), angular_accel[2] * (float)(PI / 180.0)};
    }

    vector<float> getAngularVel() {
        return gyro;
    }

    vector<float> getAngularVelRPS() {
        return {gyro[0] * (float)(PI / 180.0), gyro[1] * (float)(PI / 180.0), gyro[2] * (float)(PI / 180.0)};
    }

    vector<float> getQuaternion() {
        return {dqw,dqx,dqy,dqz};
    }
};