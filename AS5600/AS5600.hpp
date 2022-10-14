#ifndef AS5600_hpp
#define AS5600_hpp
#endif

#include <Wire.h>
#include <Encoder.h>

#define AS5600_AS5601_DEV_ADDRESS 0x36
#define AS5600_AS5601_REG_RAW_ANGLE    0x0C
#define AS5600_AS5601_REG_CONF          0x07
#define AS5600_AS5601_REG_STATUS        0x0B
#define AS5601_REG_ABN                  0x09

class AS5600 : public Encoder {
    public:

        AS5600(uint8_t pin1, uint8_t pin2) : Encoder(pin1, pin2) {

        }
        void setup() {
            Wire.begin();
            Wire.setClock(400000);
            byte error;
            uint8_t data;

            // Read AS5601 status register
            // Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
            // Wire.write(AS5600_AS5601_REG_STATUS);
            // Wire.endTransmission(false);
            // Wire.requestFrom(AS5600_AS5601_DEV_ADDRESS, 1);
            // data = Wire.read();
            // data &= 0x38;
            // if (data != 0x20)
            // {
            //     Serial.print("Magnet error : ");
            //     if ( !(data & 0x20) )
            //     Serial.println("Magnet was not detected");
            //     if (data & 0x10)
            //     Serial.println("Magnet too weak");
            //     if (data & 0x08)
            //     Serial.println("Magnet too strong");
            //     Serial.println("Stop");
            //     for (;;);
            // }
            // else
            //     Serial.println("Magnet : OK");

            // Set AS5601 resolution 2048ppr
            Serial.println("Change Encoder resolution 2048ppr");
            Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
            Wire.write(AS5601_REG_ABN);
            Wire.write(0b00000010);   // ABN(3:0)
            error = Wire.endTransmission();
            if (error){
                Serial.print("error=");Serial.println(error);
            }
        }
};