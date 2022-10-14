#include <Arduino.h>
#include <pololu.h>

const int PWM_PIN = 2;
const int ENCORDOR_PIN_1 = 3;
const int ENCORDOR_PIN_2 = 4;
const int OUT_PIN_A = 5;
const int OUT_PIN_B = 6;


Pololu motor(PWM_PIN, OUT_PIN_A, OUT_PIN_B, ENCORDOR_PIN_1, ENCORDOR_PIN_2);

void init() {
    motor.setup();
}

void update() {
    motor.update();
    motor.updateSpeed();
}