#include <Adafruit_BNO055.h>
#include "PID.h"

int direction_pin = 10;
int pwm_pin = 11;
bool flag = HIGH;
float dt = 0.01;
float max_speed_motor = 5000 * PI / 30; // rpm -> rad/s

// We create an instance of the PID controller
PID controller = PID();

// We create an instance of the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(); // 55, 0x28, &Wire

void setup() {
    Serial.begin(115200);
    pinMode(direction_pin, OUTPUT); //direction control PIN 10 with direction wire 
    pinMode(pwm_pin, OUTPUT);       //PWM PIN 11  with PWM 
    
}

void loop() {
    // We first obtain the gyroscope measurement of angular velocity of the rocket
    float w_rocket = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE)[2] * PI /180; // [rad/s]

    // We then update the PID controller and obtain the angular velocity to spin the reaction wheel at
    float w_rw = controller.calculate(w_rocket, dt);

    // Depending on the sign of w_rocket, we set the direction of the motor
    if (w_rw > 0){
        flag = HIGH;
    }
    else {
        flag = LOW;
    }
    digitalWrite(10, flag);

    // Map angular velocity to a number between 0 and 255. Max speed of motor corresponds to 5000 rpm = 
    int signal_motor = map(int(abs(w_rw)), 0, max_speed_motor, 0, 255); // map(value, fromLow, fromHigh, toLow, toHigh)
    analogWrite(pwm_pin, signal_motor);                     // input speed (must be int)
    Serial.println(signal_motor);

    // Introduce a delay into the system
    delay(int(dt*1000));

}

float get_gyroscope_measurement(){
    return 5;

}
