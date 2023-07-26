#include <Adafruit_BNO055.h>
#include <cmath>
#include "PID.h"

int direction_pin = 10;
int pwm_pin = 11;

// HIGH CLOCKWISE/NEGATIVE
// LOW ANTICLOCKWISE/POSITIVE
bool flag = LOW;
float dt = 0.5;
float max_speed_motor = 5000 * PI / 30; // rpm -> rad/s
bool start_controller = false;

// We create an instance of the PID controller
PID controller = PID();

// We create an instance of the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(); // 55, 0x28, &Wire

void setup() {
    Serial.begin(9600);
    pinMode(direction_pin, OUTPUT); //direction control PIN 10 with direction wire 
    pinMode(pwm_pin, OUTPUT);       //PWM PIN 11  with PWM 
    digitalWrite(direction_pin, flag);

    delay(500); // Wait for the sensor to initialize
    if (!bno.begin())
    {
      Serial.println("Could not find a valid BNO055 sensor, check wiring!");
      while (1);
    }
    Serial.println("BNO055 sensor initialized.");
    
}

int i = 0;

void loop() {
    // We wait until we detect rocket launch before starting to take measurements.
    if(!start_controller){
        bool decision = has_rocket_launched();
        if(decision){
            start_controller = true;
            delay(500); 
        }
    }

    else {
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

}

bool has_rocket_launched(){
    float acc_rocket_x = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL)[0]; // [m/s**2]
    float acc_rocket_y = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL)[1]; // [m/s**2]
    float acc_rocket_z = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL)[2]; // [m/s**2]

    float norm_acc_rocket = sqrt(pow(acc_rocket_x, 2) + pow(acc_rocket_y, 2) + pow(acc_rocket_z, 2));
    float acc_trigger = 30.0; // m/s**2 More or less 3g

    bool decision;
    if (norm_acc_rocket > acc_trigger){
        decision = true;
    }
    else {
        decision = false;
    }

    return decision;
    
}
