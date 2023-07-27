/* Sampling rate between 250 and 500 Hz (4 and 2 ms).
 * 
 * 
 * 
 */

#include <Adafruit_BNO055.h>
#include <math.h>
#include "PID.h"

// These are the pins to which the motor is connected
int direction_pin = 10;
int pwm_pin = 11;

// flag is used to write to the direction pin
// HIGH is ANTICLOCKWISE
// LOW is CLOCKWISE
bool flag;

// Time between every loop [ms]
float dt = 1;

// Maximum speed allowed by the motor
int max_speed_motor = 749; // [rad/s]   7155 * PI / 30 = 749   rpm -> rad/s

// Boolean which controls when we initialise PID controller calculations
bool start_controller = false;

// We create an instance of the PID controller
PID controller = PID();

// We create an instance of the BNO055 sensor
// POSITIVE ANGULAR VELOCITY means ANTICLOCKWISE
// NEGATIVE ANGULAR VELOCITY means CLOCKWISE
Adafruit_BNO055 bno = Adafruit_BNO055(); // 55, 0x28, &Wire

void setup() {
    Serial.begin(115200);
    pinMode(direction_pin, OUTPUT); //direction control PIN 10 with direction wire 
    pinMode(pwm_pin, OUTPUT);       //PWM PIN 11  with PWM 

    delay(500); // Wait for the sensor to initialize
    if (!bno.begin())
    {
      Serial.println("Could not find a valid BNO055 sensor, check wiring!");
      while (1);
    }
    Serial.println("BNO055 sensor initialized.");
    //int s = map(1000, 0, max_speed_motor, 0, 255);
    //s = constrain(255 - s, 0, 255);
    //Serial.println(s);
    
}

unsigned long prev_t;
unsigned long t;
void loop() {
    // We wait until we detect rocket launch before starting to take measurements.
    if(!start_controller){
        bool decision = has_rocket_launched();
        if(decision){
            start_controller = true;
            delay(200); 
            prev_t = millis();
        }
    }

    else {
        // We first obtain the gyroscope measurement of angular velocity of the rocket
        float w_rocket = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE)[2] * PI /180; // [rad/s]

        // We then update the PID controller and obtain the angular velocity to spin the reaction wheel at
        t = millis();
        float w_rw = controller.calculate(w_rocket, (t-prev_t)*1E-3);
        prev_t = t;

        // Depending on the sign of w_rocket, we set the direction of the motor
        if (w_rw > 0){
            flag = HIGH;
        }
        else {
            flag = LOW;
        }
        digitalWrite(10, flag);

        // Map angular velocity to a number between 0 and 255. 
        int signal_motor = map(int(abs(w_rw)), 0, max_speed_motor, 0, 255); // map(value, fromLow, fromHigh, toLow, toHigh)
        signal_motor = constrain(255 - signal_motor, 0, 255);

        analogWrite(pwm_pin, signal_motor);                     // input speed (must be int)
        Serial.println(signal_motor);

        // Introduce a delay into the system
        delay(dt);

    }

}

bool has_rocket_launched(){
    float norm_acc_rocket = get_norm_linear_acc();
    float acc_trigger = 6.0; // [m/s**2]

    bool decision = false;
    int n_tests = 10;
    int time_between_tests = 10; // [ms]
    
    int n_true_positives = 0;
    if (norm_acc_rocket > acc_trigger){
      
        for (int i = 0; i < n_tests; i++) {
          norm_acc_rocket = get_norm_linear_acc();
          
          if (norm_acc_rocket > acc_trigger){
             n_true_positives += 1;
          }

          delay(time_between_tests);
        }

        if (n_true_positives > n_tests-2){
          decision = true;
        }
    }
    
    return decision;
    
}

float get_norm_linear_acc(){
    float acc_rocket_x = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL)[0]; // [m/s**2]
    float acc_rocket_y = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL)[1]; // [m/s**2]
    float acc_rocket_z = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL)[2]; // [m/s**2]

    float norm_acc_rocket = sqrt(pow(acc_rocket_x, 2) + pow(acc_rocket_y, 2) + pow(acc_rocket_z, 2));

    return norm_acc_rocket;
}
