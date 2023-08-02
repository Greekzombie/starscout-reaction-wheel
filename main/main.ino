/* 
A PID controller is used to set the speed of a reaction wheel inside a rocket. Data from a gyroscope
attached to the rocket is used to stabilise the rocket's rotational motion.

With the current setup, a sampling rate of around 250-500 Hz can be achieved.
 */

#include <Adafruit_BNO055.h>
#include <math.h>
#include "PID.h"

// These are the pins to which the reaction wheel's motor is connected
int direction_pin = 5;
int pwm_pin = 6;

// flag is used to write to the direction pin of the motor
// HIGH is ANTICLOCKWISE
// LOW is CLOCKWISE
bool flag;

// Time delay between every loop to ensure enough time to analogWrite to the reaction wheel's motor
float dt = 1; // [ms]

// Maximum speed allowed by the motor connected to the reaction wheel
int max_speed_motor = 749; // [rad/s]   7155 * PI / 30 = 749   rpm -> rad/s

// Boolean which controls when we initialise PID controller calculations.
// We also define variables to shutdown controller after a certain amount of time passes
bool start_controller = false;
bool shutdown_controller = false;
unsigned long duration_controller_activation = 5*60*1000; // [ms]

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
    
}

// Variables to store time that passes.
unsigned long start_time_controller;
unsigned long prev_t;
unsigned long t;
void loop() {
    // We wait until we detect rocket launch before starting to take measurements.
    if(!start_controller || shutdown_controller){
        bool decision = has_rocket_launched();
        if(decision){
            // When we detect rocket has launched, we leave a time interval until the rocket's motor has burned
            start_controller = true;
            delay(200); 
            prev_t = millis();
            start_time_controller = millis();
        }
    }

    else {
        // We first obtain the gyroscope measurement of angular velocity of the rocket.
        // Depending on the orientation of the sensor relative to the rocket, a different element
        // of the gyroscope data vector will have to be used.
        float w_rocket = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE)[2] * PI /180; // [rad/s]

        // We then update the PID controller and obtain the angular velocity to spin the reaction wheel at.
        t = millis();
        float w_rw = controller.calculate(w_rocket, (t-prev_t)*1E-3); // units given to controller have to be SI
        prev_t = t;

        // Depending on the sign of w_rw, we set the direction of the motor. For example, if w_rw>0, this
        // means we want to spin reaction wheel anticlockwise, which requires flag set to HIGH.
        if (w_rw > 0){
            flag = HIGH;
        }
        else {
            flag = LOW;
        }
        digitalWrite(direction_pin, flag);

        // Map angular velocity to a number between 0 and 255. A thing to note is that analogWriting 0 to the motor is its
        // maximum speed, while writing 255 will make it stationary.
        int signal_motor = map(int(abs(w_rw)), 0, max_speed_motor, 0, 255); // map(value, fromLow, fromHigh, toLow, toHigh)
        signal_motor = constrain(255 - signal_motor, 0, 255);

        analogWrite(pwm_pin, signal_motor);                     
        Serial.println(signal_motor);

        // Introduce a delay into the system
        delay(dt);

        // If controller has been on for more than the duration specified by "duration_controller_activation" we shut it down
        // and power off the motor.
        if ((t-start_time_controller) > duration_controller_activation){
            shutdown_controller = true;
            analogWrite(pwm_pin, 255); 

        }

    }

}

bool has_rocket_launched(){
    // acc_trigger defines the linear acceleration needed to register the thrust of the motor while ignoring
    // acceleration caused by accidental jostling of the rocket. Should be at least 3g.
    float norm_acc_rocket = get_norm_linear_acc();
    float acc_trigger = 6.0; // [m/s**2]

    // In order to confirm the rocket has indeed launched, we perform several consecutive tests in a row, leaving
    // a time interval between them.
    bool decision = false;
    int n_consecutive_tests = 10;
    int time_interval_between_tests = 10; // [ms]
    
    int n_true_positives = 0;
    if (norm_acc_rocket > acc_trigger){
      
        for (int i = 0; i < n_consecutive_tests; i++) {
          norm_acc_rocket = get_norm_linear_acc();
          
          if (norm_acc_rocket > acc_trigger){
             n_true_positives += 1;
          }

          delay(time_interval_between_tests);
        }
        
        // In case of a sensor reading error, we only require that most of the measurements confirm a rocket launch.
        if (n_true_positives >= n_consecutive_tests-2){
          decision = true;
        }
    }
    
    return decision;
    
}

float get_norm_linear_acc(){
    // Obtain norm of linear acceleration. 
    // VECTOR_LINEARACCEL vector provides the linear acceleration of the sensor relative to its surroundings,
    // excluding the gravitational acceleration. When placed on a table, it measures close to 0.
    float acc_rocket_x = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL)[0]; // [m/s**2]
    float acc_rocket_y = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL)[1]; // [m/s**2]
    float acc_rocket_z = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL)[2]; // [m/s**2]

    float norm_acc_rocket = sqrt(pow(acc_rocket_x, 2) + pow(acc_rocket_y, 2) + pow(acc_rocket_z, 2));

    return norm_acc_rocket;
}
