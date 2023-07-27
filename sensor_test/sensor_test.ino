#include <Adafruit_BNO055.h>
#include <math.h>


// We create an instance of the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(); // 55, 0x28, &Wire

void setup() {
    Serial.begin(115200);

    delay(500); // Wait for the sensor to initialize
    if (!bno.begin())
    {
      Serial.println("Could not find a valid BNO055 sensor, check wiring!");
      while (1);
    }
    Serial.println("BNO055 sensor initialized.");
    
}

void loop() {
    // VECTOR_ACCELEROMETER measures 9.81 when placed stationary on a table
    // VECTOR_LINEARACCEL measures 0 when placed stationary on a table
    float acc_rocket_x = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL)[0]; // [m/s**2]
    float acc_rocket_y = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL)[1]; // [m/s**2]
    float acc_rocket_z = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL)[2]; // [m/s**2]

    float norm_acc_rocket = sqrt(pow(acc_rocket_x, 2) + pow(acc_rocket_y, 2) + pow(acc_rocket_z, 2));

    float w_rocket = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE)[2] * PI /180; // [rad/s]

    Serial.println(acc_rocket_z);
    Serial.println(w_rocket);
    Serial.println(" ");

    delay(100);
    
}
