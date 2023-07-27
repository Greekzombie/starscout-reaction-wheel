int direction_pin = 10;
int pwm_pin = 11;

// HIGH CLOCKWISE/NEGATIVE
// LOW ANTICLOCKWISE/POSITIVE
bool flag = LOW; 
float dt = 0.5;

void setup() {
    Serial.begin(115200);
    pinMode(direction_pin, OUTPUT); //direction control PIN 10 with direction wire 
    pinMode(pwm_pin, OUTPUT);       //PWM PIN 11  with PWM 
    digitalWrite(direction_pin, flag);
    
}

int i = 0;

void loop() {
    analogWrite(pwm_pin, 240);                     // input speed (must be int)
    
    for(int j = 0;j<8;j++)  {
    i += pulseIn(9, HIGH, 500000); //SIGNAL OUTPUT PIN 9 with  white line,cycle = 2*i,1s = 1000000us，Signal cycle pulse number：27*2
    }
    i = i >> 3;
    Serial.print(111111 / i); //speed   r/min  (60*1000000/(45*6*2*i))
    Serial.println("  r/min");
    i = 0;

    // Introduce a delay into the system
    delay(int(dt*1000));

}

