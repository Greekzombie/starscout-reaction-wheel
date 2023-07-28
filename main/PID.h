class PID
{
    public:
        PID();

        // Returns the manipulated variable given a setpoint and current process value
        // w_rocket must be given in rad/s
        // dt must be given in s
        // output is given in rad/s
        float calculate( float w_rocket, float dt );

    private:
        float _Kp;  // proportional gain
        float _Ki;  // integral gain
        float _Kd;  // derivative gain
        float _min; // min value of manipulated variable
        float _max; // max value of manipulated variable

        float _prev_error;
        float _integral;
        float _setpoint;
};
