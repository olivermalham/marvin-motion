#ifndef WHEEL_H

#define WHEEL_H

// NOTE: All distances are in encoder counts, time is in servo ticks (normally 50Hz / 20ms per tick)
class WheelClass {

  public:
    int PWM_max;// = 1024;
    int PWM_offset;// = 80;
    float PWM_convert;// = 5.14;
    float V_max;// = 1700.0;
    float A_max;// = 566.7;
    float D_max;// = 2550.0;

    unsigned int outputA_pin;
    unsigned int outputB_pin;
    unsigned int pwm_slice;

    unsigned int encoderA_pin;
    unsigned int encoderB_pin;

    int direction = 1;
    float distance = 0;
    float distance_target = 0;
    float distance_actual = 0;
    float distance_error = 0;
    float distance_co = 0.001;
    
    // Encoder ticks recorded (not currently used in motion control)
    int distance_encoder = 0;
        
    float velocity = 0.0;
    float velocity_corrected = 0.0;
    int pwm = 0;

    bool encoderA_last = false;
    bool encoderB_last = false;

    WheelClass(void);
    void move(float distance, float scale);
    void stop(void);
    void reset(void);
    void set_pins(int outA, int outB, int encoderA, int encoderB);
    void update_distance(float delta);
    int servo_tick(void);
    void encoder_tick(void);

  private:
    void trapezoid(void);
    void triangle(void);
    void update_motor(void);    
};
#endif
