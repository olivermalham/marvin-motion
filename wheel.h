#ifndef WHEEL_H

#define WHEEL_H

#define WHEEL_COUNT 6

#include "PID.h"

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
    float distance = 0.0;
    float distance_target = 0.0;
    float distance_last = 0.0;

    // Encoder ticks recorded (not currently used in motion control)
    int distance_encoder = 0;
    int frames = 1;

    // All velocities are in encoder ticks per second
    float velocity = 0.0;           // Commanded velocity
    float velocity_actual = 0.0;    // Measured velocity (averaged over last 8 measurements)
    float velocity_corrected = 0.0; // Velocity corrected by PID

    float velocity_av = 0.0;
    float velocity_coef = 0.2;
    int pwm = 0;

    bool encoderA_last = false;
    bool encoderB_last = false;
    unsigned long last_frame = 0; // Time stamp for the previous servo tick. Used to calc velocity

    WheelClass(void);
    void move(float distance, float scale);
    void stop(void);
    void reset(void);
    void set_pins(int outA, int outB, int encoderA, int encoderB);
    void update_distance(float delta);
    int servo_tick(unsigned long frame_time);
    void encoder_tick(void);

  private:
    void trapezoid(void);
    void trapezoid_pid(void);
    void triangle(void);
    void update_motor(void);
    float velocity_average(float);

    PID *pid;
};
#endif
