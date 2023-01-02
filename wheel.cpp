#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "wheel.h"
#include "PID.h"

WheelClass::WheelClass(void){
  distance = 0.0;
  distance_target = 0.0;
  distance_last = 0.0;
  direction = 1;
  velocity = 0.0;
  velocity_av = 0.0;
  velocity_coef = 0.3;
  frames = 1;
  pwm = 0;
  encoderA_last = 0;
  encoderB_last = 0;
  outputA_pin = 0;
  outputB_pin = 0;
  pwm_slice = 0;

  PWM_max = 1000; //1022;
  PWM_offset = 400;

  V_max = 1300.0; // Maximum encoder ticks per second
  A_max = V_max / 150.0; // FIXME!
  D_max = V_max * 150 * 0.5; // FIXME!

  PWM_convert = float(PWM_max - PWM_offset) / V_max;

  pid = new PID(&velocity_actual, &velocity_corrected, &velocity, 1, 0, 0, DIRECT);
  pid->SetOutputLimits(-1.0*V_max, V_max);
  pid->SetMode(AUTOMATIC);
}

void WheelClass::reset(void){
  PWM_max = 1000; //1022;
  PWM_offset = 400;

  V_max = 13.0; //250.0 / 50; // 34
  A_max = V_max / 150.0; // 11.333
  D_max = V_max * 150 * 0.5; // (A_max = 150)

  PWM_convert = float(PWM_max - PWM_offset) / V_max;

  velocity = 0.0;
  velocity_av = 0.0;
  velocity_coef = 0.3;
  distance = 0.0;
  distance_target = 0.0;
  distance_last = 0.0;
  frames = 1;

  pwm = 0;
  encoderA_last = gpio_get(encoderA_pin);
}

// TODO: Clean this up! "Scale" should be a velocity in ticks / second
// TODO: Worry about the trapezoid motion control later!
void WheelClass::move(float distance, float scale){
  distance_target = distance;
  D_max = D_max * scale;
  V_max = V_max * scale;
  A_max = A_max * scale;

  if(distance < 0) direction = -1;
  else direction = 1;

  // If total distance is less than twice the distance required to ramp up to the maximum velocity, then 
  // we have a triangular motion profile. Recalculate accordingly.
  if(D_max > distance / 2){ /* TODO! */};

  velocity = scale;
  
}

// Perform an immediate stop of the motor
void WheelClass::stop(void){
  velocity = 0.0;
  update_motor();
}

void WheelClass::set_pins(int outA, int outB, int encoderA, int encoderB){
  outputA_pin = outA;
  outputB_pin = outB;
  encoderA_pin = encoderA;
  encoderB_pin = encoderB;

  gpio_init(outputA_pin);
  gpio_init(outputB_pin);
  
  // Tell GPIO 0 and 1 they are allocated to the PWM
  gpio_set_function(outputA_pin, GPIO_FUNC_PWM);
  gpio_set_function(outputB_pin, GPIO_FUNC_PWM);

  // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
  pwm_slice = pwm_gpio_to_slice_num(outputA_pin);

  // 10-bit resolution, 16KHz PWM frequency assuming 133MHz system clock (CHECK!)
  pwm_set_wrap(pwm_slice, 1023);
  pwm_set_clkdiv(pwm_slice, 8.0);

  // Initialise both channels to output low
  pwm_set_chan_level(pwm_slice, PWM_CHAN_A, 0);
  pwm_set_chan_level(pwm_slice, PWM_CHAN_B, 0);

  // Set the PWM running
  pwm_set_enabled(pwm_slice, true);

  gpio_init(encoderA);
  gpio_set_dir(encoderA, GPIO_IN);
  gpio_init(encoderB);
  gpio_set_dir(encoderB, GPIO_IN);
}

// Polled by the main loop to catch encoder ticks (interrupt driven would be better, but couldn't make it work)
void WheelClass::encoder_tick(void){
  bool encoderA_current = gpio_get(encoderA_pin);
  bool encoderB_current = gpio_get(encoderB_pin);
  
  // If the motor isn't powered, ignore the encoder
  if(pwm <= 0) return;

  // Only trigger on a rising edge
  if(encoderA_current & !encoderA_last){
    if(encoderB_current){
      distance--;
    } else {
      distance++;
    }
  }
  encoderA_last = encoderA_current;
}

void WheelClass::update_distance(float delta){
  distance += delta;
}

int WheelClass::servo_tick(unsigned long frame_time){

  velocity_actual = (distance - distance_last)/(frame_time - last_frame) * 1000.0; // Time is in milliseconds
  distance_last = distance;
  last_frame = frame_time;

  if(distance <= distance_target) {
//    if(distance_target > 2*D_max) trapezoid();
//    else triangle();
      trapezoid();
  } else {
    pwm = 0;
    velocity = 0;
    pwm_set_chan_level(pwm_slice, PWM_CHAN_A, 0);
    pwm_set_chan_level(pwm_slice, PWM_CHAN_B, 0);
  }
  return 0;
}

void WheelClass::trapezoid(void){
    // Calculate the target velocity
//    if(distance < D_max){
//        // Ramp up speed to maximum
//        velocity += A_max;
//        if(velocity > V_max) velocity = V_max;
//    } else if(distance >= D_max && distance < distance_target - D_max) {
//        // Constant speed
//    } else {
//        // Ramp down
//        velocity -= A_max;
//        if(velocity <= 0.0) velocity = 0.0;
//        velocity = 0.0;
//    }

//    velocity_actual = velocity + 1.0;
    pid->Compute();
    update_motor();
}

// Cumulative moving average of velocity
float WheelClass::velocity_average(float new_velocity){
  float average = (new_velocity + frames*velocity_av)/(frames + 1);
  velocity_av = average;
  frames++;
  return(average);
}

void WheelClass::triangle(void){
      // Calculate the target velocity
      if(distance < distance_target/2){
          // Ramp up speed to maximum
          velocity += A_max;
          if(velocity > V_max) velocity = V_max;
      } else {
          // Ramp down
          velocity -= A_max;
          if(velocity <= 0.0) velocity = 0.0;
      }

      update_motor();
      distance_last = distance;
}

// Update motor contains proportional control code
void WheelClass::update_motor(void){

  pwm = int(PWM_convert * (velocity + velocity_corrected)) + PWM_offset;

  // Clamp PWM
  if(pwm > 1022) pwm = PWM_max;
  if(pwm < 0) pwm = 0;

  // TODO: Not sure if I should be using 0 here, or PWM MAX instead - had to do that with the Python version.
  if (direction > 0){
    pwm_set_chan_level(pwm_slice, PWM_CHAN_A, pwm);
    pwm_set_chan_level(pwm_slice, PWM_CHAN_B, 0);
  } else {
    pwm_set_chan_level(pwm_slice, PWM_CHAN_A, 0);
    pwm_set_chan_level(pwm_slice, PWM_CHAN_B, pwm);
  }
}
