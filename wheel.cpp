#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "wheel.h"

WheelClass::WheelClass(void){
  distance = 0.0;
  distance_target = 0.0;
  distance_actual = 0.0;
  distance_error = 0.0;
  distance_co = 0.1;
  direction = 1;
  velocity = 0.0;
  pwm = 0;
  encoderA_last = 0;
  encoderB_last = 0;
  outputA_pin = 0;
  outputB_pin = 0;
  pwm_slice = 0;
}

void WheelClass::move(float distance, float scale){
  distance_target = distance;
  D_max = D_max * scale;
  V_max = V_max * scale;
  A_max = A_max * scale;

  if(distance < 0) direction = -1;
  else direction = 1;

  // If total distance is less than twice the distance required to ramp up to the maximum velocity, then 
  // we have a triangluar motion profile. Recalculate accordingly.
  if(D_max > distance / 2){ /* TODO! */};
  
}

// Perform an immediate stop of the motor
void WheelClass::stop(void){
  velocity = 0.0;
  velocity_corrected = 0.0;
  update_motor();
}

void WheelClass::reset(void){
  PWM_max = 1022;
  PWM_offset = 0;
  
  V_max = 250.0 / 50; // 34
  A_max = V_max / 150.0; // 11.333
  D_max = V_max * 150.0 * 0.5; // 51.0

  PWM_convert = float(PWM_max - PWM_offset) / V_max;

  velocity = 0.0;
  distance = 0.0;
  distance_target = 0.0;
  distance_actual = 0.0;
  distance_error = 0.0;
  distance_co = 0.1;

  pwm = 0;
  encoderA_last = gpio_get(encoderA_pin);
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

// Polled by the main loop to catch encoder ticks (interupt driven would be better, but couldn't make it work)
void WheelClass::encoder_tick(void){
  bool encoderA_current = gpio_get(encoderA_pin);
  bool encoderB_current = gpio_get(encoderB_pin);
  
  // If the motor isn't powered, ignore the encoder
  if(!pwm) return;

  // Only trigger on a rising edge
  if(encoderA_current & !encoderA_last){
    if(encoderB_current){
      distance_actual--;
    } else {
      distance_actual++;
    }
  }
  encoderA_last = encoderA_current;
}

void WheelClass::update_distance(float delta){
  distance_actual += delta;
}

int WheelClass::servo_tick(void){
  if(distance_actual <= distance_target)
    trapezoid();
//  if(distance_target > 2*D_max) return trapezoid();
//  else return triangle();
  return 0;
}

void WheelClass::trapezoid(void){
  if(distance_actual < D_max){
    // Ramp up speed to maximum
    velocity += A_max;
    if(velocity > V_max) velocity = V_max;

    // Update calculated distance so we can determine error
    distance += velocity;

  } else if(distance_actual >= D_max && distance_actual < distance_target - D_max) {
    // Constant velocity
    distance += velocity;
  } else {
    // Ramp down
    velocity -= A_max;
    if(velocity <= 0.0) velocity = 0.0;
    distance += velocity;
  }
  distance_error = distance - distance_actual;
  velocity_corrected = velocity + (distance_error * distance_co); // Very simple proportional control algorithm
  update_motor();
}

void WheelClass::triangle(void){
  update_motor();
}

void WheelClass::update_motor(){
  pwm = int(velocity_corrected * PWM_convert) + PWM_offset;
  if(velocity_corrected <= 0.0) pwm = 0;
  if(pwm > PWM_max) pwm = PWM_max;
  if(pwm < 0) pwm = 0;
  
  if (direction > 0){
    pwm_set_chan_level(pwm_slice, PWM_CHAN_A, pwm);
    pwm_set_chan_level(pwm_slice, PWM_CHAN_B, 0);
  } else {
    pwm_set_chan_level(pwm_slice, PWM_CHAN_A, 0);
    pwm_set_chan_level(pwm_slice, PWM_CHAN_B, pwm);
  }
}
