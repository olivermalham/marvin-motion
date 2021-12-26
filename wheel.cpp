#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "wheel.h"


WheelClass::WheelClass(void){
  distance = 0.0;
  distance_target = 0.0;
  direction = 1;
  velocity = 0.0;
  velocity_coef = 0.01;
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

  velocity = V_max;
  
}

// Perform an immediate stop of the motor
void WheelClass::stop(void){
  velocity = 0.0;
  update_motor(0.0);
}

void WheelClass::reset(void){
  PWM_max = 900; //1022;
  PWM_offset = 0;
  
  V_max = 15.0; //250.0 / 50; // 34
  A_max = V_max / 30.0; // 11.333
  D_max = V_max * A_max * 0.5; // (A_max = 150)

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

int WheelClass::servo_tick(void){
  if(distance <= distance_target)
    trapezoid_pid();
//  if(distance_target > 2*D_max) return trapezoid();
//  else return triangle();
  return 0;
}

void WheelClass::trapezoid(void){
  if(distance < D_max){
    // Ramp up speed to maximum
    velocity += A_max;
    if(velocity > V_max) velocity = V_max;
  } else if(distance >= D_max && distance < distance_target - D_max) {
    // Constant velocity
  } else {
    // Ramp down
    velocity -= A_max;
    if(velocity <= 0.0) velocity = 0.0;
  }
  update_motor(velocity);
}

void WheelClass::trapezoid_pid(void){
    /* Motion control method using feed back from the motor encoder to determine */
    // Calculate the target velocity
    if(distance < D_max){
        // Ramp up speed to maximum
        velocity += A_max;
        if(velocity > V_max) velocity = V_max;
    } else if(distance >= D_max && distance < distance_target - D_max) {
        // Constant speed
    } else {
        // Ramp down
        velocity -= A_max;
        if(velocity <= 0.0) velocity = 0.0;
    }

    // Calculate velocity error
    float velocity_actual = distance - distance_last;
    float velocity_error = velocity - velocity_actual;

    // TODO: This will oscillate around the set point!
    if(velocity_error < 0) pwm += 10;
    else pwm -= 10;

    printf("PID: V: %f;  Va: %f; Verr: %f; Vc: %f; Dl: %i; D: %i; P:%i; Poff: %i; Dmax: %f\n", velocity, velocity_actual, velocity_error, velocity_corrected, int(distance_last), int(distance), (int(velocity_corrected*PWM_convert)+PWM_offset), PWM_offset, D_max);
    update_motor();
    distance_last = distance;
}

// Cumulative moving average of velocity
float WheelClass::velocity_average(float new_velocity){
  float average = (new_velocity + frames*velocity_av)/(frames + 1);
  velocity_av = average;
  frames++;
  return(average);
}

void WheelClass::triangle(void){
//  update_motor();
}

void WheelClass::update_motor(void){
  printf("PWM: %i\n", pwm);
  if(pwm > 1022) pwm = PWM_max;
  if(pwm < 0) pwm = 0;
  
  if (direction > 0){
    pwm_set_chan_level(pwm_slice, PWM_CHAN_A, pwm);
    pwm_set_chan_level(pwm_slice, PWM_CHAN_B, 0);
  } else {
    pwm_set_chan_level(pwm_slice, PWM_CHAN_A, 0);
    pwm_set_chan_level(pwm_slice, PWM_CHAN_B, pwm);
  }
}
