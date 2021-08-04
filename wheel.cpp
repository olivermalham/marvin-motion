#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "wheel.h"

WheelClass::WheelClass(void){
  distance = 0;
  distance_target = 0;
}

void WheelClass::move(float distance, float scale){
  distance_target = distance;
  D_max = D_max * scale;
  V_max = V_max * scale;
  A_max = A_max * scale;

  // If total distance is less than twice the distance required to ramp up to the maximum velocity, then 
  // we have a triangluar motion profile. Recalculate accordingly.
  if(D_max > distance / 2){};
  
}

// Perform an immediate stop of the motor
void WheelClass::stop(void){
  velocity = 0.0;
  update_motor();
}

void WheelClass::reset(void){
  PWM_max = 1024;
  PWM_offset = 80;
  
  V_max = 1700.0 / 50; // 34
  A_max = V_max / 150.0; // 11.333
  D_max = V_max * 150.0 * 0.5; // 51.0

  PWM_convert = float(PWM_max - PWM_offset) / V_max;

  velocity = 0.0;
  distance = 0.0;
  pwm = 0;
}

void WheelClass::set_pins(int outA, int outB, int encoderA, int encoderB, void* handler){
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

  // Configure encoder IRQ handling
  gpio_set_irq_enabled_with_callback(encoderA, GPIO_IRQ_EDGE_RISE, true, (gpio_irq_callback_t) handler);
  gpio_init(encoderB);
  gpio_set_dir(encoderB, GPIO_IN);
}

// Called by the interrupt handler for the encoders to update the distance count
bool WheelClass::encoder_tick(uint gpio){
  if(gpio == encoderA_pin){
    if(gpio_get(encoderB_pin)){
      distance += 1;
    } else {
      distance -= 1;
    }
    return true;
  }
  return false;
};

void WheelClass::update_distance(float delta){
  distance += delta;
}

int WheelClass::servo_tick(void){
  trapezoid();
//  if(distance_target > 2*D_max) return trapezoid();
//  else return triangle();
  return 0;
}

void WheelClass::trapezoid(void){
  if(distance < D_max){
    // Ramp up speed to maximum
    velocity += A_max;
    //if(velocity > V_max) velocity = V_max;
  } else if(distance >= D_max && distance < distance_target - D_max) {
    // Constance velocity
  } else {
    // Ramp down
    velocity -= A_max;
    if(velocity <= 0) velocity = 0;
  }

  update_motor();
}

void WheelClass::triangle(void){
  update_motor();
}

void WheelClass::update_motor(void){
  pwm = int(velocity * PWM_convert) + PWM_offset;
  if(velocity <= 0.0) pwm = 0;
  if(pwm > 1023) pwm = 1023;
  
  if (direction > 0){
    pwm_set_chan_level(pwm_slice, PWM_CHAN_A, pwm);
    pwm_set_chan_level(pwm_slice, PWM_CHAN_B, 0);
  } else {
    pwm_set_chan_level(pwm_slice, PWM_CHAN_A, 0);
    pwm_set_chan_level(pwm_slice, PWM_CHAN_B, pwm);
  }
}
