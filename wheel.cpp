#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "wheel.h"

WheelClass::WheelClass(void){
<<<<<<< HEAD
  distance = 0;
  distance_target = 0;
  distance_calc = 0;
=======
  distance = 0.0;
  distance_target = 0.0;
  distance_actual = 0.0;
  distance_error = 0.0;
  distance_co = 0.001;
>>>>>>> 33814dd670e87af19c84e76b389c7e18eb8775fe
  direction = 1;
  velocity = 0.0;
  pwm = 0;
  encoderA_last = 0;
  encoderB_last = 0;
  outputA_pin = 0;
  outputB_pin = 0;
  pwm_slice = 0;
}

void WheelClass::move(float distance_new, float scale){
  distance_target = distance_new;
  distance_calc = distance; // Start out with the calculated distance equal to the currently measured distance
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
  velocity_corrected = 0.0;
  update_motor();
}

void WheelClass::reset(void){
  PWM_max = 1022;
  PWM_offset = 650;
  
  V_max = 250.0 / 50; // 34
  A_max = V_max / 150.0; // 11.333
  D_max = V_max * 150.0 * 0.5; // 51.0

  PWM_convert = float(PWM_max - PWM_offset) / V_max;

  velocity = 0.0;
  distance = 0.0;
  distance_target = 0.0;
  distance_actual = 0.0;
  distance_error = 0.0;
  distance_co = 0.001;

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
  
  distance_last = distance; // Store the distance measured on the previous servo tick for the PID loop
  
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
<<<<<<< HEAD
  distance_calc += velocity; // This is how far we should have moved by now
  calculate_pwm(); // Calculate the PWM usng a PID controller
=======
  distance_error = distance_actual - distance;
  velocity = (distance_error * distance_co); // Very simple proportional control algorithm
>>>>>>> 33814dd670e87af19c84e76b389c7e18eb8775fe
  update_motor();
}

void WheelClass::triangle(void){
  update_motor();
}

<<<<<<< HEAD
void WheelClass::update_motor(void){
  // pwm = int(velocity * PWM_convert) + PWM_offset; // PWM value is now calculated by the PID controller
=======
void WheelClass::update_motor(){
  pwm = int(velocity * PWM_convert) + PWM_offset;
>>>>>>> 33814dd670e87af19c84e76b389c7e18eb8775fe
  if(velocity <= 0.0) pwm = 0;
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

// TODO: Figure out what the two different Proportion on... options mean, and pick one
// TODO: Compare code against other PID code, PID theory, ensure I understand!
void WheelClass::calculate_pwm(void)
{  
  /*Compute all the working error variables*/

  // distance measured
  // distance_calc calculated

  // double input = *myInput;
  double error = distance_calc - distance; //*mySetpoint - input;
  double dInput = distance - distance_last; //(input - lastInput);
  outputSum += (ki * error);

  /*Add Proportional on Measurement, if P_ON_M is specified*/
  if(!pOnE) outputSum -= kp * dInput;

  if(outputSum > 1.0) outputSum = 1.0;
  else if(outputSum < 0.0) outputSum = 0.0;

  /*Add Proportional on Error, if P_ON_E is specified*/
  double output;
  if(pOnE) output = kp * error;
  else output = 0;

  /*Compute Rest of PID Output*/
  output += outputSum - kd * dInput;

  if(output > 1.0) output = 1.0;
  else if(output < 0.0) output = 0.0;
  pwm = output;

  /*Remember some variables for next time*/
  lastInput = input;
  lastTime = now;
}
