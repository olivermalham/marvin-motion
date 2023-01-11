#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "wheel.h"
#include "command_packet.h"

#include "marvin_pins.h"

// Serial comms speed
#define SERIAL_SPEED 115200

// Milliseconds per servo frame
#define SERVO_FRAME 20

extern bool Echo;

unsigned long milliseconds = 0;
unsigned long frame_count = 0;
unsigned long frame_total = 0;
unsigned long frame_start = 0;
unsigned long seconds = 0;

// Array of Wheel classes
WheelClass wheels[6];


// Send the current status via serial link
void send_status(void){
  // Format: F<frame_count>;M1,T<>,D<>,V<>,P<>;
  printf("F%u;  ", frame_total);

  for(int i = 0; i < 6; i++){
    printf("M%i, ", i+1); 
    printf("D:%f, ", wheels[i].distance);
    printf("V:%f, ", wheels[i].velocity);
    printf("P:%i  |  ", wheels[i].pwm);
  }
  printf("\n");
}


void stop_all(void){
  for(int i = 0; i < 6; i++){
	  wheels[i].stop();
  }
}


// Gradually ramp up the PWM on the specified motor until the wheel velocity is none-zero, print out the
// PWM value to get the offset. Once offset is found, jump up to full power and print out the velocity value.
// Purpose is to find the parameters required to map velocity to PWM. Values will be hardcoded once found.
void calibrate(unsigned int motor){
  unsigned int offset = 0;
  float max_V = 0.0;

}


void setup() {
  stdio_init_all();
  printf("=============================\n");
  printf("Marvin motion controller v0.1\n");
  printf("=============================\n\n");
  printf("Command > ");

  // Configure on board LED
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  command_init();

  // Initialise all the motor classes
  wheels[0].set_pins(M1A, M1B, E1A, E1B);
  wheels[0].reset();
  
  wheels[1].set_pins(M2A, M2B, E2A, E2B);
  wheels[1].reset();
  
  wheels[2].set_pins(M3A, M3B, E3A, E3B);
  wheels[2].reset();

  wheels[3].set_pins(M4A, M4B, E4A, E4B);
  wheels[3].reset();
  
  wheels[4].set_pins(M5A, M5B, E5A, E5B);
  wheels[4].reset();
  
  wheels[5].set_pins(M6A, M6B, E6A, E6B);
  wheels[5].reset();

  milliseconds = to_ms_since_boot(get_absolute_time());
}


// Polling loop, runs until the end of the servo frame
void poll(WheelClass* wheels){
    while((to_ms_since_boot(get_absolute_time()) - frame_start) < SERVO_FRAME){
        // Poll for encoder pulses
        for(int i = 0; i < 6; i++){
          wheels[i].encoder_tick();
        }

        if(packet_read()){
          if(packet_parse(wheels) == HARDSTOP){
            // HARDSTOP! Command queue will have already been dumped, so kill all motors
            printf("HARDSTOP!!!\n");
            for(int i = 0; i < 6; i++)
              wheels[i].stop();
          };
          // Interactive mode, so display prompt
          if(Echo) printf("Command > ");
        }
    };
}


// Update all wheels motion controllers
bool update_wheels(unsigned long frame_time) {
  bool in_motion = false;

  for(int i = 0; i < 6; i++){
    wheels[i].servo_tick(frame_time);
    if(wheels[i].velocity > 0.0) {
        in_motion = true;
    }
  }
  return in_motion;
}


// Anything that needs to runs once per frame should go here
void per_frame(void){
  if(frame_count < 50) {
    gpio_put(LED_PIN, 1);
  }

  if(frame_count > 50) {
    gpio_put(LED_PIN, 0);
  }

  if(frame_count >= 100) {
    seconds++;
    frame_count = 0;

    for(int i = 0; i < 6; i++){
        printf("Wheel %i: D:%f V:%f Vmeasured:%f Vpid:%f PWM:%i\n", i,
                                wheels[i].distance,
                                wheels[i].velocity,
                                wheels[i].velocity_actual,
                                wheels[i].velocity + wheels[i].velocity_corrected,
                                wheels[i].pwm);
    };
    printf("\n");
  }

  ++frame_count;
  ++frame_total;
}


int main(void){
  // Configure everything
  setup();
  send_status();

  // Infinite loop
  while(true){
    frame_start = to_ms_since_boot(get_absolute_time());

    if(!update_wheels(frame_start)) execute_next_command(wheels);

    per_frame();
    poll(wheels);
  }
}
