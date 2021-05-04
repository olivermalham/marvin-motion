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

#define WHEEL_COUNT 6

// Milliseconds per servo frame
#define SERVO_FRAME 20

unsigned long milliseconds = 0;
unsigned long frame_count = 0;

// Array of Wheel classes
WheelClass wheel[6];

// Interrupt handler for the encoder inputs
void encoder_handler(uint gpio, uint32_t events){
  for (int i = 0; i < WHEEL_COUNT; i++) {
    // Exit the loop if the wheel claims the tick as it's own
    //if(wheel[i].encoder_tick(gpio)) break;
  }
}

void setup() {
  stdio_init_all();
  printf("Marvin motion controller v0.1\n");

  // Configure on board LED
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  // Initialise all the motor classes
  wheel[0].set_pins(M1A, M1B, E1A, E1B, (void *)&encoder_handler);
  wheel[0].reset();
//  wheel[0].move(1000.0, 1.0);
  
  wheel[1].set_pins(M2A, M2B, E2A, E2B, (void *)&encoder_handler);
  wheel[1].reset();
//  wheel[1].move(1000.0, 1.0);
  
  wheel[2].set_pins(M3A, M3B, E3A, E3B, (void *)&encoder_handler);
  wheel[2].reset();
//  wheel[2].move(1000.0, 1.0);

  wheel[3].set_pins(M4A, M4B, E4A, E4B, (void *)&encoder_handler);
  wheel[3].reset();
//  wheel[3].move(1000.0, 1.0);
  
  wheel[4].set_pins(M5A, M5B, E5A, E5B, (void *)&encoder_handler);
  wheel[4].reset();
//  wheel[4].move(1000.0, 1.0);
  
  wheel[5].set_pins(M6A, M6B, E6A, E6B, (void *)&encoder_handler);
  wheel[5].reset();
//  wheel[5].move(1000.0, 1.0);

  milliseconds = to_ms_since_boot(get_absolute_time());
}

// Primary loop code, handles the servo loop.
void loop() {
  
  // Servo frame start - milliseconds from boot
  unsigned long frame_start = to_ms_since_boot(get_absolute_time());
  
  bool in_motion = false;
  bool status_request = false;
  
  // Update all wheel motion controllers
  for(int i = 0; i < WHEEL_COUNT; i++){
    wheel[i].servo_tick();
    if(wheel[i].velocity > 0.0) in_motion = true;
  }
    
  // 20ms / 50Hz servo frame, so wait whatever time we have left since we started this loop
  // Use this while loop for handling everything that needs to process more quickly than the servo loop
  while((to_ms_since_boot(get_absolute_time()) - frame_start) < SERVO_FRAME){

  };

  // Simulate movement. Should be driven by encoder output
  // for(int i = 0; i < WHEEL_COUNT; i++){
  //   wheel[i].update_distance(wheel[i].velocity);
  // }

  // Any thing the runs at base 50Hz should go here
  if(frame_count < 50) {
    gpio_put(LED_PIN, 1);
  }
  
  if(frame_count > 50) {
    gpio_put(LED_PIN, 0);
  } 
  
  if(frame_count > 100){
    frame_count = 0;
    send_status(frame_count, wheel, WHEEL_COUNT);
  }

  ++frame_count;
}

int main(void){

  // Configure everything
  setup();

  // Infinite loop
  while(true){
    loop();
  }

}