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

extern bool Echo;

unsigned long milliseconds = 0;
unsigned long frame_count = 0;

unsigned long seconds = 0;

// Array of Wheel classes
WheelClass wheel[WHEEL_COUNT];

CommandPacket* currentCommand = NULL;

// Send the current status via serial link
void send_status(void){
  // Format: F<frame_count>;M1,T<>,D<>,V<>,P<>;
  
  printf("F%u;", frame_count);

  for(int i = 0; i < WHEEL_COUNT; i++){
    printf("M%i,", i); 
//    printf("T%f,", wheel[i].distance_target); 
    printf("D%f,", wheel[i].distance); 
//    printf("V%f,", wheel[i].velocity);
    printf("P%i;  ", wheel[i].pwm);
  }

  printf("\n");
}

void stop_all(void){
  for(int i = 0; i < WHEEL_COUNT; i++){
	  wheel[i].stop();
  }
}

// Interrupt handler for the encoder inputs
void encoder_handler(uint gpio, uint32_t events){
//  for (int i = 0; i < WHEEL_COUNT; i++) {
    // Exit the loop if the wheel claims the tick as it's own
//    if(wheel[i].encoder_tick(gpio)) break;
//  }
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
  wheel[0].set_pins(M1A, M1B, E1A, E1B);
  wheel[0].reset();
  
  wheel[1].set_pins(M2A, M2B, E2A, E2B);
  wheel[1].reset();
  
  wheel[2].set_pins(M3A, M3B, E3A, E3B);
  wheel[2].reset();

  wheel[3].set_pins(M4A, M4B, E4A, E4B);
  wheel[3].reset();
  
  wheel[4].set_pins(M5A, M5B, E5A, E5B);
  wheel[4].reset();
  
  wheel[5].set_pins(M6A, M6B, E6A, E6B);
  wheel[5].reset();

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

  // Command despatcher. 
  if(!in_motion) {
    currentCommand = command_next();
    
    switch(currentCommand->command){

      case(MOVE):
        for(int i = 0; i < WHEEL_COUNT; i++)
          wheel[i].move(currentCommand->motor[i].distance, currentCommand->motor[i].velocity);
        break;
      
      case(STOP):
        for(int i = 0; i < WHEEL_COUNT; i++)
          wheel[i].stop();
        break;
    }
  }
  
  // 20ms / 50Hz servo frame, so wait whatever time we have left since we started this loop
  // Use this while loop for handling everything that needs to process more quickly than the servo loop
  while((to_ms_since_boot(get_absolute_time()) - frame_start) < SERVO_FRAME){

    // Poll for encoder pulses
    for(int i = 0; i < WHEEL_COUNT; i++){
      wheel[i].encoder_tick();
    }

    if(packet_read()){
      if(packet_parse() == HARDSTOP){
        // HARDSTOP! Command queue will have already been dumped, so kill all motors
        printf("HARDSTOP!!!\n");
        for(int i = 0; i < WHEEL_COUNT; i++)
          wheel[i].stop();
      };
      // Interactive mode, so display prompt
      if(Echo) printf("Command > ");
    }
  };
/*
  // Simulate movement. Should be driven by encoder output
  // for(int i = 0; i < WHEEL_COUNT; i++){
  //   wheel[i].update_distance(wheel[i].velocity);
  // }
*/
  // Any thing the runs at base 50Hz should go here
  if(frame_count < 50) {
    gpio_put(LED_PIN, 1);
  }
  
  if(100 > frame_count > 50) {
    gpio_put(LED_PIN, 0);
  }

  if(frame_count > 100) {
    seconds++;
    send_status();
    frame_count = 0;
  }

  ++frame_count;
}

int main(void){

  // Configure everything
  setup();

  // wheel[0].move(100000.0, 1.0);
  // wheel[1].move(100000.0, 1.0);
  // wheel[2].move(100000.0, 1.0);
  // wheel[3].move(100000.0, 1.0);
  // wheel[4].move(100000.0, 1.0);
  // wheel[5].move(100000.0, 1.0);
  
  // Infinite loop
  while(true){
    loop();
  }

}
