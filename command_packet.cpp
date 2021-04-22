#include <stdio.h>
#include "pico/stdlib.h"

#include "command_packet.h"


// Format: F<frame_count>;M1,T<>,D<>,V<>,P<>;

void buffered_read(void){
  static char buffer[1000];
  static int bufferEnd = 0;
  char c;

  c = getchar_timeout_us(0);

}

void send_status(WheelClass* wheels, int count){
  // Format: F<frame_count>;M1,T<>,D<>,V<>,P<>;
  
  printf("F%u;", frame_count);

  for(int i = 0; i < count; i++){
    printf("M%i,", i); 
    printf("T%i,", wheel[i].distance_target); 
    printf("D%i,", wheel[i].distance); 
    printf("V%i,", wheel[i].velocity);
    printf("P%i;", wheel[i].pwm);
  }

  printf("n");
}

void packet_ready(void){
    
}