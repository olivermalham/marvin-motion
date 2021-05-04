#include <stdio.h>
#include <ctype.h>
#include "pico/stdlib.h"

#include "wheel.h"
#include "command_packet.h"

#define PACKET_BUFFER 1000
char packetBuffer[PACKET_BUFFER];
unsigned int packetBufferEnd = 0;


// Read a new packet from stdin, one char at a time. Return the length 
// of the received packet once we hit a newline. Doesn't do any kind of 
// format checking atm.
// Format: F<frame_count>;M1,T<>,D<>,V<>,P<>;\n
unsigned int packet_read(void){
  
  char c;
  c = getchar_timeout_us(0);

  // Only process the character if we didn't timeout
  if(c != PICO_ERROR_TIMEOUT){

    // If we have hit the end of the buffer, assume an error and
    // just reset the buffer
    if(packetBufferEnd == PACKET_BUFFER) packetBufferEnd = 0;

    // End of packet
    if(c == '\n'){
      packetBuffer[packetBufferEnd] = 0; // null terminate
      return packetBufferEnd;
    }

    // Only accept valid characters, anything else is ignored
    if(isalnum(c) || c == ';' || c == ','){
      packetBuffer[packetBufferEnd] = c;
      packetBufferEnd++;
      return 0;
    }

  }
  return 0;
}

void send_status(unsigned long frame_count, WheelClass* wheels, int count){
  // Format: F<frame_count>;M1,T<>,D<>,V<>,P<>;
  
  printf("F%u;", frame_count);

  for(int i = 0; i < count; i++){
    printf("M%i,", i); 
    printf("T%i,", wheels[i].distance_target); 
    printf("D%i,", wheels[i].distance); 
    printf("V%i,", wheels[i].velocity);
    printf("P%i;", wheels[i].pwm);
  }

  printf("n");
}

