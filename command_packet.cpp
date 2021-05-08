#include <stdio.h>
#include <ctype.h>
#include "pico/stdlib.h"

#include "wheel.h"
#include "command_packet.h"

#define PACKET_BUFFER 1000
char packetBuffer[PACKET_BUFFER];
unsigned int packetBufferEnd = 0;

#define COMMAND_BUFFER_LENGTH 128
unsigned int CommandBufferHead = 0;
unsigned int CommandBufferTail = 0;
CommandPacket CommandBuffer[COMMAND_BUFFER_LENGTH];


unsigned int packet_read(void){
  /* Read a new packet from stdin, one char at a time. Return the length 
  of the received packet once we hit a newline. Doesn't do any kind of 
  format checking atm.
  Format: <Command>:M<motor no>,D<distance>,V<speed>; 
  Examples:
      MOVE:M1,D1234.5,V128;
      STOP;
      STATUS;
      HARDSTOP;
  */  
  char c;
  c = getchar_timeout_us(0);

  // Only process the character if we didn't timeout
  if(c != PICO_ERROR_TIMEOUT){

    // If we have hit the end of the buffer, assume an error and
    // just reset the buffer
    if(packetBufferEnd == PACKET_BUFFER) packetBufferEnd = 0;

    // End of packet
    if(c == ';'){
      packetBuffer[packetBufferEnd] = 0; // null terminate
      printf(";\n");
      return packetBufferEnd;
    }

    // Only accept valid characters, anything else is ignored
    if(isalnum(c) || c == ',' || c == '.'){
      packetBuffer[packetBufferEnd] = c;
      packetBufferEnd++;
      printf("%c", c);
      return 0;
    }

  }
  return 0;
}


void packet_parse(unsigned char* packet_string){
  /* Parse a string into a CommandPacket struct and add it to the command
  circular buffer. If a HARDSTOP command has been received, it over-rides all other commands
  and flushes the circular buffer.
  */
  CommandBufferTail++;
  if(CommandBufferTail > COMMAND_BUFFER_LENGTH) CommandBufferTail = 0;


}


CommandPacket* packet_next(void){
  /* Return a pointer to the next command packet in the circular buffer.
  Returns NULL if there are no more commands in the buffer.
  */
  CommandPacket *result = &CommandBuffer[CommandBufferHead];
  CommandBufferHead++;
  if(CommandBufferHead > COMMAND_BUFFER_LENGTH) CommandBufferHead = 0;
  if(CommandBufferHead == CommandBufferTail) result = NULL;
  return result;
}


void packet_buffer_flush(void){
  /* Reset the packet buffer (just sets the head and tail indices to zero) */
  CommandBufferHead = 0;
  CommandBufferTail = 0;
}

void packet_buffer_print(void){
  /* Print the entire circular buffer to stdout, for debug purposes */
  unsigned int i = CommandBufferHead;

  while(i != CommandBufferTail){
    // print

    // Handle stepping over the circular buffer
    i++;
    if(i > COMMAND_BUFFER_LENGTH) i = 0;
    if(i == CommandBufferTail) break;
  }
}