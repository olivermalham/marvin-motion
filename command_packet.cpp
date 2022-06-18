#include <stdio.h>
#include <cstring>
#include <ctype.h>
#include "pico/stdlib.h"

#include "wheel.h"
#include "command_packet.h"

// If false, don't echo input back to host.
// NO_ECHO command sets this to false, for non-human / non-interactive use. 
bool Echo = true;

// Packet buffer to handle no-blocking command input
#define PACKET_BUFFER 1000
char packetBuffer[PACKET_BUFFER];
unsigned int packetBufferEnd = 0;

// CommandBuffer is implemented as a circular FIFO buffer
#define COMMAND_BUFFER_LENGTH 128
unsigned int CommandBufferHead = 0;
unsigned int CommandBufferTail = 0; // Always points to the next empty command struct
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
      TEST:2;
  */  
  char c;
  c = getchar_timeout_us(0);

  // Only process the character if we didn't timeout
  if(c != PICO_ERROR_TIMEOUT){
    
    // If we have hit the end of the buffer, assume an error and
    // just reset the buffer
    if(packetBufferEnd == PACKET_BUFFER) packetBufferEnd = 0;

    // End of packet
    if(c == ';' || c == 10 || c == 13){
      if(Echo) printf("%c\n", c);
      packetBuffer[packetBufferEnd] = 0; // null terminate
      return packetBufferEnd;
    }

    // Only accept valid characters, anything else is ignored.
    // Convert to uppercase to make interactive use easier for the user
    if(isalnum(c) || c == ',' || c == '.' || c == ':'){
      if(Echo) printf("%c", c);
      packetBuffer[packetBufferEnd] = toupper(c);
      packetBufferEnd++;
      return 0;
    }

    // Handle backspace, for interactive use
    if(c == 0x08 ){
      if(Echo) printf("%c %c", c, c);
      if(packetBufferEnd) packetBufferEnd--;
    }

  }
  return 0;
}


int packet_parse(){
  /* Parse a string into a CommandPacket struct and add it to the command
  circular buffer.

  Returns the Command enum of the packet parsed (primary use is to signal a hard stop).

  If a HARDSTOP command has been received, it over-rides all other commands and 
  flushes the circular buffer.

  Format: <Command>:M<motor no>,D<distance>,V<speed>; 
  Examples:
      MOVE:M1,D1234.5,V128;
      STOP;
      STATUS;
      HARDSTOP;
  */
  CommandPacket* newPacket;
  int failure = 0;
  
  printf("packet_parse: packetBuffer: %s\n", packetBuffer);

  // Check first if we have a HARDSTOP as that over-rides everything
  // Flushes the command buffer.
  failure = strncmp(packetBuffer, "HARDSTOP", 8);
  if(!failure){
    packet_buffer_clear();
    command_buffer_flush();
    return HARDSTOP;
  }

  // Next, check for buffer overflow (including buffer wrap case).
  // Drop the packet on overflow.
  // FIXME?
  if((CommandBufferTail == (CommandBufferHead - 1)) ||
     (CommandBufferTail == (COMMAND_BUFFER_LENGTH - 1) && CommandBufferHead == 0)){
    packet_buffer_clear();
    printf("parse_packet: BUFFER OVERFLOW\n");
    return BUFFER_OVERFLOW;
  }

  // STATUS Command - immediate response, not queued
  failure = strncmp(packetBuffer, "STATUS", 6);
  if(!failure){
    printf("packet_parse: STATUS\n");
    packet_buffer_clear();
    return STATUS;
  }

  // PRINT Command - immediate response, not queued
  failure = strncmp(packetBuffer, "PRINT", 5);
  if(!failure){
    command_buffer_print();
    packet_buffer_clear();
    return PRINT;
  }

  // NOECHO Command - immediate response, not queued
  failure = strncmp(packetBuffer, "NOECHO", 6);
  if(!failure){
    Echo = false;
    packet_buffer_clear();
    return NO_ECHO;
  }
  
  // Now handle queued commands
  newPacket = &CommandBuffer[CommandBufferTail];
  command_clear(newPacket);
  
  // TEST Command - set the specified motor to 100%
  failure = strncmp(packetBuffer, "TEST", 4);
  if(!failure){
    int motor_no = 0;
    int args = sscanf(packetBuffer, "TEST:%i", motor_no);
    
    newPacket->motor[motor_no].distance = 10000;
    newPacket->motor[motor_no].velocity = 1.0;

    if(args != 1){
      printf("packet_parse: BAD_COMMAND\n");
      packet_buffer_clear();
      return BAD_COMMAND;
    } else {
      command_advance();
      return TEST;
    }
  }

  // CALIB Command - Ramp up the specified motor PWM, monitoring encoder output
  // to determine the PWM offset and parameters for mapping PWM to speed
  failure = strncmp(packetBuffer, "CALIB", 5);
  if(!failure){
    int motor_no = 0;
    int args = sscanf(packetBuffer, "CALIB:%i", motor_no);
    
    // newPacket->motor[motor_no].distance = 10000;
    // newPacket->motor[motor_no].velocity = 1.0;

    if(args != 1){
      printf("packet_parse: BAD_COMMAND\n");
      packet_buffer_clear();
      return BAD_COMMAND;
    } else {
      command_advance();
      return CALIB;
    }
  }


  // STOP Command
  failure = strncmp(packetBuffer, "STOP", 4);
  if(!failure){
    newPacket->command = STOP;
    printf("packet_parse: STOP\n");
    command_advance();
    return STOP;
  }

  // MOVE Command
  // Most complicated as it takes arguments. Command *must* cover all 6 motors
  // Format:
  // MOVE:D<motor 1 distance>,V<motor 1 velocity>....D<motor 6 distance>,V<motor 6 velocity>;
  // e.g.
  // MOVE:D0.0,V0.0,D1.0,V1.0,D2.0,V2.0,D3.0,V3.0,D4.0,V4.0,D5.0,V5.0
  // MOVE:D10000.0,V1.0,D10000.0,V1.0,D10000.0,V1.0,D10000.0,V1.0,D10000.0,V1.0,D10000.0,V1.0
  //
  // If the command is malformed, return BAD_COMMAND without incrementing the buffer tail,
  // so the command will be over-written and ignored.
  failure = strncmp(packetBuffer, "MOVE", 4);
  if(!failure){
    newPacket->command = MOVE;
    printf("packet_parse: MOVE\n");
    
    int args = sscanf(packetBuffer, "MOVE:D%f,V%f,D%f,V%f,D%f,V%f,D%f,V%f,D%f,V%f,D%f,V%f",
            &newPacket->motor[0].distance, &newPacket->motor[0].velocity,
            &newPacket->motor[1].distance, &newPacket->motor[1].velocity,
            &newPacket->motor[2].distance, &newPacket->motor[2].velocity,
            &newPacket->motor[3].distance, &newPacket->motor[3].velocity,
            &newPacket->motor[4].distance, &newPacket->motor[4].velocity,
            &newPacket->motor[5].distance, &newPacket->motor[5].velocity);
    
    if(args != 12){
      printf("packet_parse: BAD_COMMAND\n");
      packet_buffer_clear();
      return BAD_COMMAND;
    } else {
        printf("Packet received %u\n", (unsigned int)newPacket);
        for(int i = 0; i < 6; i++){
            printf("Wheel %i: D%f V%f\n", i,
            newPacket->motor[i].distance,
            newPacket->motor[i].velocity);
        };
      command_advance();
      return MOVE;
    }
  }

  // Handle unknown / borken commands
  printf("packet_parse: BAD_COMMAND\n");
  packet_buffer_clear();
  return BAD_COMMAND;
}


void packet_buffer_clear(){
  // Just reset the buffer index to zero
  packetBufferEnd = 0;
}

// FIXME? Doesn't handle tail catching head index
void command_advance(void){
  // Advance the command buffer to the next free slot.
  CommandBufferTail++;
  if(CommandBufferTail >= COMMAND_BUFFER_LENGTH) CommandBufferTail = 0;
  packet_buffer_clear();
}

// FIXME? IS THIS WRONG?
CommandPacket* command_next(void){
  /* Return a pointer to the next command packet in the circular buffer.
  Returns NULL if there are no more commands in the buffer.
  */
  if(CommandBufferHead == CommandBufferTail) return NULL;
  CommandBufferHead++;
  if(CommandBufferHead > COMMAND_BUFFER_LENGTH) CommandBufferHead = 0;

  return &CommandBuffer[CommandBufferHead];
}


void command_buffer_flush(void){
  /* Reset the packet buffer (just sets the head and tail indices to zero) */
  CommandBufferHead = 0;
  CommandBufferTail = 0;
}


void command_buffer_print(void){
  /* Print the entire circular buffer to stdout, for debug purposes */
  unsigned int i = CommandBufferHead;

  if(CommandBufferHead == CommandBufferTail){
    //printf("\nCommand Buffer Empty\n");
    return;
  }

  printf("\nCommand Buffer:\n");
  printf("Head: %i; Tail: %i\n", CommandBufferHead, CommandBufferTail);
  while(i <= CommandBufferTail){
//
//    if(!CommandBuffer[i].command) {
//        i++; continue;
//    }

    printf("\tCommand %i:\n", i);
    switch(CommandBuffer[i].command){
      case STOP:
        printf("\t\tSTOP\n");
        break;

      case MOVE:
        printf("\t\tMOVE\n");
        for(int j = 0; j < 6; j++){
          printf("\t\t\tMotor %i: Distance = %f, Velocity = %f\n", j, 
                 CommandBuffer[i].motor[j].distance, 
                 CommandBuffer[i].motor[j].velocity);
        }
        break;
    }

    // Handle stepping over the circular buffer
    i++;
    if(i > COMMAND_BUFFER_LENGTH) i = 0;
    if(i == CommandBufferTail) break;
  }
}


void command_clear(CommandPacket* packet){
  // Reset the command packet at the specified index
  packet->command = NO_COMMAND;

  for(int i = 0; i < 6; i++){
    packet->motor[i].distance = 0.0;
    packet->motor[i].velocity = 0.0;
  }
}

void command_init(void){
  // Initialise the command buffer
  for(int i = 0; i < COMMAND_BUFFER_LENGTH; i++){
    command_clear(&CommandBuffer[i]);
  }
}
