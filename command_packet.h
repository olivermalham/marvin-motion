#include "wheel.h"

unsigned int packet_read(void);
void send_status(unsigned long frame_count, WheelClass* wheels, int count);
