
enum Commands {
    MOVE,
    STOP,
    STATUS,
    HARDSTOP
};

struct CommandMotor {
    float distance;
    float time;
};

struct CommandPacket {
    enum Commands command;
    CommandMotor motor[6];
};

unsigned int packet_read(void);
void packet_parse(unsigned char* packet_string);
CommandPacket* packet_next(void);
void packet_buffer_flush(void);
void packet_buffer_print(void);