
enum Commands {
    MOVE,
    STOP,
    TEST,
    CALIB,
    STATUS,
    HARDSTOP,
    BUFFER_OVERFLOW,
    NO_COMMAND,
    BAD_COMMAND,
    PRINT, // Print the command buffer, for human info
    NO_ECHO // Stop echoing characters back to host
};

struct CommandMotor {
    float distance;
    float velocity;
};

struct CommandPacket {
    enum Commands command;
    CommandMotor motor[6];
};

unsigned int packet_read(void);
int packet_parse();
void packet_buffer_clear();

void command_advance(void);
CommandPacket* command_next(void);
void command_buffer_flush(void);
void command_buffer_print(void);
void command_clear(CommandPacket* packet);
void command_init(void);
