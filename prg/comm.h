#include <inttypes.h>

#define COMMAND_PRG_STARTED 0
#define COMMAND_CHR_STARTED 1
#define COMMAND_ERROR_INVALID 2
#define COMMAND_ERROR_CRC 3
#define COMMAND_ERROR_OVERFLOW 4
#define COMMAND_PRG_INIT 5
#define COMMAND_CHR_INIT 6
#define COMMAND_PRG_READ_REQUEST 7
#define COMMAND_PRG_READ_RESULT 8
#define COMMAND_PRG_WRITE_REQUEST 9
#define COMMAND_PRG_WRITE_DONE 10
#define COMMAND_CHR_READ_REQUEST 11
#define COMMAND_CHR_READ_RESULT 12
#define COMMAND_CHR_WRITE_REQUEST 13
#define COMMAND_CHR_WRITE_DONE 14
#define COMMAND_PHI2_INIT 15
#define COMMAND_PHI2_INIT_DONE 16
#define COMMAND_MIRRORING_REQUEST 17
#define COMMAND_MIRRORING_RESULT 18
#define COMMAND_RESET 19
#define COMMAND_RESET_ACK 20
#define COMMAND_PRG_EPROM_WRITE_REQUEST 21
#define COMMAND_CHR_EPROM_WRITE_REQUEST 22
#define COMMAND_EPROM_PREPARE 23
#define COMMAND_PRG_FLASH_ERASE_REQUEST 24
#define COMMAND_PRG_FLASH_WRITE_REQUEST 25
#define COMMAND_CHR_FLASH_ERASE_REQUEST 26
#define COMMAND_CHR_FLASH_WRITE_REQUEST 27

void comm_init();
void comm_start(uint8_t command, unsigned int length);
void comm_send_byte(uint8_t data);
void comm_proceed(uint8_t data);

extern volatile uint8_t comm_recv_command;
extern volatile unsigned int comm_recv_length;
extern volatile uint8_t recv_buffer[RECV_BUFFER];
extern volatile uint8_t comm_recv_done;
