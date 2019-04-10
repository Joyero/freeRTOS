#include "spi.h"

#define MEM_WRITE_DISABLE_CMD	0x04
#define MEM_READ_STATUS_REG1	0x05
#define MEM_WRITE_ENABLE_CMD	0x06

#define MEM_MANUF_DEVICE_ID_ADD	0x90;


void mem_init(void);
