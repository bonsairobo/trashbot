#ifndef __DYNAMIXEL_SERIAL_BUS_H__
#define __DYNAMIXEL_SERIAL_BUS_H__

#include "dynamixel_bus.h"

typedef struct dynamixel_serial_bus_impl dynamixel_serial_bus_impl_t;
struct dynamixel_serial_bus_impl
{
    int fd;         // file descriptor
    int baud;       // baud rate
};

dynamixel_msg_t * serial_bus_send_command(dynamixel_bus_t *bus,
                                          int id,
                                          int protocol,
                                          int instruction,
                                          dynamixel_msg_t *params,
                                          int retry);

dynamixel_bus_t * serial_bus_create(const char *device, int baud);
void serial_bus_destroy(dynamixel_bus_t *bus);
uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

#endif
