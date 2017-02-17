#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "common/serial.h"
#include "common/ioutils.h"

#include "dynamixel_serial_bus.h"
#include "dynamixel_device.h"

#define TIMEOUT_MS 50
#define VERBOSE 0

#define MAKEWORD(x,y) ((uint16_t)(((uint8_t)(x)) | (((uint16_t)((uint8_t)(y)))<<8)))
#define LOBYTE(x) ((uint8_t) ((x) & 0xFF))
#define HIBYTE(x) ((uint8_t) ((x) >> 8 & 0xFF))

// === Bus specific implementation ===================
// Send an instruction with the specified parameters. The error code,
// body, and checksum of the response are returned (while the initial 4
// bytes of the header are removed)
static dynamixel_msg_t *
send_command_raw(dynamixel_bus_t *bus,
                 uint8_t id,
                 int instruction,
                 dynamixel_msg_t *params)
{
    dynamixel_serial_bus_impl_t *impl = (bus->impl);

    // Missing synchronization
    int parameterlen = (params == NULL) ? 0 : params->len;
    uint8_t *cmd = malloc((6+parameterlen)*sizeof(uint8_t));
    cmd[0] = 0xff;   // MAGIC
    cmd[1] = 0xff;   // MAGIC
    cmd[2] = id;    // servo id
    cmd[3] = (uint8_t)(parameterlen+2) & 0xff;  // Length
    cmd[4] = (uint8_t)(instruction & 0xff);

    if (params != NULL) {
        for (int i = 0; i < params->len; i++)
            cmd[5+i] = params->buf[i];
    }

    int checksum = 0;
    for (int i = 2; i < parameterlen+6-1; i++) {
        checksum += (cmd[i] & 0xff);
    }
    cmd[5+parameterlen] = (uint8_t)((checksum ^ 0xff) & 0xff);

    int res = write(impl->fd, cmd, 6+parameterlen);
    if (VERBOSE) {
        printf("WRITE_v1: = %d : ", 5+parameterlen);
        for (int i = 0; i < 5+parameterlen; i++){
            printf("%02x ", cmd[i] & 0xff);
        }
        printf("\n");
    }
    free(cmd);

    // Read response. The header is really 5 bytes, but we put the
    // error code in the body so that the caller knows what went wrong
    // if something bad happens. Synchronize on the first two 0xffff
    // characters.
    dynamixel_msg_t *header = dynamixel_msg_create(4);
    int header_have = 0;
    while (header_have < 4) {
        res = read_fully_timeout(impl->fd,
                                 (header->buf)+header_have,
                                 4 - header_have,
                                 TIMEOUT_MS);

        if (VERBOSE) {
            printf("READ_v1:  header = %d : ", res);
            dynamixel_msg_dump(header);
        }

        if (res < 1)
            return NULL;

        assert (res <= (4 - header_have));
        //assert (res + header_have == 4);

        // If the first two bytes are the sync bytes, we're done, check for 3dr byte of protocol v2
        if ((header->buf[0] & 0xff) == 0xff && (header->buf[1] & 0xff) == 0xff){
            if((header->buf[2] & 0xff) != 0xfd){
                break;
            }
        }
            

        // Shift buffer, read one more character
        header_have = 3;
        for (int i = 0; i < 3; i++)
            header->buf[i] = header->buf[i+1];
    }

    if ((header->buf[2] & 0xff) != id) {
        printf("serial_bus: Received response for wrong servo %d\n",
               header->buf[2] & 0xff);
        return NULL;
    }

    //int thisid = header->buf[2] & 0xff;
    int length = header->buf[3] & 0xff;

    if (length < 2)
        return NULL;

    dynamixel_msg_t *body = dynamixel_msg_create(length);
    res = read_fully_timeout(impl->fd,
                             body->buf,
                             body->len,
                             TIMEOUT_MS);

    if (VERBOSE) {
        printf("READ_v1:  body = %d : ", res);
        dynamixel_msg_dump(body);
    }

    if (1) {
        int checksum = 0;
        for (int i = 2; i < header->len; i++)
            checksum += (header->buf[i] & 0xff);
        for (int i = 0; i < body->len-1; i++)
            checksum += (body->buf[i] & 0xff);
        checksum = (checksum & 0xff) ^ 0xff;
        if ((body->buf[body->len - 1] & 0xff) != checksum) {
            printf("serial_bus: Bad checksum %02x %02x\n",
                   body->buf[body->len - 1] & 0xff,
                   checksum);
            return NULL;
        }
    }

    dynamixel_msg_destroy(header);
    return body;
}

// === Bus specific implementation ===================
// Send an instruction with the specified parameters in DXL2.0. The error code,
// body, and checksum of the response are returned (while the initial 6
// bytes of the header are removed)
static dynamixel_msg_t *
send_command_raw_2(dynamixel_bus_t *bus,
                 uint8_t id,
                 int instruction,
                 dynamixel_msg_t *params)
{
    dynamixel_serial_bus_impl_t *impl = (bus->impl);

    // Missing synchronization
    int parameterlen = (params == NULL) ? 0 : params->len;
    uint8_t *cmd = malloc((9+parameterlen)*sizeof(uint8_t));
    cmd[0] = 0xff;   // Header
    cmd[1] = 0xff;   // Header
    cmd[2] = 0xfd;   // Header
    cmd[3] = 0x00;   // Header
    cmd[4] = id;    // ID
    cmd[5] = LOBYTE(parameterlen+3);  // length low byte
    cmd[6] = HIBYTE(parameterlen+3);  // length hi byte
    cmd[7] = (uint8_t)(instruction & 0xff);
    if (params != NULL) {
        for (int i = 0; i < parameterlen; i++)
            cmd[8+i] = params->buf[i];
    }

    // add CRC16, replaces checksum in DXL 2.0 protocol
    uint16_t crc = updateCRC(0, cmd, 8+parameterlen);
    cmd[8+parameterlen] = LOBYTE(crc);
    cmd[9+parameterlen] = HIBYTE(crc);

    int res = write(impl->fd, cmd, 10+parameterlen);
     if (VERBOSE) {
        printf("WRITE_v2: = %d : ", 10+parameterlen);
        for (int i = 0; i < 10+parameterlen; i++){
            printf("%02x ", cmd[i] & 0xff);
        }
        printf("\n");
    }
    free(cmd);

    // Read response DXL2.0
    // grab 3 byte header, reserved byte, ID and 2 bytes of length + 0x55 status = 8 bytes total
    // Synchronize on the three magic bytes 0xFF 0xFF 0xFD.
    dynamixel_msg_t *header = dynamixel_msg_create(8);
    int header_have = 0;
    while (header_have < 8) {
        res = read_fully_timeout(impl->fd,
                                 (header->buf)+header_have,
                                 8 - header_have,
                                 TIMEOUT_MS);

        if (VERBOSE) {
            printf("READ_v2:  header = %d : ", res);
            dynamixel_msg_dump(header);
        }

        if (res < 1){
            dynamixel_msg_destroy(header);
            return NULL;
        }

        assert (res <= (8 - header_have));

        // If the first three bytes are the sync bytes, we're done
        if ((header->buf[0] & 0xff) == 0xff && 
            (header->buf[1] & 0xff) == 0xff &&
            (header->buf[2] & 0xff) == 0xfd)
            break;

        // Shift buffer, read one more character
        header_have = 7;
        for (int i = 0; i < 7; i++)
            header->buf[i] = header->buf[i+1];
    }

    if ((header->buf[4] & 0xff) != id) {
        printf("serial_bus: Received response for wrong servo %d\n",
               header->buf[4] & 0xff);
        dynamixel_msg_destroy(header);
        return NULL;
    }

    uint16_t length = MAKEWORD(header->buf[5],header->buf[6]);

    if (length < 2){
        dynamixel_msg_destroy(header);
        return NULL;
    }

    dynamixel_msg_t *body = dynamixel_msg_create(length-1); 
    res = read_fully_timeout(impl->fd,
                             body->buf,
                             body->len,
                             TIMEOUT_MS);

    if (VERBOSE) {
        printf("READ_v2:  body = %d : ", res);
        dynamixel_msg_dump(body);
    }


    //check crc
    crc = (uint16_t)MAKEWORD(body->buf[length-3],body->buf[length-2]);
    dynamixel_msg_t *rx_msg = dynamixel_msg_create(length + 5);
    for(int i = 0; i<8; i++){
        rx_msg->buf[i] = header->buf[i];
    }
    for(int i = 0; i<length-3; i++){
        rx_msg->buf[8+i] = body->buf[i];
    }

    if(VERBOSE)
        printf("CRC = %x : %x\n", crc, updateCRC(0, rx_msg->buf, length+5));
    if (updateCRC(0, rx_msg->buf, length+5) == crc){
        dynamixel_msg_destroy(header);
        dynamixel_msg_destroy(rx_msg);
        if(VERBOSE)
            printf("CRC GOOD\n");
        return body;

    }

    else {
        dynamixel_msg_destroy(header);
        dynamixel_msg_destroy(rx_msg);
        dynamixel_msg_destroy(body);
        printf("CRC ERR\n");
        return NULL;
    }

}

dynamixel_msg_t *
serial_bus_send_command(dynamixel_bus_t *bus,
                        int id,
                        int protocol,
                        int instruction,
                        dynamixel_msg_t *params,
                        int retry)
{
    do {
        

        dynamixel_msg_t *resp = NULL;
        if (protocol == 1){
            resp = send_command_raw(bus,
                                    id,
                                    instruction,
                                    params);
        }
        else if (protocol == 2){
            resp = send_command_raw_2(bus,
                                      id,
                                      instruction,
                                      params);
        }
        else {
            printf("PROTOCOL NOT SPECIFIED\n");
            return NULL;
        }

        if (resp == NULL || resp->len < 1) {
            if (VERBOSE) {
                printf("serial_bus id=%d error: short response.\n", id);
            }
            continue;
        }

        // Something went wrong!
        if (resp->buf[0] != 0) {
            int code = (resp->buf[0]) & 0xff;

            int errormask = ERROR_ANGLE_LIMIT |
                            ERROR_VOLTAGE     |
                            ERROR_OVERLOAD;

            if ((code & (~errormask)) != 0)
                continue;
        }

        return resp;
    } while (retry && bus->retry_enable);

    return NULL;
}

uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
  uint16_t i, j;
  uint16_t crc_table[256] = { 0x0000,
    0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
    0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
    0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
    0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
    0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
    0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
    0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
    0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
    0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
    0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
    0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
    0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
    0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
    0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
    0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
    0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
    0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
    0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
    0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
    0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
    0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
    0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
    0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
    0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
    0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
    0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
    0x820D, 0x8207, 0x0202 };

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}


// === Bus creation and destruction ==================
dynamixel_bus_t *
serial_bus_create(const char *device, int baud)
{
    dynamixel_bus_t *bus = dynamixel_bus_create();

    // Implementation stuff
    dynamixel_serial_bus_impl_t *impl = malloc(sizeof(*impl));
    impl->baud = baud;
    int fd = serial_open(device, baud, 1);

    // Check to see if we opened a file
    if (fd == -1) {
        printf("ERR: could not open serial port at %s\n", device);
        exit(-1);
    }
    impl->fd = fd;

    bus->impl = impl;

    bus->send_command = serial_bus_send_command;
    bus->destroy = serial_bus_destroy;

    return bus;
}

void
serial_bus_destroy(dynamixel_bus_t *bus)
{
    free(bus->impl);
    dynamixel_bus_destroy(bus);
}
