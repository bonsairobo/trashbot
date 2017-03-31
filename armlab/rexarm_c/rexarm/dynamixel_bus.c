#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_bus.h"
#include "dynamixel_axseries.h"
#include "dynamixel_xlseries.h"
#include "dynamixel_mxseries.h"

// === Messages (passed to and from the dynamixel devices over the bus ===

dynamixel_msg_t *
dynamixel_msg_create(int len)
{
    dynamixel_msg_t *msg = malloc(sizeof(*msg));
    msg->len = len;
    msg->buf = malloc(len*sizeof(uint8_t));
    return msg;
}

void
dynamixel_msg_destroy(dynamixel_msg_t *msg)
{
    if (msg) {
        free(msg->buf);
        free(msg);
    }
}

void
dynamixel_msg_dump(dynamixel_msg_t *msg)
{
    if (msg) {
        for (int i = 0; i < msg->len; i++)
            printf("%02x ", msg->buf[i] & 0xff);
    }
    
    printf("\n");
}


// === Bus Default Implementation =============

void
dynamixel_bus_set_retry_enable(dynamixel_bus_t *bus, int retry_enable)
{
    bus->retry_enable = retry_enable;
}

int
dynamixel_bus_get_servo_model(dynamixel_bus_t *bus, uint8_t id)
{
    dynamixel_msg_t *msg = dynamixel_msg_create(2);
    msg->buf[0] = 0x00;
    msg->buf[1] = 0x02;
    dynamixel_msg_t *resp = bus->send_command(bus,
                                              id,
                                              1,
                                              INST_READ_DATA,
                                              msg,
                                              0);
    if (resp == NULL)
        return -1;

    dynamixel_msg_destroy(msg);
    int v = (resp->buf[1] & 0xff) + ((resp->buf[2] & 0xff) << 8);
    dynamixel_msg_destroy(resp);
    return v;
}

int
dynamixel_bus_get_servo_model_2(dynamixel_bus_t *bus, uint8_t id)
{
    dynamixel_msg_t * msg = dynamixel_msg_create(4);
    msg->buf[0] = 0x00;
    msg->buf[1] = 0x00;
    msg->buf[2] = 0x02;
    msg->buf[3] = 0x00;
    dynamixel_msg_t *resp = bus->send_command(bus,
                              id,
                              2,
                              INST_READ_DATA,
                              msg,
                              0);


    if (resp == NULL){
        return -1;
    }
    dynamixel_msg_destroy(msg);
    int v = (resp->buf[1] & 0xff) + ((resp->buf[2] & 0xff) << 8);
    dynamixel_msg_destroy(resp);
    return v;
}

dynamixel_device_t *
dynamixel_bus_get_servo(dynamixel_bus_t *bus, uint8_t id)
{
    int model = bus->get_servo_model(bus, id);
    if (model < 0)
        model = bus->get_servo_model_2(bus, id);
    if (model < 0)
        return NULL;

    switch (model) {
        case 0x000c: // definitely for AX12+. Do other AX12 variants have same ID?
            return dynamixel_axseries_create(bus, id);
        case 0x001d: // MX28T
        case 0x0136: // MX64T
        case 0x0140: // MX106T
        case 0x0168: // MX12W
            return dynamixel_mxseries_create(bus, id);
        case 0x015E:
            return dynamixel_xlseries_create(bus, id);
        default:
            break;
    }

    printf("WRN: Bus did not recognize unknown servo type %04x at id %d\n",
           model, id);

    return NULL;
}

dynamixel_bus_t *
dynamixel_bus_create(void)
{
    dynamixel_bus_t *bus = malloc(sizeof(*bus));
    bus->retry_enable = 1;

    // Set default functions
    bus->set_retry_enable = dynamixel_bus_set_retry_enable;
    bus->get_servo_model = dynamixel_bus_get_servo_model;
    bus->get_servo_model_2 = dynamixel_bus_get_servo_model_2;
    bus->get_servo = dynamixel_bus_get_servo;

    // User will need to fill the following:
    //bus->send_command = serial_bus_send_command

    return bus;
}

void
dynamixel_bus_destroy(dynamixel_bus_t *bus)
{
    free(bus);
}
