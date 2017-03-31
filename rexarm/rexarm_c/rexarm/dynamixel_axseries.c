#include <stdlib.h>
#include <stdio.h>

#include "math_util.h"

#include "dynamixel_device.h"
#include "dynamixel_axseries.h"

// === AX series device implementation =============
static int
axseries_is_address_EEPROM(int address)
{
    return address < 0x18;
}

static double
axseries_get_min_position_radians(dynamixel_device_t *device)
{
    return to_radians(-150);
}

static double
axseries_get_max_position_radians(dynamixel_device_t *device)
{
    return to_radians(150);
}

static void
axseries_set_joint_goal(dynamixel_device_t *device,
                        double radians,
                        double speedfrac,
                        double torquefrac)
{
    dynamixel_set_joint_goal_default(device,
                                     0x3ff,
                                     radians,
                                     speedfrac,
                                     torquefrac);
}

static dynamixel_device_status_t *
axseries_get_status(dynamixel_device_t *device)
{
    dynamixel_msg_t *msg = dynamixel_msg_create(2);
    msg->buf[0] = 0x24;
    msg->buf[1] = 8;
    dynamixel_msg_t *resp = device->bus->send_command(device->bus,
                                                      device->id,
                                                      device->protocol,
                                                      INST_READ_DATA,
                                                      msg,
                                                      1);
    dynamixel_msg_destroy(msg);

    if (resp == NULL)
        return NULL;

    dynamixel_device_status_t *stat = dynamixel_device_status_create();
    stat->position_radians = ((resp->buf[1] & 0xff) +
                              ((resp->buf[2] & 0x3) << 8)) *
                             to_radians(300) / 1024.0 - to_radians(150);
    int tmp = ((resp->buf[3] & 0xff) + ((resp->buf[4] & 0x3f) << 8));
    if (tmp < 1024)
        stat->speed = tmp / 1023.0;
    else
        stat->speed = -(tmp - 1024)/1023.0;

    // load is signed, we scale to [-1, 1]
    tmp = (resp->buf[5] & 0xff) + ((resp->buf[6] & 0xff) << 8);
    if (tmp < 1024)
        stat->load = tmp / 1023.0;
    else
        stat->load = -(tmp - 1024) / 1023.0;

    stat->voltage = (resp->buf[7] & 0xff) / 10.0;   // scale to voltage
    stat->temperature = (resp->buf[8] & 0xff);      // deg celsius
    stat->continuous = device->rotation_mode;
    stat->error_flags = (resp->buf[0] & 0xff);

    return stat;
}

static void
axseries_set_rotation_mode(dynamixel_device_t *device, int mode)
{
    dynamixel_msg_t *msg = dynamixel_msg_create(5);
    msg->buf[0] = 0x06;
    msg->buf[1] = 0;
    msg->buf[2] = 0;
    msg->buf[3] = mode ? 0 : 0xff;
    msg->buf[4] = mode ? 0 : 0x03;
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    dynamixel_msg_destroy(msg);
    if (resp != NULL)
        dynamixel_msg_destroy(resp);
}

static const char *
axseries_get_name(dynamixel_device_t *device)
{
    // XXX Later, give version lookup?
    return "AX-Series";
}

// === AX series device creation ===================
dynamixel_device_t *
dynamixel_axseries_create(dynamixel_bus_t *bus, uint8_t id)
{
    dynamixel_device_t *device = dynamixel_device_create(id,1);

    // Bus stuff
    device->bus = bus;

    device->is_address_EEPROM = axseries_is_address_EEPROM;
    device->get_min_position_radians = axseries_get_min_position_radians;
    device->get_max_position_radians = axseries_get_max_position_radians;
    device->set_joint_goal = axseries_set_joint_goal;
    device->get_status = axseries_get_status;
    device->set_rotation_mode = axseries_set_rotation_mode;
    device->get_name = axseries_get_name;

    // Return delay time
    uint8_t delay = 0x02;   // each unit = 2 usec
    dynamixel_msg_t *msg = dynamixel_msg_create(2);
    msg->buf[0] = 0x5;
    msg->buf[1] = delay;
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    dynamixel_msg_destroy(msg);
    if (resp != NULL)
        dynamixel_msg_destroy(resp);

    return device;
}
