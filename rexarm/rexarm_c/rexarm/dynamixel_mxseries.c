#include <stdlib.h>
#include <stdio.h>
#include <float.h>

#include "math_util.h"

#include "dynamixel_device.h"
#include "dynamixel_mxseries.h"

// === MX series device implementation =============
static int
mxseries_is_address_EEPROM(int address)
{
    return address < 0x19;
}

static double
mxseries_get_min_position_radians(dynamixel_device_t *device)
{
    return -M_PI;
}

static double
mxseries_get_max_position_radians(dynamixel_device_t *device)
{
    return M_PI;
}

// XXX PID controls currently not supported

static void
mxseries_set_joint_goal(dynamixel_device_t *device,
                        double radians,
                        double speedfrac,
                        double torquefrac)
{
    dynamixel_set_joint_goal_default(device,
                                     0xfff,
                                     radians,
                                     speedfrac,
                                     torquefrac);
}

static dynamixel_device_status_t *
mxseries_get_status(dynamixel_device_t *device)
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
                              ((resp->buf[2] & 0xf) << 8)) *
                              2 * M_PI / 0xfff - M_PI;
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

    // foo
    //stat->voltage = 5.0;

    return stat;
}

static void
mxseries_set_rotation_mode(dynamixel_device_t *device, int mode)
{
    dynamixel_msg_t *msg = dynamixel_msg_create(5);
    msg->buf[0] = 0x06;
    msg->buf[1] = 0;
    msg->buf[2] = 0;
    msg->buf[3] = mode ? 0 : 0xff;
    msg->buf[4] = mode ? 0 : 0x0f;
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    dynamixel_msg_destroy(msg);
    if (resp != NULL)
        dynamixel_msg_destroy(resp);
}

static const char *
mxseries_get_name(dynamixel_device_t *device)
{
    // XXX Later, give version lookup
    return "MX-Series";
}

// === MX series device creation ===================
dynamixel_device_t *
dynamixel_mxseries_create(dynamixel_bus_t *bus,
                          uint8_t id)
{
    dynamixel_device_t *device = dynamixel_device_create(id,1);

    // Bus stuff
    device->bus = bus;
    
    device->is_address_EEPROM = mxseries_is_address_EEPROM;
    device->get_min_position_radians = mxseries_get_min_position_radians;
    device->get_max_position_radians = mxseries_get_max_position_radians;
    device->set_joint_goal = mxseries_set_joint_goal;
    device->get_status = mxseries_get_status;
    device->set_rotation_mode = mxseries_set_rotation_mode;
    device->get_name = mxseries_get_name;

    // Return delay time
    uint8_t delay = 0x02;   // each unit = 2 usec
    dynamixel_msg_t *msg = dynamixel_msg_create(2);
    msg->buf[0] = 0x5;
    msg->buf[1] = delay;
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    if (resp != NULL)
        dynamixel_msg_destroy(resp);

    // Set Alarm Shutdown (EEPROM)
    msg->buf[0] = 18;
    msg->buf[1] = 36;
    resp = device->ensure_EEPROM(device, msg);

    dynamixel_msg_destroy(msg);
    if (resp != NULL)
        dynamixel_msg_destroy(resp);

    return device;
}
