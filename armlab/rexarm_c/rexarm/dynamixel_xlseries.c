#include <stdlib.h>
#include <stdio.h>

#include "math_util.h"

#include "dynamixel_device.h"
#include "dynamixel_xlseries.h"

#define dmax(A, B) (A > B ? A : B)
#define dmin(A, B) (A < B ? A : B)
#define dabs(A) (A < 0 ? -A : A)

// === XL series device implementation =============
static int
xlseries_is_address_EEPROM(int address)
{
    return address < 0x18;
}

static double
xlseries_get_min_position_radians(dynamixel_device_t *device)
{
    return to_radians(-150);
}

static double
xlseries_get_max_position_radians(dynamixel_device_t *device)
{
    return to_radians(150);
}

static void
xlseries_set_joint_goal(dynamixel_device_t *device,
                        double radians,
                        double speedfrac,
                        double torquefrac)
{
    int pmask = 0x3FF;
    assert (!device->rotation_mode && (pmask == 0xfff || pmask == 0x3ff));

    // Ensure proper ranges
    radians = mod2pi(radians);
    speedfrac = dmax(0.0, dmin(1.0, dabs(speedfrac)));
    torquefrac = dmax(0.0, dmin(1.0, torquefrac));

    double min = device->get_min_position_radians(device);
    double max = device->get_max_position_radians(device);
    radians = dmax(min, dmin(max, radians));

    int stop = speedfrac < (1.0/0x3ff);

    int posv = ((int) round((radians - min) / (max - min) * pmask)) & pmask;
    // in joint-mode, speed == 0 --> maxspeed
    int speedv = stop ? 0x1 : (int)(speedfrac * 0x3ff);
    int torquev = (int)(torquefrac * 0x3ff);

    dynamixel_msg_t *msg = dynamixel_msg_create(9);
    msg->buf[0] = 0x1e;
    msg->buf[1] = 0x00;
    msg->buf[2] = posv & 0xff;
    msg->buf[3] = (posv >> 8) & 0xff;
    msg->buf[4] = speedv & 0xff;
    msg->buf[5] = (speedv >> 8) & 0xff;
    msg->buf[6] = 0x00;
    msg->buf[7] = torquev & 0xff;
    msg->buf[8] = (torquev >> 8) & 0xff;
    dynamixel_msg_t *resp = device->bus->send_command(device->bus,
                                         device->id,
                                         device->protocol,
                                         INST_WRITE_DATA,
                                         msg,
                                         1);

    dynamixel_msg_destroy(msg);
    if (resp != NULL);
        dynamixel_msg_destroy(resp);

    // Handle speed == 0 case (after slowing down, above) by relaying current
    // position back to servo. Do not set torque == 0, b/c that is possibly not
    // desired...
    if (stop) {
        msg = dynamixel_msg_create(4);
        msg->buf[0] = 0x24;
        msg->buf[1] = 0x00;
        msg->buf[2] = 0x02;
        msg->buf[3] = 0x00;
        resp = device->bus->send_command(device->bus,
                                         device->id,
                                         device->protocol,
                                         INST_READ_DATA,
                                         msg,
                                         1);
        dynamixel_msg_destroy(msg);
        if (resp != NULL) {
            dynamixel_msg_destroy(resp);
            posv = (resp->buf[1] & 0xff) + ((resp->buf[2] & 0xff) << 8);
            msg = dynamixel_msg_create(4);
            msg->buf[0] = 0x1e;
            msg->buf[1] = 0x00;
            msg->buf[1] = posv & 0xff;
            msg->buf[2] = (posv > 8) & 0xff;
            resp = device->bus->send_command(device->bus,
                                         device->id,
                                         device->protocol,
                                         INST_WRITE_DATA,
                                         msg,
                                         1);
        }

        if (resp != NULL)
            dynamixel_msg_destroy(resp);
    }
}

static dynamixel_device_status_t *
xlseries_get_status(dynamixel_device_t *device)
{
    //printf("Getting Status...\n");
    dynamixel_msg_t *msg = dynamixel_msg_create(4);
    msg->buf[0] = 0x25;
    msg->buf[1] = 0x00;
    msg->buf[2] = 0x0B;
    msg->buf[3] = 0x00;
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
    stat->voltage = (resp->buf[9] & 0xff) / 10.0;   // scale to voltage
    stat->temperature = (resp->buf[10] & 0xff);      // deg celsius
    stat->continuous = device->rotation_mode;
    stat->error_flags = (resp->buf[0] & 0xff);

    return stat;
}

static void
xlseries_set_rotation_mode(dynamixel_device_t *device, int mode)
{
    //address 0x0B controls joint/wheel mode. 1=wheel, 2=joint
    dynamixel_msg_t *msg = dynamixel_msg_create(3);
    msg->buf[0] = 0x0B;
    msg->buf[1] = 0x00;
    msg->buf[2] = mode ? 0x01 : 0x02;
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    dynamixel_msg_destroy(msg);
    if (resp != NULL)
        dynamixel_msg_destroy(resp);
}

static const char *
xlseries_get_name(dynamixel_device_t *device)
{
    // XXX Later, give version lookup?
    return "XL-Series";
}

// === XL series device creation ===================
dynamixel_device_t *
dynamixel_xlseries_create(dynamixel_bus_t *bus, uint8_t id)
{
    dynamixel_device_t *device = dynamixel_device_create(id,2);

    // Bus stuff
    device->bus = bus;

    device->is_address_EEPROM = xlseries_is_address_EEPROM;
    device->get_min_position_radians = xlseries_get_min_position_radians;
    device->get_max_position_radians = xlseries_get_max_position_radians;
    device->set_joint_goal = xlseries_set_joint_goal;
    device->get_status = xlseries_get_status;
    device->set_rotation_mode = xlseries_set_rotation_mode;
    device->get_name = xlseries_get_name;

    //Return delay time
    uint8_t delay = 0x02;   // each unit = 2 usec
    dynamixel_msg_t *msg = dynamixel_msg_create(3);
    msg->buf[0] = 0x05;
    msg->buf[1] = 0x00;
    msg->buf[2] = delay;
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    dynamixel_msg_destroy(msg);
    if (resp != NULL)
        dynamixel_msg_destroy(resp);

    //Torque Enable
    msg = dynamixel_msg_create(3);
    msg->buf[0] = 0x18;
    msg->buf[1] = 0x00;
    msg->buf[2] = 0x01;
    resp = device->bus->send_command(device->bus,
                                    device->id,
                                    device->protocol,
                                    INST_WRITE_DATA,
                                    msg,
                                    1);

    dynamixel_msg_destroy(msg);
    if (resp != NULL)
        dynamixel_msg_destroy(resp);

    return device;
}
