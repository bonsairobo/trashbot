#ifndef __DYNAMIXEL_BUS_H__
#define __DYNAMIXEL_BUS_H__

#include <stdint.h>

#define INST_PING           0x01
#define INST_READ_DATA      0x02
#define INST_WRITE_DATA     0x03
#define INST_REG_WRITE      0x04
#define INST_ACTION         0x05
#define INST_RESET_DATA     0x06
#define INST_SYNC_WRITE     0x83

// Forward declarations
typedef struct dynamixel_device dynamixel_device_t;

typedef struct dynamixel_msg dynamixel_msg_t;
struct dynamixel_msg
{
    int len;
    uint8_t *buf;
};

typedef struct dynamixel_bus dynamixel_bus_t;
struct dynamixel_bus
{
    void *impl;

    int retry_enable;

    dynamixel_msg_t * (*send_command)(dynamixel_bus_t *bus,
                                      int id,
                                      int protocol,
                                      int instruction,
                                      dynamixel_msg_t *msg,
                                      int retry);

    void (*set_retry_enable)(dynamixel_bus_t *bus, int retry_enable);
    int (*get_servo_model)(dynamixel_bus_t *bus, uint8_t id);
    int (*get_servo_model_2)(dynamixel_bus_t *bus, uint8_t id);

    dynamixel_device_t * (*get_servo)(dynamixel_bus_t *bus, uint8_t id);

    // Required for your bus to clean itself up
    void (*destroy)(dynamixel_bus_t *bus);
};

// === Message creation, destruction, and debugging ===========
dynamixel_msg_t * dynamixel_msg_create(int len);
void dynamixel_msg_destroy(dynamixel_msg_t *msg);
void dynamixel_msg_dump(dynamixel_msg_t *msg);

// === Default bus stuff ======================================
void dynamixel_bus_set_retry_enable(dynamixel_bus_t *bus, int retry_enable);
int dynamixel_bus_get_servo_model(dynamixel_bus_t *bus, uint8_t id);
int dynamixel_bus_get_servo_model_2(dynamixel_bus_t *bus, uint8_t id);
dynamixel_device_t * dynamixel_bus_get_servo(dynamixel_bus_t *bus, uint8_t id);

dynamixel_bus_t * dynamixel_bus_create(void);
void dynamixel_bus_destroy(dynamixel_bus_t *bus);

#endif
