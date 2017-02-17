#ifndef __DYNAMIXEL_DEVICE_H__
#define __DYNAMIXEL_DEVICE_H__

#include <stdint.h>

#include "dynamixel_bus.h"

#define ERROR_INSTRUCTION (1 << 6)
#define ERROR_OVERLOAD    (1 << 5)
#define ERROR_CHECKSUM    (1 << 4)
#define ERROR_RANGE       (1 << 3)
#define ERROR_OVERHEAT    (1 << 2)
#define ERROR_ANGLE_LIMIT (1 << 1)
#define ERROR_VOLTAGE     (1 << 0)

typedef struct dynamixel_device_status dynamixel_device_status_t;
struct dynamixel_device_status
{
    double position_radians;
    double speed;
    double load;
    double voltage;
    double temperature;

    int continuous;
    int32_t error_flags;

    void (*to_string)(dynamixel_device_status_t *status, char *buf);
};

typedef struct dynamixel_device dynamixel_device_t;
struct dynamixel_device
{
    void *impl;

    dynamixel_bus_t *bus;
    int id;
    int protocol;
    int rotation_mode;

    void (*destroy)(dynamixel_device_t *device);

    // === General functionality ===============================
    dynamixel_msg_t * (*ensure_EEPROM)(dynamixel_device_t *device, dynamixel_msg_t *params);
    int (*is_address_EEPROM)(int addr);
    int (*get_firmware_version)(dynamixel_device_t *device);

    void (*idle)(dynamixel_device_t *device);
    // Returns >0 if device is on the bus
    int (*ping)(dynamixel_device_t *device);
    const char * (*get_name)(dynamixel_device_t *device);

    // Read/write data from/to specified address.
    dynamixel_msg_t * (*read)(dynamixel_device_t *device, dynamixel_msg_t *params, int retry);
    dynamixel_msg_t * (*read_noretry)(dynamixel_device_t *device, dynamixel_msg_t *params, uint8_t num_bytes);
    dynamixel_msg_t * (*write_to_RAM_noretry)(dynamixel_device_t *device, dynamixel_msg_t *params);
    dynamixel_msg_t * (*write_to_RAM)(dynamixel_device_t *device, dynamixel_msg_t *params, int retry);


    void (*set_baud)(dynamixel_device_t *device, int baud);
    void (*set_goal)(dynamixel_device_t *device, double radians, double speedfrac, double torquefrac);
    void (*set_id)(dynamixel_device_t *device, int newid);
    // Set goal in joint mode
    void (*set_joint_goal)(dynamixel_device_t *device, double radians, double speedfrac, double torquefrac);



    // === Servo Specific Functionality ========================
    // Ensure data at specified EEPROM address. First, read EEPROM bytes
    // and write if different from desired
    double (*get_min_position_radians)(dynamixel_device_t *device);
    double (*get_max_position_radians)(dynamixel_device_t *device);
    int (*read_rotation_mode)(dynamixel_device_t *device);
    void (*set_rotation_mode)(dynamixel_device_t *device, int mode);
    void (*set_continuous_goal)(dynamixel_device_t *device, double speedfrac, double torquefrac);
    void (*set_continuous_mode)(dynamixel_device_t *device, int mode);

    dynamixel_device_status_t * (*get_status)(dynamixel_device_t *device);
};

// === Available general purpose functionality =================
void dynamixel_set_id(dynamixel_device_t *device, int newid);
void dynamixel_set_baud(dynamixel_device_t *device, int baud);
int dynamixel_get_firmware_version(dynamixel_device_t *device);
int dynamixel_ping(dynamixel_device_t *device);
void dynamixel_set_goal(dynamixel_device_t *device, double radians, double speedfrac, double torquefrac);
void dynamixel_set_joint_goal_default(dynamixel_device_t *device, int pmask, double radians, double speedfrac, double torquefrac);
void dynamixel_set_continuous_goal(dynamixel_device_t *device, double speedfrac, double torquefrac);
void dynamixel_idle(dynamixel_device_t *device);
dynamixel_msg_t * dynamixel_read(dynamixel_device_t *device, dynamixel_msg_t *params, int retry);
dynamixel_msg_t * dynamixel_read_noretry(dynamixel_device_t *device, dynamixel_msg_t *params, uint8_t num_bytes);
dynamixel_msg_t * dynamixel_write_to_RAM(dynamixel_device_t *device, dynamixel_msg_t *params, int retry);
dynamixel_msg_t * dynamixel_write_to_RAM_noretry(dynamixel_device_t *device, dynamixel_msg_t *params);
dynamixel_msg_t * dynamixel_ensure_EEPROM(dynamixel_device_t *device, dynamixel_msg_t *params);
int dynamixel_get_rotation_mode(dynamixel_device_t *device);
int dynamixel_read_rotation_mode(dynamixel_device_t *device);
void dynamixel_set_continuous_mode(dynamixel_device_t *device, int mode);

// Create a default dynamixel_device_t with most of the functionality already set
dynamixel_device_t *dynamixel_device_create(uint8_t id, uint8_t protocol);
void dynamixel_device_destroy(dynamixel_device_t *device);

// Status creation
dynamixel_device_status_t * dynamixel_device_status_create(void);
void dynamixel_device_status_destroy(dynamixel_device_status_t *status);

#endif
