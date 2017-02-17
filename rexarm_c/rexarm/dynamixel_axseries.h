#ifndef __AXSERIES_H__
#define __AXSERIES_H__

#include <stdint.h>
#include "dynamixel_bus.h"
#include "dynamixel_device.h"

dynamixel_device_t *
dynamixel_axseries_create(dynamixel_bus_t *bus, uint8_t id);

#endif
