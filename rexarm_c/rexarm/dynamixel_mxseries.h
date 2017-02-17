#ifndef __MXSERIES_H__
#define __MXSERIES_H__

#include <stdint.h>
#include "dynamixel_bus.h"
#include "dynamixel_device.h"

dynamixel_device_t *
dynamixel_mxseries_create(dynamixel_bus_t *bus, uint8_t id);

#endif
