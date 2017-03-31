#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include "math_util.h"

#include "dynamixel_device.h"

#define dmax(A, B) (A > B ? A : B)
#define dmin(A, B) (A < B ? A : B)
#define dabs(A) (A < 0 ? -A : A)

#define VERBOSE 0

void
dynamixel_set_id(dynamixel_device_t *device, int newid)
{
    assert(newid >=0 && newid < 254);

    if(device->protocol == 1){
        dynamixel_msg_t *msg = dynamixel_msg_create(2);
        msg->buf[0] = 0x03;
        msg->buf[1] = (newid & 0xff);
        dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);
        if (resp == NULL || resp->len < 1 || resp->buf[0] != 0) {
        printf("set_id failed for %d. Aborting in order to avoid EEPROM wear-out.", device->id);
        exit(-1);
        }
        dynamixel_msg_destroy(msg);
        dynamixel_msg_destroy(resp);
    }

    if(device->protocol == 2){
        dynamixel_msg_t *msg = dynamixel_msg_create(3);
        msg->buf[0] = 0x03;
        msg->buf[1] = 0x00;
        msg->buf[2] = (newid & 0xff);
        dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);
        if (resp == NULL || resp->len < 1 || resp->buf[0] != 0) {
            printf("set_id failed for %d. Aborting in order to avoid EEPROM wear-out.", device->id);
            exit(-1);
        } 
        else {
            printf("NOTE: set_id requires power cycling the servo motor before changes fully take effect.\n");
        }
        dynamixel_msg_destroy(msg);
        dynamixel_msg_destroy(resp);
    }
}

void
dynamixel_set_baud(dynamixel_device_t *device, int baud)
{
    int code = 0;

    switch (baud) {
        case 1000000:
            code = 1;
            break;
        case 500000:
            code = 3;
            break;
        case 115200:
            code = 16;
            break;
        case 57600:
            code = 24;
            break;
        default:
            // Unknown baud rate
            assert(0);
    }

    dynamixel_msg_t *msg = dynamixel_msg_create(2);
    msg->buf[0] = 0x04;
    msg->buf[1] = code;
    dynamixel_msg_t *resp = device->ensure_EEPROM(device, msg);

    if (resp == NULL || resp->len < 1 || resp->buf[0] != 0) {
        printf("set_baud failed for %d. Aborting in order to avoid EEPROM wear-out.", device->id);
        exit(-1);
    }

    dynamixel_msg_destroy(msg);
    dynamixel_msg_destroy(resp);
}

int
dynamixel_get_firmware_version(dynamixel_device_t *device)
{
    int version = 0; 

    if(device->protocol == 1){
        dynamixel_msg_t *msg = dynamixel_msg_create(2);
        msg->buf[0] = 0x2;
        msg->buf[1] = 8;
        dynamixel_msg_t *resp = device->read(device, msg, 1);
        version = resp->buf[1]&0xff;
        dynamixel_msg_destroy(msg);
        dynamixel_msg_destroy(resp);
    }
    
    else if(device->protocol == 2){
        dynamixel_msg_t *msg = dynamixel_msg_create(4);
        msg->buf[0] = 0x02;
        msg->buf[1] = 0x00;
        msg->buf[2] = 0x01;
        msg->buf[3] = 0x00;
        dynamixel_msg_t *resp = device->read(device, msg, 1);
        version = resp->buf[1]&0xff;
        dynamixel_msg_destroy(msg);
        dynamixel_msg_destroy(resp);
    }

    else {        
        printf("Incorect Protocol Specified\n");
        return -1;
    }

    return version;
}

int
dynamixel_ping(dynamixel_device_t *device)
{
    dynamixel_msg_t *resp = device->bus->send_command(device->bus,
                                                      device->id,
                                                      device->protocol,
                                                      INST_PING,
                                                      NULL,
                                                      0);

    if (resp == NULL || resp->len != 2)
        return 0;
    dynamixel_msg_destroy(resp);
    return 1;
}

// radians [-pi,pi]
// speedfrac [0, 1] in joint mode
//           [-1,1] in wheel mode (make sure to set continuous mode 1)
// torquefrac [0,1]
void
dynamixel_set_goal(dynamixel_device_t *device,
                   double radians,
                   double speedfrac,
                   double torquefrac)
{
    if (device->rotation_mode)
        device->set_continuous_goal(device, speedfrac, torquefrac);
    else
        device->set_joint_goal(device, radians, speedfrac, torquefrac);
}

// radians [-pi, pi]
// speedfrac [0,1]
// torquefrac [0,1]
void
dynamixel_set_joint_goal_default(dynamixel_device_t *device,
                                 int pmask,
                                 double radians,
                                 double speedfrac,
                                 double torquefrac)
{
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

    dynamixel_msg_t *msg = dynamixel_msg_create(7);
    msg->buf[0] = 0x1e;
    msg->buf[1] = posv & 0xff;
    msg->buf[2] = (posv >> 8) & 0xff;
    msg->buf[3] = speedv & 0xff;
    msg->buf[4] = (speedv >> 8) & 0xff;
    msg->buf[5] = torquev & 0xff;
    msg->buf[6] = (torquev >> 8) & 0xff;
    dynamixel_msg_t *resp = device->write_to_RAM(device, msg, 1);

    dynamixel_msg_destroy(msg);
    if (resp != NULL);
        dynamixel_msg_destroy(resp);

    // Handle speed == 0 case (after slowing down, above) by relaying current
    // position back to servo. Do not set torque == 0, b/c that is possibly not
    // desired...
    if (stop) {
        msg = dynamixel_msg_create(2);
        msg->buf[0] = 0x24;
        msg->buf[1] = 2;
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
            msg = dynamixel_msg_create(3);
            msg->buf[0] = 0x1e;
            msg->buf[1] = posv & 0xff;
            msg->buf[2] = (posv > 8) & 0xff;
            resp = device->write_to_RAM(device, msg, 1);
        }

        if (resp != NULL)
            dynamixel_msg_destroy(resp);
    }
}

// speedfrac [-1,1] pos for CCW, neg for CW
// torquefrac [0,1]
void
dynamixel_set_continuous_goal(dynamixel_device_t *device,
                              double speedfrac,
                              double torquefrac)
{
    assert (device->rotation_mode);

    speedfrac = dmax(-1, dmin(1, speedfrac));
    torquefrac = dmax(0, dmin(1, torquefrac));

    int speedv = (int)abs(speedfrac * 0x3ff);
    if (speedfrac < 0)
        speedv |= 0x400;    // CW direction
    int torquev = (int)(0x3ff * torquefrac);

    dynamixel_msg_t *msg = dynamixel_msg_create(5);
    msg->buf[0] = 0x20;
    msg->buf[1] = speedv & 0xff;
    msg->buf[2] = (speedv >> 8) & 0xff;
    msg->buf[3] = torquev & 0xff;
    msg->buf[4] = (torquev >> 8) & 0xff;

    dynamixel_msg_t *resp = device->write_to_RAM(device, msg, 1);
    dynamixel_msg_destroy(msg);
    if (resp != NULL)
        dynamixel_msg_destroy(resp);
}

void
dynamixel_idle(dynamixel_device_t *device)
{
    device->set_goal(device, 0, 0, 0);
}

// Read data from specified RAM address
// params: parameters of read command. First byte is address in
//         servo control table, followed by number of bytes to read
//         beginning at that address. params->len == 2
// retry:  if 1, retry on non-fatal errors
//
// returns servo response from bus
// User is responsible for cleaning up params and the response
dynamixel_msg_t *
dynamixel_read(dynamixel_device_t *device,
               dynamixel_msg_t *params,
               int retry)
{
    if (params->len != 2 && device->protocol == 1)
        printf("WRN: Invalid read command length %d\n", params->len);
    if (params->len != 4 && device->protocol == 2)
        printf("WRN: Invalid read command length %d\n", params->len);
    return device->bus->send_command(device->bus,
                                     device->id,
                                     device->protocol,
                                     INST_READ_DATA,
                                     params,
                                     retry);
}

dynamixel_msg_t *
dynamixel_read_noretry(dynamixel_device_t *device,
                       dynamixel_msg_t *params,
                       uint8_t num_bytes)
{
    // Doesn't actually use num_bytes in current implementation
    return dynamixel_read(device, params, 0);
}

// Write data to specified RAM address.
// params: parameters of the write command. First byte is address in
//         servo control table, followed by data to write beginning at
//         that address
// retry:  if 1, retry on non-fatal errors
//
// returns servo response from bus.
// User is responsibile for cleaning up params and the response
dynamixel_msg_t *
dynamixel_write_to_RAM(dynamixel_device_t *device,
                       dynamixel_msg_t *params,
                       int retry)
{
    if (device->is_address_EEPROM(0xff & params->buf[0])) {
        printf("WRN: Write failed because RAM address given is in EEPROM area\n");
        return NULL;
    }
    return device->bus->send_command(device->bus,
                                     device->id,
                                     device->protocol,
                                     INST_WRITE_DATA,
                                     params,
                                     retry);
}

dynamixel_msg_t *
dynamixel_write_to_RAM_noretry(dynamixel_device_t *device,
                               dynamixel_msg_t *params)
{
    return device->write_to_RAM(device, params, 0);
}

// Ensure the data at specified EEPROM address
//
// First, read EEPROM bytes and write if different from desired.
// params: parameters of write command. First byte is address in servo
//         control table, followed by data to write beginning at that
//         address. No retry allowed
//
// returns servo response from bus.
//
// User is responsible for cleaning up params and the response
dynamixel_msg_t *
dynamixel_ensure_EEPROM(dynamixel_device_t *device,
                        dynamixel_msg_t *params)
{
    if(device->protocol == 1){
        if (!device->is_address_EEPROM(0xff & params->buf[0])) {
            printf("WRN: Write failed because EEPROM address given is in RAM area.\n");
            return NULL;
        }

        int num_bytes = params->len - 1;
        dynamixel_msg_t *msg = dynamixel_msg_create(2);
        msg->buf[0] = params->buf[0];
        msg->buf[1] = num_bytes & 0xff;
        dynamixel_msg_t *resp = device->read(device, msg, 0);

        dynamixel_msg_destroy(msg);

        if (resp == NULL || resp->len != (num_bytes+2) || resp->buf[0] != 0) {
            printf("WRN: Invalid EEPROM read: \n");
            dynamixel_msg_dump(resp);
            return resp;
        }
        else {
            int differ = 0;
            for (int i = 1; i <= num_bytes && !differ; i++)
                differ |= (params->buf[i] != resp->buf[i]);
            if (!differ) {
                dynamixel_msg_destroy(resp);
                resp = dynamixel_msg_create(1);
                resp->buf[0] = 0;
                return resp;    // as if no error write occured (w/o checksum)
            }
            printf("WRN: Writing to EEPROM (address %d)\n", (0xff & params->buf[0]));
        }

        dynamixel_msg_destroy(resp);
        resp = device->bus->send_command(device->bus,
                                         device->id,
                                         device->protocol,
                                         INST_WRITE_DATA,
                                         params,
                                         0);
        if (resp == NULL || resp->len != 2 || resp->buf[0] != 0) {
            printf("WRN: Error occurred while writing to EEPROM");
            dynamixel_msg_dump(resp);
        }
        return resp;
    }
    else if(device->protocol == 2){
        if (!device->is_address_EEPROM(0xff & params->buf[0])) {
            printf("WRN: Write failed because EEPROM address given is in RAM area.\n");
            return NULL;
        }
        
        // Disable the torque enable message to allow writing to the EEPROM
        dynamixel_msg_t *offMsg = dynamixel_msg_create(3);
        offMsg->buf[0] = 0x18;
        offMsg->buf[1] = 0x00;
        offMsg->buf[2] = 0x00;
        dynamixel_msg_t *offResp = device->write_to_RAM(device, offMsg, 0);
        if (offResp == NULL) {
            printf("Turning off torque enabled failed.\n");
        }
        else {
            dynamixel_msg_destroy(offResp);
        }
        dynamixel_msg_destroy(offMsg);
        
        // Send the EEPROM command
        int num_bytes = params->len - 2;
        
        dynamixel_msg_t *msg = dynamixel_msg_create(4);
        msg->buf[0] = params->buf[0];
        msg->buf[1] = 0x00;
        msg->buf[2] = num_bytes & 0xff;
        msg->buf[3] = 0x00;
        dynamixel_msg_t *resp = device->read(device, msg, 0);
        if(VERBOSE){
            printf("MSG_V2 = %d :", msg->len);
            dynamixel_msg_dump(msg);
            printf("PARAMS_V2 = %d :", params->len);
            dynamixel_msg_dump(params);
            printf("RESP_V2 = %d : ",resp->len);
            dynamixel_msg_dump(resp);
        }
        if (resp == NULL || resp->len != (num_bytes+3) || resp->buf[0] != 0) {
            printf("WRN: Invalid EEPROM read = %d: \n",resp->len);
            dynamixel_msg_dump(resp);
            return resp;
        }
        else {
            int differ = 0;
            for (int i = 2; i <= num_bytes + 1 && !differ; i++){
                differ |= (params->buf[i] != resp->buf[i-1]);
            }
            if (!differ) {
                dynamixel_msg_destroy(resp);
                resp = dynamixel_msg_create(1);
                resp->buf[0] = 0;
                return resp;    // as if no error write occured (w/o checksum)
            }
            printf("WRN: Writing to EEPROM (address %d)\n", (0xff & params->buf[0]));
        }

        dynamixel_msg_destroy(resp);
        resp = device->bus->send_command(device->bus,
                                         device->id,
                                         device->protocol,
                                         INST_WRITE_DATA,
                                         params,
                                         0);
        if (resp == NULL || resp->len != 3 || resp->buf[0] != 0) {
            printf("WRN: Error occurred while writing to EEPROM");
            dynamixel_msg_dump(resp);
        }
        
        // After changing EEPROM, re-enable torque for motor control -- NOTE: This will fail for set-id
        dynamixel_msg_t *onMsg = dynamixel_msg_create(3);
        onMsg->buf[0] = 0x18;
        onMsg->buf[1] = 0x00;
        onMsg->buf[2] = 0x01;
        dynamixel_msg_t* onResp = device->write_to_RAM(device, onMsg, 0);
        if (onResp == NULL) {
            printf("Turning on torque enabled failed.\n");
        }
        else {
            dynamixel_msg_destroy(onResp);
        }
        
        dynamixel_msg_destroy(onMsg);
        
        return resp;
    }
    else{
        printf("Incorect Protocol Specified\n");
        return NULL;
    }
}

// Read (and set) the rotation mode from servo
int
dynamixel_read_rotation_mode(dynamixel_device_t *device)
{
    int mode = 1;
    if(device->protocol == 1){  
        dynamixel_msg_t *msg = dynamixel_msg_create(2);
        msg->buf[0] = 0x06;
        msg->buf[1] = 4;
        dynamixel_msg_t *resp = device->read(device, msg, 1);

        dynamixel_msg_destroy(msg);
        if (resp == NULL || resp->len != 6) {
            printf("WRN: Invalid read of continuous state: len=%d\n",
                   resp == NULL ? 0 : resp->len);
            dynamixel_msg_destroy(resp);
            return device->rotation_mode;   // best guess
        }
        for (int i = 1; i < 5; i++) {
            if (resp->buf[i] != 0) {
                mode = 0;
                break;
            }
        }
        device->rotation_mode = mode;
        dynamixel_msg_destroy(resp);
    }

    if(device->protocol == 2){
        dynamixel_msg_t *msg = dynamixel_msg_create(4);
        msg->buf[0] = 0x0B;
        msg->buf[1] = 0x00;
        msg->buf[2] = 0x01;
        msg->buf[3] = 0x00;
        dynamixel_msg_t *resp = device->read(device, msg, 1);
        dynamixel_msg_dump(resp);
        if (resp == NULL || resp->len != 4) {
            printf("WRN: Invalid read of continuous state: len=%d\n",
                    resp == NULL ? 0 : resp->len);
            dynamixel_msg_destroy(resp);
            return device->rotation_mode;
        }
        return -1;

    }
    return mode;
}

void
dynamixel_set_continuous_mode(dynamixel_device_t *device, int mode)
{
    printf("NFO: Setting rotation mode for servo %d to %d (%s)\n",
           device->id,
           mode,
           mode ? "wheel" : "joint");

    device->set_rotation_mode(device, mode);
    device->rotation_mode = mode;
}

// === Create/Destroy default device ========================
dynamixel_device_t *
dynamixel_device_create(uint8_t id, uint8_t protocol)
{
    dynamixel_device_t *device = malloc(sizeof(*device));
    device->id = id;
    device->protocol = protocol;
    device->rotation_mode = 0;

    device->destroy = dynamixel_device_destroy;

    // Set functions that we have
    device->set_id = dynamixel_set_id;
    device->set_baud = dynamixel_set_baud;
    device->get_firmware_version = dynamixel_get_firmware_version;
    device->ping = dynamixel_ping;
    device->set_goal = dynamixel_set_goal;
    device->set_continuous_goal = dynamixel_set_continuous_goal;
    device->set_continuous_mode = dynamixel_set_continuous_mode;
    device->idle = dynamixel_idle;
    device->read = dynamixel_read;
    device->read_noretry = dynamixel_read_noretry;
    device->write_to_RAM = dynamixel_write_to_RAM;
    device->write_to_RAM_noretry = dynamixel_write_to_RAM_noretry;
    device->ensure_EEPROM = dynamixel_ensure_EEPROM;
    device->read_rotation_mode = dynamixel_read_rotation_mode;

    // A few more functions/variables must be provided by the final type
    // device->is_address_EEPROM
    // device->get_min_position_radians
    // device->get_max_position_radians
    // device->set_joint_goal (feel free to use the partial implementation provided)
    // device->get_status
    // device->set_rotation_mode
    //
    // device->bus

    return device;
}

void
dynamixel_device_destroy(dynamixel_device_t *device)
{
    //device->bus->bus_destroy(device->bus);
    free(device);
}

// Make sure that buf is large enough
void
dynamixel_device_status_to_string(dynamixel_device_status_t *status, char *buf)
{
    sprintf(buf,
            "pos=%6.3f, speed=%6.3f, load=%6.3f, volts=%4.1f, temp=%4.1f, mode=%s, err=%08x",
            status->position_radians,
            status->speed,
            status->load,
            status->voltage,
            status->temperature,
            status->continuous ? "wheel" : "joint",
            status->error_flags);
}

dynamixel_device_status_t *
dynamixel_device_status_create(void)
{
    dynamixel_device_status_t *status = malloc(sizeof(*status));
    status->to_string = dynamixel_device_status_to_string;

    return status;
}

void
dynamixel_device_status_destroy(dynamixel_device_status_t *status)
{
    free(status);
}
