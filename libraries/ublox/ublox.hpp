//
//  ublox.hpp
//  FC_Safe_vector
//
//  Created by blue-i on 01/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#ifndef ublox_hpp
#define ublox_hpp

#include <stdio.h>

#include <iostream>
#include <iostream>
#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>        //Used for UART
#include <math.h>
#include <sys/ioctl.h>
#include "string.h"
#include "poll.h"


#define UBLOX_INITIAL_BAUD 9600
#define UBLOX_OPERATING_BAUD 57600

#define UBX_SYNC_CHAR_1 (0xb5)
#define UBX_SYNC_CHAR_2 (0x62)
#define UBX_CLASS_NAV (0x01)
#define UBX_ID_POS_LLH (0x02)
#define UBX_ID_VEL_NED (0x12)
#define UBX_ID_SOL (0x06)
#define UBX_ID_TIME_UTC (0x21)

#define UBX_FRESHNESS_LIMIT (500)  // millisends

struct UBXPosLLH
{
    uint32_t gps_ms_time_of_week;
    int32_t longitude;
    int32_t latitude;
    int32_t height_above_ellipsoid;
    int32_t height_mean_sea_level;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
} __attribute__((packed));

struct UBXVelNED
{
    uint32_t gps_ms_time_of_week;
    int32_t velocity_north;
    int32_t velocity_east;
    int32_t velocity_down;
    uint32_t total_speed;
    uint32_t horizontal_speed;
    int32_t course;
    uint32_t speed_accuracy;
    uint32_t course_accuracy;
} __attribute__((packed));

struct UBXSol
{
    uint32_t gps_ms_time_of_week;
    int32_t fractional_time_of_week;
    int16_t gps_week;
    uint8_t gps_fix_type;
    uint8_t gps_fix_status_flags;
    int32_t ecef_x_coordinate;
    int32_t ecef_y_coordinate;
    int32_t ecef_z_coordinate;
    uint32_t coordinate_accuracy;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t velocity_accuracy;
    uint16_t position_dop;
    uint8_t reserved1;
    uint8_t number_of_satelites_used;
    uint32_t reserved2;
} __attribute__((packed));

#define UBLOX_DATA_BUFFER_LENGTH (sizeof(struct UBXSol))

struct UBXTimeUTC
{
    uint32_t gps_ms_time_of_week;
    uint32_t t_acc;
    int32_t nano;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
} __attribute__((packed));

enum UBXNewDataBits {
    UBX_NEW_DATA_BIT_POS_LLH  = 1<<0,
    UBX_NEW_DATA_BIT_VEL_NED  = 1<<1,
    UBX_NEW_DATA_BIT_SOL      = 1<<2,
    UBX_NEW_DATA_BIT_TIME_UTC = 1<<3,
};

enum UBXErrorBits {
    UBX_ERROR_BIT_STALE = 1<<0,
};

struct UBXPayload {
    int32_t longitude; // [10^-7 deg]
    int32_t latitude; // [10^-7 deg]
    float z; // height above sea level [m], downward positive
    float velocity[3]; // [m/s]
    uint8_t gps_status; // 3: pos & vel OK 2: only pos OK 1: only vel OK 0: unavailable
} __attribute__((packed));

void ProcessIncomingUBloxByte(uint8_t byte);

void CopyUBloxMessage(uint8_t id);

const struct UBXPayload * UBXPayload(void);

uint8_t UBXNewDataAvailable(void);

// -----------------------------------------------------------------------------
void ClearUBXNewDataFlags(void);
#endif /* ublox_hpp */
