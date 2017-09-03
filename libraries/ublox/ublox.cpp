//
//  ublox.cpp
//  FC_Safe_vector
//
//  Created by blue-i on 01/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#include "ublox.hpp"


uint32_t status_ = 0;
uint32_t new_data_bits_ = 0;


uint8_t data_buffer_[UBLOX_DATA_BUFFER_LENGTH];


size_t bytes_processed = 0, payload_length = 0;
uint8_t id, checksum_a, checksum_b;
uint8_t * data_buffer_ptr = NULL;

struct UBXPosLLH ubx_pos_llh_;
struct UBXVelNED ubx_vel_ned_;
struct UBXSol ubx_sol_;
struct UBXTimeUTC ubx_time_utc_;

struct UBXPayload ubx_payload_;

void ProcessIncomingUBloxByte(uint8_t byte)
{
    switch (bytes_processed)
    {
        case 0:  // Sync char 1
            if (byte != UBX_SYNC_CHAR_1) goto RESET;
            break;
        case 1:  // Sync char 2
            if (byte != UBX_SYNC_CHAR_2) goto RESET;
            break;
        case 2:  // Class (NAV)
            if (byte != UBX_CLASS_NAV) goto RESET;
            checksum_a = byte;
            checksum_b = byte;
            break;
        case 3:  // ID
            id = byte;
            /*
             #define UBX_ID_POS_LLH (0x02)
             #define UBX_ID_VEL_NED (0x12)
             #define UBX_ID_SOL (0x06)
             #define UBX_ID_TIME_UTC (0x21)
             */
            if(id == UBX_ID_POS_LLH)printf(" UBX_ID_POS_LLH\n");
            else if(id == UBX_ID_VEL_NED)printf(" UBX_ID_VEL_NED\n");
            else if(id == UBX_ID_SOL)printf(" UBX_ID_SOL\n");
            else if(id == UBX_ID_TIME_UTC)printf(" UBX_ID_TIME_UTC\n");
            else printf(" UNKNOWN_ID %x\n", id);

            break;
        case 4:  // Payload length (lower byte)
            if (byte > UBLOX_DATA_BUFFER_LENGTH) goto RESET;
            payload_length = byte;
            data_buffer_ptr = &data_buffer_[0];
        case 5:  // Payload length (upper byte should always be zero)
            break;
        default:  // Payload or checksum
            if (bytes_processed < (6 + payload_length))  // Payload
            {
                *data_buffer_ptr++ = byte;
            }
            else if (bytes_processed == (6 + payload_length))  // Checksum A
            {

            }
            else  // Checksum B
            {
                CopyUBloxMessage(id);
                goto RESET;
            }
            break;
    }
    bytes_processed++;
    return;

RESET:
    bytes_processed = 0;
}





void run(void)
{
    int fd = open("/dev/gps_fifo", O_RDONLY | O_NONBLOCK);

    for(;;)
    {
        struct pollfd temp;
        temp.fd = fd;
        temp.events = POLLIN;
        poll(&temp, POLLIN, -1);
        unsigned char c;
        int r = (int) read(fd, &c, 1);
        if(r)ProcessIncomingUBloxByte(c);

    }
}

void CopyUBloxMessage(uint8_t id)
{
    // TODO: do this in a more efficient way
    switch (id)
    {
        case UBX_ID_POS_LLH:

            memcpy(&ubx_pos_llh_, &data_buffer_[0], sizeof(struct UBXPosLLH));
            status_ = ubx_pos_llh_.horizontal_accuracy < 5000;
            new_data_bits_ |= UBX_NEW_DATA_BIT_POS_LLH;
            printf("+----------------------------+\n| LONGITUDE:[%d], LATITUDE:[%d] | \n+----------------------------+\n",  ubx_pos_llh_.longitude, ubx_pos_llh_.latitude);
            //UpdatePositionToFlightCtrl(UBLOX);
            break;
        case UBX_ID_VEL_NED:
            memcpy(&ubx_vel_ned_, &data_buffer_[0], sizeof(struct UBXVelNED));
            new_data_bits_ |= UBX_NEW_DATA_BIT_VEL_NED;
            //UpdateVelocityToFlightCtrl(UBLOX);
            break;
        case UBX_ID_SOL:
            memcpy(&ubx_sol_, &data_buffer_[0], sizeof(struct UBXSol));
            new_data_bits_ |= UBX_NEW_DATA_BIT_SOL;
            printf(" ***************\n GPS_fix_type [%d] \n ***************\n", ubx_sol_.gps_fix_type);
            break;
        case UBX_ID_TIME_UTC:
            memcpy(&ubx_time_utc_, &data_buffer_[0], sizeof(struct UBXTimeUTC));
            new_data_bits_ |= UBX_NEW_DATA_BIT_TIME_UTC;
            break;
    }

    //last_reception_timestamp_ = GetTimestamp();
    //error_bits_ &= ~UBX_ERROR_BIT_STALE;
}


const struct UBXPayload * UBXPayload(void)
{
    ubx_payload_.longitude = ubx_pos_llh_.longitude;
    ubx_payload_.latitude = ubx_pos_llh_.latitude;
    ubx_payload_.z = ubx_pos_llh_.height_mean_sea_level;
    ubx_payload_.velocity[0] = float(ubx_vel_ned_.velocity_north)/100.0f;
    ubx_payload_.velocity[1] = float(ubx_vel_ned_.velocity_east)/100.0f;
    ubx_payload_.velocity[2] = float(ubx_vel_ned_.velocity_down)/100.0f;
    ubx_payload_.gps_status = 3;
    return &ubx_payload_;
}


uint8_t UBXNewDataAvailable(void)
{
    return new_data_bits_& UBX_NEW_DATA_BIT_POS_LLH;
}

// -----------------------------------------------------------------------------
void ClearUBXNewDataFlags(void)
{
    new_data_bits_ &= ~UBX_NEW_DATA_BIT_POS_LLH;
    new_data_bits_ &= ~UBX_NEW_DATA_BIT_VEL_NED;
}
