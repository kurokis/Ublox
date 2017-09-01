//#include "../../libraries/helper/helper.hpp"

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

static void CopyUBloxMessage(uint8_t id);
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

static struct UBXPosLLH ubx_pos_llh_;
static struct UBXVelNED ubx_vel_ned_;
static struct UBXSol ubx_sol_;
static struct UBXTimeUTC ubx_time_utc_;

static enum UBXErrorBits error_bits_ = UBX_ERROR_BIT_STALE;
static uint32_t status_ = 0;
static uint32_t new_data_bits_ = 0;
static uint32_t last_reception_timestamp_ = 0;

#define UBLOX_DATA_BUFFER_LENGTH (sizeof(struct UBXSol))

static uint8_t data_buffer_[UBLOX_DATA_BUFFER_LENGTH];


int ublox_fd = -1;
static void ProcessIncomingUBloxByte(uint8_t byte)
{
    static size_t bytes_processed = 0, payload_length = 0;
    static uint8_t id, checksum_a, checksum_b;
    static uint8_t * data_buffer_ptr = NULL;
    printf("\n bytes processed [%zu]", bytes_processed);

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
            if(id == UBX_ID_POS_LLH)printf("\n UBX_ID_POS_LLH");
                else if(id == UBX_ID_VEL_NED)printf("\n UBX_ID_VEL_NED");
                    else if(id == UBX_ID_SOL)printf("\n UBX_ID_SOL");
                        else if(id == UBX_ID_TIME_UTC)printf("\n UBX_ID_TIME_UTC");
                            else printf("\n UNKNOWN_ID %x", id);

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

void UBloxTxBuffer(const char b[], int t)
{
    int wr =0 ;
    while(wr != t)
    {
        if((int) write(ublox_fd,&b[wr],1)) wr++;
        printf("\n printing [%x] index[%d] writen:%d", b[wr-1], wr-1, wr);
        if(wr == t || wr > t) break;
    }

    printf("\n wr = %d", wr);
}

void UART_Init(int b)
{
    //-------------------------
    //----- SETUP USART 0 -----
    //-------------------------
    //At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
    ublox_fd = -1;
    close(ublox_fd);
    //OPEN THE UART
    //The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //
    //	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //											immediately with a failure status if the output can't be written immediately.
    //
    //	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
    ublox_fd = open("/dev/cu.usbserial-DJ00LQ19", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
    if (ublox_fd == -1)
    {
        //ERROR - CAN'T OPEN SERIAL PORT
        printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }

    //CONFIGURE THE UART
    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE:- CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)
    struct termios tty;

    memset (&tty, 0, sizeof tty);
    if (tcgetattr (ublox_fd, &tty) != 0)
    {
        if(b == 57600)
        {
              cfsetospeed (&tty, B57600);
            printf("57600 if");
        }
        else if(b == 9600)
        {
           cfsetospeed (&tty, B9600);
            printf("9600 if");
        }
    }


    if(b == 57600){
         printf("\n57600 ti");
        cfsetispeed (&tty, B57600);
    }
    if(b == 9600)
    {
        cfsetispeed (&tty, B9600);
 printf("\n9600 ti");

    }

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars

    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,

    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls

    // enable reading
    tty.c_cflag &= ~PARENB;      // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;


    tcsetattr (ublox_fd, TCSANOW, &tty);
    printf("\n Open status : %d", ublox_fd);



}

void UART_Close()
{
    close(ublox_fd);
}

int main(int argc, char const *argv[])
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
  return 0;
}

static void CopyUBloxMessage(uint8_t id)
{
    // TODO: do this in a more efficient way
    switch (id)
    {
        case UBX_ID_POS_LLH:
            memcpy(&ubx_pos_llh_, &data_buffer_[0], sizeof(struct UBXPosLLH));
            status_ = ubx_pos_llh_.horizontal_accuracy < 5000;
            new_data_bits_ |= UBX_NEW_DATA_BIT_POS_LLH;
            printf("\n+----------------------------+\n| LONGITUDE:[%d], LATITUDE:[%d] | \n+----------------------------+",  ubx_pos_llh_.longitude, ubx_pos_llh_.latitude);
            //UpdatePositionToFlightCtrl(UBLOX);
#ifdef LOG_DEBUG_TO_SD
            // LogUBXPosLLH();
#endif
            break;
        case UBX_ID_VEL_NED:
            memcpy(&ubx_vel_ned_, &data_buffer_[0], sizeof(struct UBXVelNED));
            new_data_bits_ |= UBX_NEW_DATA_BIT_VEL_NED;
            //UpdateVelocityToFlightCtrl(UBLOX);
#ifdef LOG_DEBUG_TO_SD
            // LogUBXVelNED();
#endif
            break;
        case UBX_ID_SOL:
            memcpy(&ubx_sol_, &data_buffer_[0], sizeof(struct UBXSol));
            new_data_bits_ |= UBX_NEW_DATA_BIT_SOL;
            printf("\n ***************\n GPS_fix_type [%d] \n ***************", ubx_sol_.gps_fix_type);
#ifdef LOG_DEBUG_TO_SD
            // LogUBXSol();
#endif
            break;
        case UBX_ID_TIME_UTC:
            memcpy(&ubx_time_utc_, &data_buffer_[0], sizeof(struct UBXTimeUTC));
            new_data_bits_ |= UBX_NEW_DATA_BIT_TIME_UTC;
#ifdef LOG_DEBUG_TO_SD
            // LogUBXTimeUTC();
#endif
            break;
    }

    //last_reception_timestamp_ = GetTimestamp();
    //error_bits_ &= ~UBX_ERROR_BIT_STALE;
}
