//
//  helper.cpp
//  GPS_READER
//
//  Created by blue-i on 01/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#include "helper.hpp"

int ublox_fd = -1;

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
    ublox_fd = open(GPS_PORT, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
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

        cfsetospeed (&tty, b);


    }



    cfsetispeed (&tty, b);


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
void UBloxTxBuffer(const char b[], int t)
{
    int wr =0 ;
    while(wr < t)
    {
        if((int) write(ublox_fd,&b[wr],1)) wr++;
        printf("\n printing [%x] index[%d] writen:%d", b[wr-1], wr-1, wr);
        if(wr == t || wr > t)
	{
		printf("\n what?");
		return;
	 }
    }

    std::cout<<"\n wr = %d"<< wr<<std::endl;
}

void UART_Close()
{
    close(ublox_fd);
}

void GPS_Init(void)
{
    UART_Init(UBLOX_INITIAL_BAUD);


    {
        printf("\n Set the port to UART UBX @ 57600.\n");
        const char tx_buffer[28] = { '\xb5', '\x62', '\x06', '\x00', '\x14', '\x00', '\x01',
            '\x00', '\x00', '\x00', '\xd0', '\x08', '\x00', '\x00', '\x00', '\xe1', '\x00', '\x00', '\x01',
            '\x00', '\x01', '\x00', '\x00', '\x00', '\x00', '\x00', '\xd6', '\x8d' };
        UBloxTxBuffer(tx_buffer, 28);
    }

    usleep(150000);
    UART_Close();
    UART_Init(UBLOX_OPERATING_BAUD);

    // Enable UART Rx interrupt.


    {   printf("\n Configure USB for UBX input with no output.\n");
        const char tx_buffer[28] = { '\xb5', '\x62', '\x06', '\x00', '\x14', '\x00', '\x03',
            '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x01',
            '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x1e', '\x8c' };
        UBloxTxBuffer(tx_buffer, 28);
    }
    {   printf("\n Set antenna flags to 0x0b and pins to 0x380f.\n");
        const char tx_buffer[12] = { '\xb5', '\x62', '\x06', '\x13', '\x04', '\x00', '\x0b',
            '\x00', '\x0f', '\x38', '\x6f', '\x4f' };
        UBloxTxBuffer(tx_buffer, 12);
    }
    {   printf("\n Set measurement period to 200ms (5Hz) with UTC reference.\n");
        const char tx_buffer[14] = { '\xb5', '\x62', '\x06', '\x08', '\x06', '\x00', '\xc8',
            '\x00', '\x01', '\x00', '\x00', '\x00', '\xdd', '\x68' };
        UBloxTxBuffer(tx_buffer, 14);
    }
    {   printf("\n Configure TimPulse.\n");
        const char tx_buffer[28] = { '\xb5', '\x62', '\x06', '\x07', '\x14', '\x00', '\x40',
            '\x42', '\x0f', '\x00', '\x90', '\x86', '\x03', '\x00', '\xff', '\x01', '\x00', '\x00', '\x32',
            '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\xfd', '\x70' };
        UBloxTxBuffer(tx_buffer, 28);
    }
    {   printf("\n Configure SBAS.\n");
        const char tx_buffer[16] = { '\xb5', '\x62', '\x06', '\x16', '\x08', '\x00', '\x03',
            '\x03', '\x01', '\x00', '\x00', '\x00', '\x00', '\x00', '\x2b', '\xbd' };
        UBloxTxBuffer(tx_buffer, 16);
    }
    {   printf("\n Configure navigation engine.\n");
        const char tx_buffer[44] = { '\xb5', '\x62', '\x06', '\x24', '\x24', '\x00', '\xff',
            '\xff', '\x06', '\x02', '\x00', '\x00', '\x00', '\x00', '\x10', '\x27', '\x00', '\x00', '\x08',
            '\x3c', '\x50', '\x00', '\x32', '\x00', '\x23', '\x00', '\x23', '\x00', '\x00', '\x00', '\x00',
            '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x97',
            '\xfa' };
        UBloxTxBuffer(tx_buffer, 44);
    }
    {   printf("\n Configure navigation engine expert settings.\n");
        const char tx_buffer[48] = { '\xb5', '\x62', '\x06', '\x23', '\x28', '\x00', '\x00',
            '\x00', '\x4c', '\x06', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x04', '\x10', '\x14',
            '\x00', '\x01', '\x00', '\x00', '\x00', '\xf8', '\x05', '\x00', '\x00', '\x00', '\x00', '\x00',
            '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00', '\x00',
            '\x00', '\x00', '\x00', '\xc9', '\xea' };
        UBloxTxBuffer(tx_buffer, 48);
    }
    {   printf("\n Request NAV-POSLLH message to be output every measurement cycle.\n");
        const char tx_buffer[11] = { '\xb5', '\x62', '\x06', '\x01', '\x03', '\x00', '\x01',
            '\x02', '\x01', '\x0e', '\x47' };
        UBloxTxBuffer(tx_buffer, 11);
    }
    {   printf("\n Request NAV-VELNED message to be output every measurement cycle.\n");
        const char tx_buffer[11] = { '\xb5', '\x62', '\x06', '\x01', '\x03', '\x00', '\x01',
            '\x12', '\x01', '\x1e', '\x67' };
        UBloxTxBuffer(tx_buffer, 11);
    }
    {   printf("\n Request NAV-SOL message to be output every measurement cycle.\n");
        const char tx_buffer[11] = { '\xb5', '\x62', '\x06', '\x01', '\x03', '\x00', '\x01',
            '\x06', '\x01', '\x12', '\x4f' };
        UBloxTxBuffer(tx_buffer, 11);
    }
    {   printf("\n Request Time-UTC message to be output every 5 measurement cycles.\n");
        const char tx_buffer[11] = { '\xb5', '\x62', '\x06', '\x01', '\x03', '\x00', '\x01',
            '\x21', '\x05', '\x31', '\x89' };
        UBloxTxBuffer(tx_buffer, 11);
    }

}

bool file_exist(const std::string& name)
{
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

bool pollin(void)
{
    bool retval = false ;

    struct pollfd temp_poll;
    temp_poll.fd = ublox_fd;
    temp_poll.events = POLLIN;

    poll(&temp_poll, 1, -1);



    if(temp_poll.revents & POLLIN) retval = true;

    return retval;

}
void Reader_sender(void)
{
    std::string name = "/dev/gps_fifo";
    if(!file_exist(name))
	{
	int i = mkfifo(name.c_str(), 0666);// S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    	std::cout<<"\n fifo res : "<<i<<std::endl;
	}
    int flag = O_WRONLY | O_NONBLOCK;//debug
    int dum = open(name.c_str(), O_RDONLY | O_NONBLOCK);
    int fifo_fd = open(name.c_str(), flag);
    std::cout<<"\n fifo fd "<<fifo_fd<<" dum" <<dum<<std::endl;
    for(;;)
    {
        pollin();
        int ioctl_bytes = 0;
        ioctl(ublox_fd, FIONREAD, &ioctl_bytes);
        std::cout<<" bytes : %d"<< ioctl_bytes<<std::endl;
        int r_bytes = 0;
        while(r_bytes != ioctl_bytes)
        {
            unsigned char c;
            if(read(ublox_fd, &c, 1)){
              r_bytes++;
              write(fifo_fd, &c, 1);
            }
//            std::cout<<"\n Read & Write one byte";
        }
    }

}

void run(void)
{
    GPS_Init();
    Reader_sender();
}
