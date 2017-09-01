//
//  helper.hpp
//  GPS_READER
//
//  Created by blue-i on 01/09/2017.
//  Copyright © 2017 blue-i. All rights reserved.
//

#ifndef helper_hpp
#define helper_hpp

#include <stdio.h>
#include "string.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>        //Used for UART
#include <sys/ioctl.h>      //Used for UART
#include <sys/stat.h>       //Used for fifo
#include <poll.h>           //Pollin()




#define UBLOX_INITIAL_BAUD B9600
#define UBLOX_OPERATING_BAUD B57600
#define GPS_PORT "/dev/ttyUSB_Ublox"



void UART_Init(int b);

void UBloxTxBuffer(const char b[], int t);

void UART_Close();

void GPS_Init(void);

bool file_exist(const std::string& name);

bool pollin(void);

void Reader_sender(void);

void run(void);

#endif /* helper_hpp */
