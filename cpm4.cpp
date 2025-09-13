#include <iostream>
#include <cstdint>
#include <unistd.h> // NUTNE kvuji write, close...
#include <fcntl.h> // File control definitions
#include <errno.h>
#include <termios.h>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>

#include "cpm4.h"

std::atomic<bool> BigRedOn(false);
std::atomic<bool> BigRedOff(false);
std::atomic<bool> MissionOn(false);
std::atomic<bool> MissionOff(false);

std::atomic<bool> SerialOK;
std::atomic<bool> SerialShutdown;

bool NewCOMData;
char CPM4PortName[20] = "/dev/SayaCPM4";
int COM; // COM port handle
#define RxBufSize 4096 //1023
//uint32_t ComLen;

//std::unique_lock<std::mutex> ComLock(ComMutex, std::defer_lock);

// ============================================================================
int CheckCom() {
    COM = open(CPM4PortName, O_RDWR | O_NOCTTY | O_NDELAY | O_SYNC);
    if (COM < 0) {
        SerialOK = false;
        return -1;
    }
    else {
        close(COM);
        return 0;
    }
}

int RunCom(void) {
    struct termios tty;

    SerialOK = false;

    COM = open(CPM4PortName, O_RDWR | O_NOCTTY | O_NDELAY | O_SYNC);
    if (COM < 0) {
        std::cout << ">>> CPM4 port not found." << std::endl;
        return -1;
    }

    if (tcgetattr (COM, &tty) < 0)
        return -2;

    cfsetospeed(&tty, (speed_t) B115200);
    cfsetispeed(&tty, (speed_t) B115200);
    // cfsetospeed(&tty, (speed_t) B9600);
    // cfsetispeed(&tty, (speed_t) B9600);
    tty.c_cflag &= ~PARENB; // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS; // no flow control
    tty.c_cc[VMIN] = 1; // read doesn't block
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines

    // Make raw
    cfmakeraw(&tty);

    tcflush (COM, TCIFLUSH);
    if (tcsetattr (COM, TCSANOW, &tty))
        return -3;

    std::cout << ">>> CPM4 port opened: " << CPM4PortName << std::endl;
    //std::cout << "Starting CPM4 thread..." << std::endl;

    SerialOK = true;
    NewCOMData = false;
    SerialShutdown = false;

    while (!SerialShutdown) {
        unsigned char RxBuf[RxBufSize],B;
        int32_t RxLen, I;

        memset(RxBuf,0,sizeof(RxBuf));
        RxLen = read(COM,RxBuf,RxBufSize);

        if (RxLen > 0) {
            //std::cout << RxBuf << std::endl;
            if (strcmp((const char *) &RxBuf,"RP\n") == 0) {
                BigRedOn = true;
            }
            else if (strcmp((const char *) &RxBuf,"RR\n") == 0) {
                BigRedOff = true;
            }
            else if (strcmp((const char *) &RxBuf,"MP\n") == 0) {
                MissionOn = true;
            }
            else if (strcmp((const char *) &RxBuf,"MR\n") == 0) {
                MissionOff = true;
            }
        }

        usleep(5000);
    }
    close(COM);
    std::cout << ">>> CPM4 port closed: " << CPM4PortName << std::endl;
    std::cout << "CPM4 thread ended." << std::endl;

    return 0;
}
