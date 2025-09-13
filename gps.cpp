// GPS.C - GPS NMEA driver - UART1 @ 9600 Bd nebo 34800 Bd
#include <iostream>
#include <unistd.h> // NUTNE kvuji write, close...
#include <fcntl.h> // File control definitions
#include <errno.h>
#include <termios.h>
#include <cstring>
#include <stdlib.h>

#include <string.h>
//#include <math.h>
#include "config.h"
#include "gps.h"
#include "SayaGlobals/globals.h"

std::atomic<bool> GPS_OK;
std::atomic<bool> GPS_Shutdown;
std::atomic<bool> NewGPS;

/*
// Data sloucena ze zprav GGA a RMC
typedef struct {
 uint8_t Valid; // Data valid - RMC
 float UTCTime; // UTC time - RMC
 uint32_t UTCDate; // UTC date - RMC
 long Lat; // Lat North positive - RMC, format v desetinach stupne * 1E7
 long Lon; // Lon East positive - RMC, format v desetinach stupne * 1E7
 float Hdg; // Heading - RMC, format v setinach stupne
 float Alt; // Altitude - GGA, v metrech
 float GroundSpeed; // Ground speed - RMC, v m/s
 uint8_t Fix; // Fix - GGA
 uint8_t SatUsed; // SatUsed (pocet satelitu uzitych pro fix) - GGA
 float HDOP; // HDOP - GGA
} TGPSData;
*/

// Konvertovat NMEA (deg * 100 + min) na desetinne stupne DD.DDDDDDD
/*
 D = NMEA / 100.0 -> cele stupne
 M = NMEA - (D * 100.0) -> desetinne minuty
 DD = D + (M / 60.0);
*/

// 1 m/s =cca 1,94384 kt

#define Separator ','
#define ChkStart '*'
#define CharLF 0x0A

// NMEA state machine - filtruje pouze zpravy GGA a RMC
typedef enum {
 stDolar, stG, stP, stName, stGGA, stRMC, stNMEA_CS
} TNMEAState;
TNMEAState NMEA_St = stDolar;
// $GPGGA,161229.487,3723.2475,N,12158.3416,W,1,07,1.0,9.0,M, , , ,0000*18
// $GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10
uint8_t NMEA_Name[3];
uint8_t Idx;
#define NMEAMsgLen 256
uint8_t MsgRMC[NMEAMsgLen];
uint8_t MsgGGA[NMEAMsgLen];

TGPSData GPSData;

char GPSPortName[20] = "/dev/SayaGPS";
int GPS_COM; // GPS_COM port handle

bool RqNewGGA;
bool RqNewRMC;
bool NewGGA;
bool NewRMC;

// ===========================================================================
#define RxBufSize 4096

int RunGPS() {
struct termios tty;
    TLB L;
    TFB FB;
    bool ComValid = false;
    uint8_t RxBuf[RxBufSize],B,C;
    int32_t RxLen, I;
    char *P1;
    uint8_t S[16];
    uint32_t Ignore,J;
    uint8_t S1[16];
    uint32_t LD,LM;
    float N;//,M;
    char BufRMC[NMEAMsgLen];

// ----------------------------------------------------------------------------
    if (!config.contains("GPS")) {
        config["GPS"]["GPS_Port"] = "/dev/SayaGPS"; // Vytvoril jsem si udev rule
        WriteConfig();
    }
    std::string s = config["GPS"]["GPS_Port"];
    //strcpy(GPSPortName,s.c_str());

    std::cout << "Starting GPS thread..." << std::endl;
    GPS_Shutdown = false;
    // Init GPS_COM port
    GPS_OK = false;

    GPS_COM = open(GPSPortName, O_RDWR | O_NOCTTY | O_NDELAY | O_SYNC);
    if (GPS_COM >= 0) {
        if (tcgetattr (GPS_COM, &tty) >= 0) {
            cfsetospeed(&tty, (speed_t) B115200);
            cfsetispeed(&tty, (speed_t) B115200);
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

            tcflush (GPS_COM, TCIFLUSH);
            if (!(tcsetattr (GPS_COM, TCSANOW, &tty))) {
                GPS_OK = true;
                NewGPS = false;
                ComValid = true;
                std::cout << ">>> GPS port opened: " << GPSPortName << std::endl;
                std::cout << "GPS thread started." << std::endl << std::endl;
            }
        }
    }
    if (!ComValid) {
        std::cout << "No GPS GPS_COM port found." << std::endl << std::endl;
    }

    while (!GPS_Shutdown) {
        if (ComValid) {
            RxLen = read(GPS_COM,RxBuf,RxBufSize);

            if (RxLen > 0) {
                for (int q=0;q<RxLen;q++) {
                    C = RxBuf[q];
                    switch (NMEA_St) {
                        case stDolar:
                            if (C == '$') {
                                RqNewGGA = false;
                                RqNewRMC = false;
                                NMEA_St = stG;
                            }
                            break;

                        case stG:
                            if (C == 'G')
                                NMEA_St = stP;
                            else
                                NMEA_St = stDolar;
                            break;

                        case stP:
    #ifdef GPS_GN
                            if (C == 'N') // GNGGA & GNRMC
    #else
                            if (C == 'P') // GPGGA & GPRMC
    #endif
                            {
                                NMEA_St = stName;
                                Idx = 0;
                            }
                            else
                                NMEA_St = stDolar;
                            break;

                        case stName:
                            if (C == Separator) {
                                Idx = 0;
                                if ((NMEA_Name[0] == 'G') && (NMEA_Name[1] == 'G') && (NMEA_Name[2] == 'A')) {
                                    NMEA_St = stGGA;
                                }
                                else if ((NMEA_Name[0] == 'R') && (NMEA_Name[1] == 'M') && (NMEA_Name[2] == 'C')) {
                                    NMEA_St = stRMC;
                                }
                                else {
                                    NMEA_St = stDolar;
                                }
                            }
                            else {
                                NMEA_Name[Idx] = C;
                                if (Idx < 3) Idx++;
                            }
                            break;

                        case stGGA:
                            if (NewGGA) {
                                NMEA_St = stDolar; // Neni dokonceno parsovani - zahodime zpravu, abychom neprepsali buffer
                            }
                            else {
                                MsgGGA[Idx] = C;
                                if (Idx < NMEAMsgLen) {
                                    Idx++;
                                }
                                else {
                                    NMEA_St = stDolar; // Prilis dlouha zprava
                                }
                                if (C == ChkStart) {
                                    NMEA_St = stNMEA_CS;
                                    RqNewGGA = true;
                                }
                            }
                            break;

                        case stRMC:
                            if (NewRMC) {
                                NMEA_St = stDolar; // Neni dokonceno parsovani - zahodime zpravu, abychom neprepsali buffer
                            }
                            else {
                                MsgRMC[Idx] = C;
                                if (Idx < NMEAMsgLen) {
                                    Idx++;
                                }
                                else {
                                    NMEA_St = stDolar; // Prilis dlouha zprava
                                }
                                if (C == ChkStart) {
                                    NMEA_St = stNMEA_CS;
                                    RqNewRMC = true;
                                }
                            }
                            break;

                        case stNMEA_CS: // Doplnit pocitani a kontrolu CS
                            if (RqNewGGA) {
                                RqNewGGA = false;
                                NewGGA = true;
                            }
                            if (RqNewRMC) {
                                RqNewRMC = false;
                                NewRMC = true;
                            }
                            NMEA_St = stDolar;
                            break;

                        default:
                            NMEA_St = stDolar;
                            break;
                    }

                    if ((NewRMC) && (NewGGA)) {
                        NewGGA = false;
                        NewRMC = false;

                        if (NewGPS == false) {
                            // Parsovani pouze pokud je uz shozeny flag z hlavni smycky

    //ParseGGA
    #define GGA_Ignore 5 // Zajima nas az sesty parametr
                            Ignore = GGA_Ignore;
                            for (I=0;I<NMEAMsgLen;I++) {
                                if (MsgGGA[I] == ',') {
                                    Ignore--;
                                    if (Ignore == 0)
                                    break;
                                }
                            }

                            I++; J = 0;
                            // I ukazuje na zacatek prvniho zajimaveho pole
                            do {
                                C = MsgGGA[I++];
                                if (C != ',')
                                    S[J++] = C;
                                else
                                    S[J] = 0;
                            } while (C != ',');
                            GPSData.Fix = (atoi((char *)S) != 0);

                            J = 0;
                            // I ukazuje na zacatek dalsiho zajimaveho pole
                            do {
                                C = MsgGGA[I++];
                                if (C != ',')
                                    S[J++] = C;
                                else
                                    S[J] = 0;
                            } while (C != ',');
                            GPSData.SatUsed = atoi((char *)S);

                            J = 0;
                            // I ukazuje na zacatek dalsiho zajimaveho pole
                            do {
                                C = MsgGGA[I++];
                                if (C != ',')
                                    S[J++] = C;
                                else
                                    S[J] = 0;
                            } while (C != ',');
                            GPSData.HDOP = atof((char *)S);

                            J = 0;
                            // I ukazuje na zacatek dalsiho zajimaveho pole
                            do {
                                C = MsgGGA[I++];
                                if (C != ',')
                                    S[J++] = C;
                                else
                                    S[J] = 0;
                            } while (C != ',');
                            GPSData.Alt = atof((char *)S);

                            /*
                            D = NMEA / 100.0 -> cele stupne
                            M = NMEA - (D * 100.0) -> desetinne minuty
                            DD = D + (M / 60.0);
                            */
    // ParseRMC
                            for (I=0;I<NMEAMsgLen;I++) {
                                BufRMC[I] = (char) MsgRMC[I];
                            }
                            P1 = strtok((char *)&BufRMC[0],",");
                            if (P1 != NULL) {
                                strcpy((char *)S,(char *)P1);
                                GPSData.UTCTime = atof((char *)S);
                                // Proc chodi sekundy v rozsahu 01 .. 60 ????
                                P1 = strtok((char *)NULL,(char *)",");
                                if (P1 != NULL) {
                                    strcpy((char *)S,(char *)P1);
                                    if (S[0] == 'A') {
                                        GPSData.Valid = true;
                                    }
                                    else {
                                        GPSData.Valid = false;
                                    }
                                }
                                if (GPSData.Valid) {
                                    P1 = strtok((char *)NULL,(char *)",");
                                    if (P1 != NULL) {
                                        // Pokus o zlepseni presnosti
                                        // Vezme se zvlast cela cast (stupne) z retezce, vynasobi se 1E7 a pote ulozi do long
                                        // Vezme se zvlast 2digity+zlomkova cast (minuty a desetiny minut) z retezce, vynasobi se 1E7 a pote ulozi do long
                                        // obe casi se celociselne sectou
                                        strcpy((char *)S,(char *)P1);
                                        S1[0] = S[0];
                                        S1[1] = S[1];
                                        S1[2] = 0;
                                        N = atof((char *)S1); // Degrees 2 digit fixed
                                        N = N * 1E7;
                                        LD = floor(N+0.5);
                                        N = atof((char*)(S+2)); // Minutes desetinne
                                        N = N / 60.0;
                                        N = N * 1E7;
                                        LM = floor(N+0.5);
                                        GPSData.Lat = LD+LM;
                                    }
                                    P1 = strtok((char *)NULL,(char *)",");
                                    if (P1 != NULL) {
                                        strcpy((char *)S,(char *)P1);
                                        if (S[0] == 'S') GPSData.Lat = GPSData.Lat * -1;
                                    }
                                    P1 = strtok((char *)NULL,(char *)",");
                                    if (P1 != NULL) {
                                        strcpy((char *)S,(char *)P1);
                                        S1[0] = S[0];
                                        S1[1] = S[1];
                                        S1[2] = S[2];
                                        S1[3] = 0;
                                        N = atof((char *)S1); // Degrees 3 digit fixed
                                        N = N * 1E7;
                                        LD = floor(N+0.5);
                                        N = atof((char*)(S+3)); // Minutes desetinne
                                        N = N / 60.0;
                                        N = N * 1E7;
                                        LM = floor(N+0.5);
                                        GPSData.Lon = LD+LM;
                                    }
                                    P1 = strtok((char *)NULL,(char *)",");
                                    if (P1 != NULL) {
                                        strcpy((char *)S,(char *)P1);
                                        if (S[0] == 'W') GPSData.Lon = GPSData.Lon * -1;
                                    }
                                    P1 = strtok((char *)NULL,(char *)",");
                                    if (P1 != NULL) {
                                        strcpy((char *)S,(char *)P1);
                                        N = atof((char *)S); // 1 knot = 0.514444444 meters / second
                                        GPSData.GroundSpeed = N * 0.514444444; // kt -> m/s
                                    }
                                    P1 = strtok((char *)NULL,(char *)",");
                                    S[0] = *(P1-1);
                                    if (S[0] != ',') {
                                        if (P1 != NULL) {
                                            strcpy((char *)S,(char *)P1);
                                            GPSData.Hdg = atof((char *)S);
                                        }
                                        P1 = strtok((char *)NULL,(char *)",");
                                    }
                                    if (P1 != NULL) {
                                        strcpy((char *)S,(char *)P1);
                                        GPSData.UTCDate = atol((char *)S);
                                    }
                                }
                            }
                            NewGPS = true;
                        }
                    }

                    // std::cout << RxLen << ": ";
                    // for (int i=0;i<RxLen;i++) {
                    //     std::cout << RxBuf[i];
                    // }
                    // std::cout << std::endl;
                }
            }
            usleep(10000);
        }
        else {
            usleep(200000);
        }
    }
    if (ComValid)
        close(GPS_COM);
	std::cout << ">>> GPS port closed: " << GPSPortName << std::endl;
    std::cout << "GPS thread ended." << std::endl;

    return 0;
}
