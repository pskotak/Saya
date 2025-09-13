// gps.h
#ifndef GPS_H
#define GPS_H

// Neo M9N modul ELT0137 od Eltehs GNSS OEM Store
// Pripjeni pres USB virtual COM port (ttyACMx)

#include <atomic>
//#include "cvgui.h"

#define GPS_GN // Zarizeni posila GNGGA misto GPGGA atd.

// Data sloucena ze zprav GGA a RMC
typedef struct {
 bool Valid; // Data valid - RMC
 float UTCTime; // UTC time - RMC
 uint32_t UTCDate; // UTC date - RMC
 int32_t Lat; // Lat North positive - RMC, format v desetinach stupne * 1E7
 int32_t Lon; // Lon East positive - RMC, format v desetinach stupne * 1E7
 float Hdg; // Heading - RMC, format v setinach stupne
 float Alt; // Altitude - GGA, v metrech
 float GroundSpeed; // Ground speed - RMC, v m/s
 bool Fix; // Fix - GGA
 uint8_t SatUsed; // SatUsed (pocet satelitu uzitych pro fix) - GGA
 float HDOP; // HDOP - GGA
} TGPSData;

extern TGPSData GPSData;

extern std::atomic<bool> GPS_OK;
extern std::atomic<bool> GPS_Shutdown;
extern std::atomic<bool> NewGPS;

extern int RunGPS();

#endif

