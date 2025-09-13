#ifndef LOGGER_H
#define LOGGER_H

// #include "globals.h"
#include "SayaGlobals/globals.h"

namespace logger {

typedef struct {
    uint32_t RecNo; // Record number
    struct rs2_pose Pose;
    float setVelocity;
    float setAngularVelocity;
    float omegaL;
    float omegaR;
    int64_t Timestamp; // std::chrono::now in milliseconds
    float UTCTime;
    uint32_t UTCDate;
    int32_t Lat;
    int32_t Lon;
    float Brg;
    bool Fix;
} TLogRecord;
extern TLogRecord LogRec;
extern unsigned char* rgbData;
extern unsigned char* depthData;

extern bool LogEnabled;
extern bool Logging;

extern void StartLog();
extern void EndLog();
extern void WriteLogRec();

} // end namespace
#endif
