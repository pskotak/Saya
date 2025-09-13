#include <iostream>
#include <fstream>
#include <zstd.h>
#include "logger.h"
#include "gps.h"
//#include "d455.h"

//#define LogVerbatin
namespace logger {

bool LogEnabled = true;
bool Logging = false;

#define FnLen 256
char fname[FnLen];
static std::ofstream logfile;
static uint32_t RecCnt = 0;

//cv::Mat logrgb((D455H),(D455W),CV_8UC3);
//cv::Mat logdepth((D455H),(D455W),CV_16UC1);

// ZSTD compression level (negative values prioritize speed)
const int ZSTD_COMPRESSION_LEVEL = -5; // A good starting point for speed

void StartLog() {
    auto currtime = std::chrono::system_clock::now();
    std::time_t cctime = std::chrono::system_clock::to_time_t(currtime);
    std::string fstr;

    if (logfile.is_open()) {
        logfile.close();
    }

    bool filecreateok = false;
    if (LogEnabled) {
        fstr.assign("/home/owner/Develop/logy/");

        if (std::strftime(fname, sizeof(fname), "LOG_%Y%m%d_%H%M%S.sayal2", std::localtime(&cctime))) {
            filecreateok = true;
        }

        if (filecreateok) {
            fstr.append(fname);
#ifdef LogVerbatin
            std::cout << ">> Log: " << fstr << std::endl;
#endif
            logfile.open (fstr, std::ios::out | std::ios::binary);
            RecCnt = 0;
            Logging = true;
        }
        else {
#ifdef LogVerbatin
            std::cout << ">> Problem creating log file." << std::endl;
#endif
            Logging = false;
        }

    }
    else {
#ifdef LogVerbatin
        std::cout << ">> LOGGING DISABLED !!!" << std::endl;
#endif
    }
}

void EndLog() {
    if (logfile.is_open()) {
        logfile.close();
#ifdef LogVerbatin
        std::cout << ">> End Log." << std::endl;
#endif
    }
    Logging = false;
}


// typedef struct {
//  bool Valid; // Data valid - RMC
//  float UTCTime; // UTC time - RMC
//  uint32_t UTCDate; // UTC date - RMC
//  int32_t Lat; // Lat North positive - RMC, format v desetinach stupne * 1E7
//  int32_t Lon; // Lon East positive - RMC, format v desetinach stupne * 1E7
//  float Hdg; // Heading - RMC, format v setinach stupne
//  float Alt; // Altitude - GGA, v metrech
//  float GroundSpeed; // Ground speed - RMC, v m/s
//  bool Fix; // Fix - GGA
//  uint8_t SatUsed; // SatUsed (pocet satelitu uzitych pro fix) - GGA
//  float HDOP; // HDOP - GGA
// } TGPSData;

// typedef struct {
//     uint32_t RecNo; // Record number
//     struct rs2_pose Pose;
//     float setVelocity;
//     float setAngularVelocity;
//     float omegaL;
//     float omegaR;
//     int64_t Timestamp; // std::chrono::now in milliseconds
//     float UTCTime;
//     uint32_t UTCDate;
//     int32_t Lat;
//     int32_t Lon;
//     float Brg;
//     bool Fix;
// } TLogRecord;
TLogRecord LogRec;
unsigned char* rgbData;
unsigned char* depthData ;

void WriteLogRec() {
    auto now = std::chrono::system_clock::now();
    auto duration_since_epoch = now.time_since_epoch();
    auto milliseconds_duration = std::chrono::duration_cast<std::chrono::milliseconds>(duration_since_epoch);
    int64_t milliseconds_count = milliseconds_duration.count();

    // Compress
    // const unsigned char* rgbData = DispRGB_image.data;
    // const unsigned char* depthData = reinterpret_cast<const unsigned char*>(depth_image.data); // Cast depth data
    size_t rgbCompressedBound = ZSTD_compressBound(LogRGBBufSize);
    size_t depthCompressedBound = ZSTD_compressBound(LogDepthBufSize);
    std::vector<char> rgbCompressedBuffer(rgbCompressedBound);
    std::vector<char> depthCompressedBuffer(depthCompressedBound);
    size_t rgbCompressedSize = ZSTD_compress(rgbCompressedBuffer.data(), rgbCompressedBound,rgbData, LogRGBBufSize, ZSTD_COMPRESSION_LEVEL);
    size_t depthCompressedSize = ZSTD_compress(depthCompressedBuffer.data(), depthCompressedBound,depthData, LogDepthBufSize, ZSTD_COMPRESSION_LEVEL);

    LogRec.RecNo = RecCnt;
    // LogRec.Pose = platformPose;
    // LogRec.setVelocity = setVelocity;
    // LogRec.setAngularVelocity = setAngularVelocity;
    // LogRec.omegaL = omegaL;
    // LogRec.omegaR = omegaR;
    LogRec.Timestamp = milliseconds_count;
    // LogRec.UTCTime = GPSData.UTCTime; // UTC time - RMC
    // LogRec.UTCDate = GPSData.UTCDate;
    // LogRec.Lat = GPSData.Lat;
    // LogRec.Lon = GPSData.Lon;
    // LogRec.Brg = GPSData.Hdg;
    // LogRec.Fix = GPSData.Fix;
    // // std::memcpy(LogRec.RGBData,DispRGB_image.data,DispRGB_image.dataend - DispRGB_image.datastart);
    // // std::memcpy(LogRec.DepthData,depth_image.data,depth_image.dataend - depth_image.datastart);

    logfile.write((const char*) &LogRec, sizeof(LogRec));
    // Write RGB metadata and data
    logfile.write(reinterpret_cast<const char*>(&rgbCompressedSize), sizeof(rgbCompressedSize));
    logfile.write(rgbCompressedBuffer.data(), rgbCompressedSize);
    // Write Depth metadata and data
    logfile.write(reinterpret_cast<const char*>(&depthCompressedSize), sizeof(depthCompressedSize));
    logfile.write(depthCompressedBuffer.data(), depthCompressedSize);

    RecCnt++;
}

} // end namespace
