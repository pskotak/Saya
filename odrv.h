#ifndef ODRV_H
#define ODRV_H

#include <libusb-1.0/libusb.h>
#include "SayaGlobals/globals.h"

namespace ODRV {

// Presunuto do globals.h
// #define ODRV_MFGR_ID 0x1209
// #define ODRV_DEV_ID 0x0D32

#define EP_ID 3

class odrv {
    unsigned char rqSN[8]      = {0xD6,0x03,0x05,0x80,0x08,0x00,0x71,0xD2};
    unsigned char rqVBus[8]    = {0xD7,0x03,0x02,0x80,0x04,0x00,0x71,0xD2};
    unsigned char rqEnc0[8]    = {0xd8,0x03,0x37,0x81,0x04,0x00,0x71,0xd2};
    unsigned char rqEnc1[8]    = {0xd9,0x03,0x68,0x82,0x04,0x00,0x71,0xd2};
    unsigned char rqVel0[12]   = {0xda,0x03,0xfd,0x80,0x04,0x00, 0x00,0x00,0x00,0x00, 0x71,0xd2}; // Velocity float idx 6..9
    unsigned char rqVel1[12]   = {0xdb,0x03,0x2e,0x82,0x04,0x00, 0x00,0x00,0x00,0x00, 0x71,0xd2}; // Velocity float idx 6..9
    unsigned char rqPGain0[12] = {0xdc,0x03,0x0e,0x81,0x04,0x00, 0xdd,0x24,0x46,0x3f, 0x71,0xd2}; // Gain float idx 6..9
    unsigned char rqPGain1[12] = {0xde,0x03,0x3f,0x82,0x04,0x00, 0xdd,0x24,0x46,0x3f, 0x71,0xd2}; // Gain float idx 6..9
    unsigned char rqArm0[9]    = {0xdf,0x03,0x87,0x80,0x01,0x00,0x08,0x71,0xd2};
    unsigned char rqArm1[9]    = {0xe0,0x03,0xb8,0x81,0x01,0x00,0x08,0x71,0xd2};
    unsigned char rqIdle0[9]   = {0xe1,0x03,0x87,0x80,0x01,0x00,0x01,0x71,0xd2};
    unsigned char rqIdle1[9]   = {0xe2,0x03,0xb8,0x81,0x01,0x00,0x01,0x71,0xd2};
    //unsigned char rqEnc0[] = {};

    int OdrvSeqNo = 1; // TODO Naimplementovat Sequence number do odesilanych paketu a pripadnou kontrolu u vracenych
    #define OdrvRxBufLen 256
    unsigned char OdrvRxBuf[OdrvRxBufLen];
    int RdNum = 0; // Actual number if bytes read

    bool USB_Opened = false;
    struct libusb_device_handle *dh = NULL;

    //typedef enum {stInit, stGetSN, stGetVBus, stEnd, stWaitRq} TOdrvState;
    typedef enum {
        stRqSN,stHaSN,
        stVBus,stEnc0,stEnc1,stVel0,stVel1,stPGain0,stPGain1,stArm0,stArm1,stIdle0,stIdle1,
        stEnd,stWaitRq,stWaitHa
    } TOdrvState;
    TOdrvState OdrvState = stRqSN;
    TOdrvState NextOdrvState = stHaSN;
    bool RqDone = false;
    bool ResDone = false;

public:
	bool InitFailed = true;

    float WheelRadius_m = 0.15;
    float LinVMax_m_s = 2.5;
    float AngularVMax_rad_s = 16.0;
    float MaxVel_rps = 100.0; // Max Wheel Velocity in revolution per second

    uint64_t OdrvSN = 0;
    int32_t Enc0 = 0;
    int32_t Enc1 = 0;
    float Vel0 = 0.0;
    float Vel1 = 0.0;
    float PGain = 0.774;
    bool Finish = false; // Ukonci USB komunikaci s ODrive deskou
    bool Finished = false;
    bool Armed = false;
    float VBus = 0.0;
    float Speed = 0.0;
    float Dir = 0.0;
    // Setting triggers
    bool SetArm = false;
    bool SetIdle = false;
    bool SetGain = false;

    odrv();
    ~odrv();
    void Shutdown();
    void Update();
    void Run(const float inSpeed, const float inDir);
    void Motors(const float in_omegaL, const float in_omegaR);
    void Stop();
};

} // end namespace

#endif
