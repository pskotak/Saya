#include <iostream>
#include "odrv.h"

#define ODRV_MFGR_ID 0x1209
#define ODRV_DEV_ID 0x0D32

namespace ODRV {

odrv::odrv() {
    int rs;

    std::cout << ">>> Starting Odrive ..." << std::endl;
    InitFailed = true;
	rs = libusb_init(NULL);
    if (rs < 0) {
        std::cout << ">>> LibUSB init error." << std::endl;
    }
    dh = libusb_open_device_with_vid_pid(NULL,ODRV_MFGR_ID,ODRV_DEV_ID);
    if (!dh) {
        std::cout << ">>> Cannot open Odrive USB device!" << std::endl;
    }
    else {
        USB_Opened = true;
    }

    if (USB_Opened) {
        rs = libusb_claim_interface(dh,2);
        if (rs < 0) {
            std::cout << ">>> Cannot claim USB interface!" << std::endl;
            return;
        }
    }
    OdrvState = stRqSN;
    RqDone = false;
    ResDone = false;
    Finish = false;
    Finished = false;
    if (USB_Opened) {
        InitFailed = false;
    }
}

odrv::~odrv() {
    libusb_exit(NULL);
}

void odrv::Shutdown() {
    if (USB_Opened) {
        libusb_release_interface(dh,0);
        libusb_close(dh);
    }
}

void odrv::Update() {
    int rs,actual,RxNum;
    TFB R;
    TSLB SL;
    struct timeval tv;
    tv.tv_sec = 0; tv.tv_usec = 0;

    if (USB_Opened) {
        switch(OdrvState) {
            case stRqSN:
                OdrvState = stVBus;
                //libusb_submit_transfer(trSN);
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_OUT), rqSN, 8, &actual, 0);
                //std::cout << "Write: Result=" << r << " Written=" << actual << std::endl;
                RxNum = 0;
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_IN), OdrvRxBuf, OdrvRxBufLen, &RxNum, 0);

                OdrvSN = OdrvRxBuf[7]; OdrvSN = OdrvSN << 8;
                OdrvSN |= OdrvRxBuf[6]; OdrvSN = OdrvSN << 8;
                OdrvSN |= OdrvRxBuf[5]; OdrvSN = OdrvSN << 8;
                OdrvSN |= OdrvRxBuf[4]; OdrvSN = OdrvSN << 8;
                OdrvSN |= OdrvRxBuf[3]; OdrvSN = OdrvSN << 8;
                OdrvSN |= OdrvRxBuf[2];
                std::cout << "Odrive SN: " << std::hex << OdrvSN << std::dec << std::endl;

                break;

            case stVBus:
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_OUT), rqVBus, 8, &actual, 0);
                RxNum = 0;
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_IN), OdrvRxBuf, OdrvRxBufLen, &RxNum, 0);

                R.B[0] = OdrvRxBuf[2]; R.B[1] = OdrvRxBuf[3]; R.B[2] = OdrvRxBuf[4]; R.B[3] = OdrvRxBuf[5];
                VBus = R.F;

                OdrvState = stEnc0;
                break;

            case stEnc0:
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_OUT), rqEnc0, 8, &actual, 0);
                RxNum = 0;
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_IN), OdrvRxBuf, OdrvRxBufLen, &RxNum, 0);

                SL.B[0] = OdrvRxBuf[2]; SL.B[1] = OdrvRxBuf[3]; SL.B[2] = OdrvRxBuf[4]; SL.B[3] = OdrvRxBuf[5];
                Enc0 = SL.L;

                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_OUT), rqEnc1, 8, &actual, 0);
                RxNum = 0;
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_IN), OdrvRxBuf, OdrvRxBufLen, &RxNum, 0);

                SL.B[0] = OdrvRxBuf[2]; SL.B[1] = OdrvRxBuf[3]; SL.B[2] = OdrvRxBuf[4]; SL.B[3] = OdrvRxBuf[5];
                Enc1 = -SL.L;

                OdrvState = stVel0;
                break;

            case stEnc1:
                break;

            case stVel0:
                R.F = Vel0;
                rqVel0[6] = R.B[0]; rqVel0[7] = R.B[1]; rqVel0[8] = R.B[2]; rqVel0[9] = R.B[3];

                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_OUT), rqVel0, 12, &actual, 0);
                RxNum = 0;
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_IN), OdrvRxBuf, OdrvRxBufLen, &RxNum, 0);

                R.F = -Vel1;
                rqVel1[6] = R.B[0]; rqVel1[7] = R.B[1]; rqVel1[8] = R.B[2]; rqVel1[9] = R.B[3];

                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_OUT), rqVel1, 12, &actual, 0);
                RxNum = 0;
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_IN), OdrvRxBuf, OdrvRxBufLen, &RxNum, 0);

                OdrvState = stVel1;
                break;

            case stVel1:
                // Trigger flags
                if (SetIdle) {
                    SetIdle = false;
                    OdrvState = stIdle0;
                }
                else if (SetArm) {
                    SetArm = false;
                    OdrvState = stArm0;
                }
                else if (SetGain) {
                    SetGain = false;
                    OdrvState = stPGain0;
                }
                else {
                    OdrvState = stVBus;
                }
                break;

            case stArm0:
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_OUT), rqArm0, 9, &actual, 0);
                RxNum = 0;
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_IN), OdrvRxBuf, OdrvRxBufLen, &RxNum, 0);

                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_OUT), rqArm1, 9, &actual, 0);
                RxNum = 0;
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_IN), OdrvRxBuf, OdrvRxBufLen, &RxNum, 0);

                OdrvState = stVBus;
                break;

            case stArm1:
                break;

            case stIdle0:
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_OUT), rqIdle0, 9, &actual, 0);
                RxNum = 0;
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_IN), OdrvRxBuf, OdrvRxBufLen, &RxNum, 0);

                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_OUT), rqIdle1, 9, &actual, 0);
                RxNum = 0;
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_IN), OdrvRxBuf, OdrvRxBufLen, &RxNum, 0);

                OdrvState = stVBus;
                break;

            case stIdle1:
                break;

            case stPGain0:
                R.F = PGain;
                rqPGain0[6] = R.B[0]; rqPGain0[7] = R.B[1]; rqPGain0[8] = R.B[2]; rqPGain0[9] = R.B[3];
                rqPGain1[6] = R.B[0]; rqPGain1[7] = R.B[1]; rqPGain1[8] = R.B[2]; rqPGain1[9] = R.B[3];

                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_OUT), rqPGain0, 12, &actual, 0);
                RxNum = 0;
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_IN), OdrvRxBuf, OdrvRxBufLen, &RxNum, 0);

                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_OUT), rqPGain1, 12, &actual, 0);
                RxNum = 0;
                rs = libusb_bulk_transfer(dh, (EP_ID | LIBUSB_ENDPOINT_IN), OdrvRxBuf, OdrvRxBufLen, &RxNum, 0);

                OdrvState = stVBus;
                break;

            case stPGain1:
                break;

            default:
                break;
        }
    }
}

void odrv::Run(const float inSpeed, const float inDir) {
    float R,Q,SetSpeedM0,SetSpeedM1;

    Speed = inSpeed;
    //Dir = 0.4 * inDir;
    Dir = inDir;
    R = Dir;
    R = fabs(R);
    R = 1.0-R;
    Q = Speed * R;
    // Prepocet pro levy a pravy motor
    SetSpeedM0 = Q - Dir;
    SetSpeedM1 = Q + Dir;
    if (SetSpeedM0 > 1.0) {
        R = SetSpeedM0 - 1.0;
        SetSpeedM0 = 1.0;
        SetSpeedM1 = SetSpeedM1 - R;
    }
    if (SetSpeedM0 < -1.0) {
        R = -1.0 - SetSpeedM0;
        SetSpeedM0 = -1.0;
        SetSpeedM1 = SetSpeedM1 + R;
    }
    if (SetSpeedM1 > 1.0) {
        R = SetSpeedM1 - 1.0;
        SetSpeedM1 = 1.0;
        SetSpeedM0 = SetSpeedM0 - R;
    }
    if (SetSpeedM1 < -1.0) {
        R = -1.0 - SetSpeedM1;
        SetSpeedM1 = -1.0;
        SetSpeedM0 = SetSpeedM0 + R;
    }

    // FIXME Netusim, proc to tady scaluji * 0.02 !!!
    Vel0 = 0.02 * MaxVel_rps*SetSpeedM0;
    Vel1 = 0.02 * MaxVel_rps*SetSpeedM1;
}

void odrv::Stop() {
    //Run(0.0,0.0);
    Vel0 = 0.0;
    Vel1 = 0.0;
}

void odrv::Motors(const float in_omegaL, const float in_omegaR) {
    float RevPmL = 0.0;
    float RevPmR = 0.0;
    // Prevod [rad/s] na [rev/s]:
    // RevS [rev/s] = AngularV[rad/s] * 2*PI
    RevPmL = in_omegaL * 2 * M_PI;
    RevPmR = in_omegaR * 2 * M_PI;

    // FIXME Netusim, proc to tady scaluji * 0.02 !!!
    Vel0 = 0.02 * RevPmL;
    Vel1 = 0.02 * RevPmR;
}

} // end namespace
