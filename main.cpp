#include <iostream>
#include <unistd.h>
#include <thread>
#include <string>

#include "SayaGlobals/globals.h"
#include "vision/vision.h"
#include "LocalMap/locmap.h"
#include "t265.h"
#include "qrcode.h"

int main(int argc, char **argv) {
    bool QuitProgram = false;
    cv::Mat processed_frame;
    std::string Qr,OldQr = "";

    std::cout << ProgVersion << std::endl;

    vision::Init();
    std::cout << "Vision running." << std::endl;

    while (!QuitProgram) {
        vision::Frame();
        if (vision::NewD455) {
            Qr = qr::GetCode(vision::RGB_image);
            //if (!Qr.empty() && (OldQr != Qr) ) {
            if (!Qr.empty()) {
                std::cout << "Qr: " << Qr << std::endl;
            }
            OldQr = Qr;
            cv::imshow("RGB",vision::RGB_image);

            vision::NewD455 = false;
        }

        int key = cv::waitKey(1); // Wait for a key press for 1ms
        if (key == 27) { // ESC key
            QuitProgram = true;
        }
        usleep(10000);
    }
    std::cout << "Saya finished" << std::endl;
    return 0;
}
