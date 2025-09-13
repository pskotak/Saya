#include <iostream>
#include <unistd.h>
#include <thread>

#include "SayaGlobals/globals.h"
#include "vision/vision.h"
#include "LocalMap/locmap.h"
#include "t265.h"

int main(int argc, char **argv) {
    bool QuitProgram = false;
    cv::Mat processed_frame;
    
    std::cout << ProgVersion << std::endl;
    
    vision::Init();
    std::cout << "Vision running." << std::endl;
    
    while (!QuitProgram) {
        vision::Frame();
        if (vision::NewD455) {
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
