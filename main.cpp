#include <iostream>
#include "vision/vision.h"

int main(int argc, char **argv) {
    std::cout << "Saya the robot" << std::endl;
    
    vision::Init();
    
    return 0;
}
