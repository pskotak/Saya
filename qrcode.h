#ifndef QRCODE_H
#define QRCODE_H

#include <string>
#include "SayaGlobals/globals.h"

namespace qr {
    
extern std::string GetCode(cv::Mat image);

    
} // end namespace

#endif
