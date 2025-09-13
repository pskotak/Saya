// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

#include "joystick.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sstream>
#include "unistd.h"
#include "SayaGlobals/globals.h"

Joystick::Joystick()
{
  openPath("/dev/input/js0");
}

Joystick::Joystick(int joystickNumber)
{
  std::stringstream sstm;
  sstm << "/dev/input/js" << joystickNumber;
  openPath(sstm.str());
}

Joystick::Joystick(std::string devicePath)
{
  openPath(devicePath);
}

Joystick::Joystick(std::string devicePath, bool blocking)
{
  openPath(devicePath, blocking);
}

void Joystick::openPath(std::string devicePath, bool blocking)
{
  // Open the device using either blocking or non-blocking
  _fd = open(devicePath.c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);
}

bool Joystick::sample(JoystickEvent* event)
{
  int bytes = read(_fd, event, sizeof(*event));

  if (bytes == -1)
    return false;

  // NOTE if this condition is not met, we're probably out of sync and this
  // Joystick instance is likely unusable
  return bytes == sizeof(*event);
}

bool Joystick::isFound()
{
  return _fd >= 0;
}

Joystick::~Joystick()
{
  close(_fd);
}

std::ostream& operator<<(std::ostream& os, const JoystickEvent& e)
{
  os << "type=" << static_cast<int>(e.type)
     << " number=" << static_cast<int>(e.number)
     << " value=" << static_cast<int>(e.value);
  return os;
}

void Joystick::Update() {
    JoystickEvent jev;
    if (sample(&jev)) {
        if (jev.isButton()) {
            //std::cout << "BT: No=" << +jev.number << " V=" << jev.value << std::endl;
            // A = 0, B = 1, X = 3, Y = 4, Left Fire SW = 6, Right Fire SW = 7
            if (jev.number == 4) {
                if (jev.value == 1) {
                    Arm = true;
                    if (PrevArm == false) {
                        TrigArm = true;
                    }
                }
                else {
                    Arm = false;
                }
                PrevArm = Arm;
            }
            else if (jev.number == 0) {
                if (jev.value == 1) {
                    Idle = true;
                    if (PrevIdle == false) {
                        TrigIdle = true;
                    }
                }
                else {
                    Idle = false;
                }
                PrevIdle = Idle;
            }
            else if (jev.number == 3) {
                if (jev.value == 1) {
                    BtX = true;
                    if (PrevX == false) {
                        TrigX = true;
                    }
                }
                else {
                    BtX = false;
                }
                PrevX = BtX;
            }
            else if (jev.number == 1) {
                if (jev.value == 1) {
                    BtB = true;
                    if (PrevB == false) {
                        TrigB = true;
                    }
                }
                else {
                    BtB = false;
                }
                PrevB = BtB;
            }
            else if (jev.number == 7) {
                if (jev.value == 1) {
                    RightFireSW = true;
                    if (PrevRightFireSW == false) {
                        TrigRightFireSW = true;
                    }
                }
                else {
                    RightFireSW = false;
                }
                PrevRightFireSW = RightFireSW;
            }

        }
        else if (jev.isAxis()) {
            //std::cout << "Axis: No=" << +jev.number << " V=" << jev.value << std::endl;
            // Speed - Axis 3, vpred = zaporna hodnota, vzad = kladna hodnota, +/- val = 32767
            // Dir - Axis 2, vpravo = kladna hodnota, vlevo = zaporna hodnota, +/- val = 32767
            if (jev.number == 3) {
                Speed = -jev.value / 32767.0;
            }
            else if (jev.number == 2) {
                Dir = jev.value / 32767.0;
            }
        }
    }
}
