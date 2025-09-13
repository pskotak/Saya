#ifndef PLATFORM_H
#define PLATFORM_H

#include "SayaGlobals/globals.h"

namespace platform {

extern bool Armed;

extern void InitPlatform();
extern void Shutdown();
extern void Update();
extern void Idle();
extern void Arm();
extern void GoJoy(const float inSpeed, const float inDir);
extern void Stop();

} // end namespace

#endif
