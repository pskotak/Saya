#ifndef CPM4_H
#define CPM4_H

#include "SayaGlobals/globals.h"

#include <atomic>
#include <mutex>

extern std::atomic<bool> BigRedOn;
extern std::atomic<bool> BigRedOff;
extern std::atomic<bool> MissionOn;
extern std::atomic<bool> MissionOff;

extern std::atomic<bool> SerialOK;
extern std::atomic<bool> SerialShutdown;
extern bool NewCOMData;

extern char CPM4PortName[20];

extern int CheckCom(void);
extern int RunCom(void);

#endif
