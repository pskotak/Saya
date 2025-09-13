#include "platform.h"
#include "config.h"
#include "odrv.h"

namespace platform {

float CruiseSpeed = 0.6;
float CruiseExp = 0.1;
float TurnSpeed = 0.8;
float TurnExp = 0.1;
bool UseOdrive = false;
bool Armed = false;

ODRV::odrv odrive;



// output = ( (1 - factor) x input^3 ) + ( factor x input )
// factor = 1 is linear.
// 0 < factor < 1 is less sensitive in the center, more sensitive at the ends of throw.
float GetExponential(const float In, const float Ex, const float Lim) {
    float Q = 0.0;
    float Cub = In*In*In;

    Q = ((1.0-Ex) * Cub) + (Ex*In);
    Q = Q * Lim;
    return Q;
}

void InitPlatform() {
    if (!config.contains("platform")) {
        config["platform"]["CruiseSpeed"] = 0.6;
        config["platform"]["TurnSpeed"] = 0.8;
        config["platform"]["CruiseExp"] = 0.1;
        config["platform"]["TurnExp"] = 0.1;
        config["platform"]["UseOdrive"] = false;
        WriteConfig();
    }
    CruiseSpeed = config["platform"]["CruiseSpeed"];
    CruiseExp = config["platform"]["CruiseExp"];
    TurnSpeed = config["platform"]["TurnSpeed"];
    TurnExp = config["platform"]["TurnExp"];
    UseOdrive = config["platform"]["UseOdrive"];
}

void Shutdown() {
    odrive.Shutdown();
}

void Update() {
    odrive.Update();
}

void Idle() {
    Armed = false;
    if (UseOdrive) {
        odrive.SetIdle = true; // Idle - prestat budit motory na nulovou rychlost
    }
}

void Arm() {
    Armed = true;
    if (UseOdrive) {
        odrive.SetArm = true;
    }
}

void GoJoy(const float inSpeed, const float inDir) {
    float Speed = GetExponential(inSpeed,CruiseExp,CruiseSpeed);
    float Dir = GetExponential(inDir,TurnExp,TurnSpeed);
    if (BigRedSwitch) {
        Speed = 0.0;
        Dir = 0.0;
    }
    if (UseOdrive) {
        odrive.Run(Speed,Dir);
    }
}

void Stop() {
    odrive.Stop();
}

} // end namespace
