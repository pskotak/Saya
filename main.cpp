#include <iostream>
#include <unistd.h>
#include <thread>
#include <string>

#include "SayaGlobals/globals.h"
#include "vision/vision.h"
#include "LocalMap/locmap.h"
#include "t265.h"
#include "qrcode.h"
#include "config.h"
#include "cpm4.h"
#include "gps.h"
#include "joystick.h"

bool parseGeoString(const std::string& geoString, double& lat, double& lon) {
    try {
        // Najdi pozici dvojtečky
        size_t colonPos = geoString.find(':');
        if (colonPos == std::string::npos) {
            return false; // Dvojtečka nenalezena
        }

        // Najdi pozici čárky
        size_t commaPos = geoString.find(',');
        if (commaPos == std::string::npos || commaPos < colonPos) {
            return false; // Čárka nenalezena nebo je před dvojtečkou
        }

        // Extrahuje podřetězec pro zeměpisnou šířku
        std::string latStr = geoString.substr(colonPos + 1, commaPos - (colonPos + 1));

        // Extrahuje podřetězec pro zeměpisnou délku
        std::string lonStr = geoString.substr(commaPos + 1);

        // Převede podřetězce na double
        lat = std::stod(latStr);
        lon = std::stod(lonStr);

        return true;
    } catch (const std::invalid_argument& ia) {
        std::cerr << "Chyba: Neplatný argument pro konverzi na double." << std::endl;
        return false;
    } catch (const std::out_of_range& oor) {
        std::cerr << "Chyba: Hodnota je mimo rozsah pro double." << std::endl;
        return false;
    }
}

// ============================================================================
int main(int argc, char **argv) {
    bool QuitProgram = false;
    std::thread CPM4Thread;
    cv::Mat processed_frame;
    std::string Qr,OldQr = "";
    bool JoyOK = true;
    int32_t Lat = 0; // Lat North positive - RMC, format v desetinach stupne * 1E7
    int32_t Lon = 0;
    int32_t GoalLat;
    int32_t GoalLon;
    double fGoalLat;
    double fGoalLon;
// ----------------------------------------------------------------------------

    std::cout << ProgVersion << std::endl;
    std::cout << "Exit program by pressing \"BigRed\" and next \"Mission start\" buttons OR Joystick X" << std::endl;
    ReadConfig();

    std::cout << "Starting CPM4 thread..." << std::endl;
    CPM4Thread = std::thread(&RunCom);
    std::cout << "Starting GPS thread..." << std::endl;
    std::thread GPSThread(RunGPS);

    std::thread T265Thread;
    std::unique_lock<std::mutex> mainT265_lock(T265_mutex, std::defer_lock);

    Joystick joy("/dev/input/js0");
    if (!joy.isFound()) {
        std::cout << ">>> NO Joystick" << std::endl;
        JoyOK = false;
    }

    vision::StartPcRow = 160;
    vision::EndPcRow = D455H-120;
    vision::Init();

    t265::t265_serial_number = vision::t265_serial_number; // vision::GetSerNo(); aktualizuje i T265 serial number
    T265Thread = std::thread(&t265::RunT265);

    while (!QuitProgram) {
        if (BigRedOn) {
            BigRedOn = false;
            BigRedSwitch = true; // Aktivni brzdeni (Speed = 0, Dir - 0)
            BigRedPressed = true;
            std::cout << "BigRed PRESSED" << std::endl;
        }
        if (BigRedOff) {
            BigRedOff = false;
            BigRedSwitch = false;
            BigRedReleased = true;
            //odrive.SetIdle = true; // Idle - prestat budit motory na nulovou rychlost
            std::cout << "BigRed released" << std::endl;
        }
        if (MissionOn) {
            MissionOn = false;
            MissionPressed = true;
            std::cout << "Mission PRESSED" << std::endl;
        }
        if (MissionOff) {
            MissionOff = false;
            MissionReleased = true;
            std::cout << "Mission released" << std::endl;
        }

        if (NewGPS) {
            Lat = GPSData.Lat;
            Lon = GPSData.Lon;
            NewGPS = false;
        }

        if (JoyOK) {
            joy.Update();

            if (joy.TrigArm) {
                joy.TrigArm = false;
                if (BigRedSwitch == false) { // Nepovolit armovani pri stisknutem BigRed
                    //odrive.SetArm = true;
                    std::cout << ">>> Joy ARM" << std::endl;
                }
            }
            if (joy.TrigIdle) {
                joy.TrigIdle = false;
                //odrive.SetIdle = true;
                std::cout << ">>> Joy IDLE" << std::endl;
            }
            if (joy.TrigX) {
                joy.TrigX = false;
                QuitProgram = true;
                std::cout << ">>> Joy X" << std::endl;
            }
            if (joy.TrigB) {
                joy.TrigB = false;
                // if (Logging) {
                //     std::cout << "> LOG End" << std::endl;
                //     EndLog();
                // }
                // else {
                //     std::cout << "> LOG Start" << std::endl;
                //     StartLog();
                // }
            }
            // if (joy.TrigRightFireSW) {
            //     joy.TrigRightFireSW = false;
            //     outfile << Lat << "," << Lon << std::endl;
            //     std::cout << "> GPS Mark: Lat = " << Lat*1E-7 << ", Lon = " << Lon*1E-7 << std::endl;
            // }
        }

        vision::Frame();
        if (vision::NewD455) {
            Qr = qr::GetCode(vision::RGB_image);
            //if (!Qr.empty() && (OldQr != Qr) ) {
            if (!Qr.empty()) {
                std::cout << "Qr: " << Qr << std::endl;
                if (parseGeoString(Qr,fGoalLat,fGoalLon)) {
                    GoalLat = round(fGoalLat * 1e7);
                    GoalLon = round(fGoalLon * 1e7);
                    std::cout << "Lat: " << GoalLat << " Lon: " << GoalLon << std::endl;
                }
            }
            OldQr = Qr;
            cv::imshow("RGB",vision::RGB_image);

            vision::NewD455 = false;
        }

        // Program lze ukoncit stiskem BigRedSwitch a nasledne stiskem Mission button
        if (BigRedSwitch && MissionPressed) {
            QuitProgram = true;
        }

        // Globalni udalosti - nulovat az na konci, aby na ne mohl reagovat cely system
        if (BigRedPressed) {
            BigRedPressed = false;
        }
        if (BigRedReleased) {
            BigRedReleased = false;
        }
        if (MissionPressed) {
            MissionPressed = false;
        }
        if (MissionReleased) {
            MissionReleased = false;
        }

        int key = cv::waitKey(2);
        if (key == 27) { // ESC key
            QuitProgram = true;
        }
        //usleep(2000);
    }

    std::cout << "down T265" << std::endl;
    t265::ShutdownT265 = true;
    T265Thread.join();

    std::cout << "down GPS" << std::endl;
    GPS_Shutdown = true;
    GPSThread.join();

    std::cout << "down CPM4" << std::endl;
    SerialShutdown = true;
    CPM4Thread.join();

    std::cout << "Saya finished" << std::endl;
    return 0;
}
