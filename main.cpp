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
#include "logger.h"
#include "platform.h"

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
    bool AutoLog = false;
    int32_t Lat = 0; // Lat North positive - RMC, format v desetinach stupne * 1E7
    int32_t Lon = 0;
    int32_t GoalLat = 0;
    int32_t GoalLon = 0;
    double fGoalLat = 0.0;
    double fGoalLon = 0.0;

    int Cnt = 0;

    float yaw_rad,R;
// Dummy log variables
    float setVelocity = 0.0;
    float setAngularVelocity = 0.0;
    float omegaL = 0.0;
    float omegaR = 0.0;

// ----------------------------------------------------------------------------

    std::cout << ProgVersion << std::endl;
    std::cout << "Exit program by pressing \"BigRed\" and next \"Mission start\" buttons OR Joystick X" << std::endl;
    ReadConfig();
    if (!config.contains("logging")) {
        config["logging"]["autolog"] = false; // Vytvoril jsem si udev rule
        WriteConfig();
    }
    AutoLog = config["logging"]["autolog"];
    std::cout << ">>> AutoLog: " << AutoLog << std::endl;

    std::cout << "Starting CPM4 thread..." << std::endl;
    CPM4Thread = std::thread(&RunCom);

    std::thread GPSThread(RunGPS);

    std::thread T265Thread;
    std::unique_lock<std::mutex> mainT265_lock(T265_mutex, std::defer_lock);

    Joystick joy("/dev/input/js0");
    if (!joy.isFound()) {
        std::cout << ">>> NO Joystick" << std::endl;
        JoyOK = false;
    }
    platform::InitPlatform();

    vision::StartPcRow = 160;
    vision::EndPcRow = D455H-120;
    vision::Init();

    t265::t265_serial_number = vision::t265_serial_number; // vision::GetSerNo(); aktualizuje i T265 serial number
    T265Thread = std::thread(&t265::RunT265);

    //std::cout << std::endl << std::endl << std::endl;
    sleep(2);

    bool First = true;

    while (!QuitProgram) {
        if (BigRedOn) {
            BigRedOn = false;
            BigRedSwitch = true; // Aktivni brzdeni (Speed = 0, Dir - 0)
            BigRedPressed = true;
            //std::cout << "BigRed PRESSED" << std::endl;
        }
        if (BigRedOff) {
            BigRedOff = false;
            BigRedSwitch = false;
            BigRedReleased = true;
            //odrive.SetIdle = true; // Idle - prestat budit motory na nulovou rychlost
            //std::cout << "BigRed released" << std::endl;
        }
        if (MissionOn) {
            MissionOn = false;
            MissionPressed = true;
            //std::cout << "Mission PRESSED" << std::endl;
        }
        if (MissionOff) {
            MissionOff = false;
            MissionReleased = true;
            //std::cout << "Mission released" << std::endl;
        }

        if (MissionPressed) {
            if (AutoLog) {
                if (!logger::Logging) {
                    logger::StartLog();
                }
                // if (logger::Logging) {
                //     logger::EndLog();
                // }
                // else {
                //     logger::StartLog();
                // }
            }
        }

        // if (BigRedPressed) {
        //     if (AutoLog) {
        //         if (logger::Logging) {
        //             logger::EndLog();
        //         }
        //     }
        // }

        if (NewGPS) {
            Lat = GPSData.Lat;
            Lon = GPSData.Lon;

// BEGIN Status screen --------------------------------------------------------
            if (First) {
                 First = false;
                 std::cout << "-------------------------" << std::endl;
                 //std::cout << "Counter: " << Cnt << std::endl;
                 std::cout << "<GPS> Fix: " << GPSData.Fix << " Lat: " << Lat*1e-7 << " Lon: " << Lon*1e-7 << " GLat: " << GoalLat*1e-7 << " GLon: " << GoalLon*1e-7 << " Alive: " << Cnt++ << std::endl;
            }
            else {
                std::cout << "\x1b[3F" << "<GPS> Fix: " << GPSData.Fix << " Lat: " << Lat*1e-7 << " Lon: " << Lon*1e-7 << " GLat: " << GoalLat*1e-7 << " GLon: " << GoalLon*1e-7 << " Alive: " << Cnt++ << std::endl;
                //std::cout << "\x1b[3F" << "Counter: " << Cnt << "                                   " << std::endl;
            }
            if (platform::Armed) {
                std::cout << "<Platform> " << "ARMED" << " Log: " << logger::Logging << "                    " << std::endl;
            }
            else {
                std::cout << "<Platform> " << "IDLE" << " Log: " << logger::Logging << "                     " << std::endl;
            }


            std::cout << "<Mission> " << "IDLE" << "                                                " << std::endl;
// END Status screen ----------------------------------------------------------
            NewGPS = false;
        }

        if (JoyOK) {
            joy.Update();

            if (joy.TrigArm) {
                joy.TrigArm = false;
                if (BigRedSwitch == false) { // Nepovolit armovani pri stisknutem BigRed
                    platform::Arm();
                    //odrive.SetArm = true;
                    //std::cout << ">>> Joy ARM" << std::endl;
                }
            }
            if (joy.TrigIdle) {
                joy.TrigIdle = false;
                platform::Idle();
                //odrive.SetIdle = true;
                //std::cout << ">>> Joy IDLE" << std::endl;
            }
            if (joy.TrigX) {
                joy.TrigX = false;
                QuitProgram = true;
                //std::cout << ">>> Joy X" << std::endl;
            }
            if (joy.TrigB) {
                joy.TrigB = false;
                if (logger::Logging) {
                    //std::cout << "> LOG End" << std::endl;
                    logger::EndLog();
                }
                else {
                    //std::cout << "> LOG Start" << std::endl;
                    logger::StartLog();
                }
            }
            // if (joy.TrigRightFireSW) {
            //     joy.TrigRightFireSW = false;
            //     outfile << Lat << "," << Lon << std::endl;
            //     std::cout << "> GPS Mark: Lat = " << Lat*1E-7 << ", Lon = " << Lon*1E-7 << std::endl;
            // }
            platform::GoJoy(joy.Speed,joy.Dir);
        }

        vision::Frame();
        if (vision::NewD455) {
            vision::depth_image16.convertTo(processed_frame,CV_8U,255.0 / D455depth_color_max,0.0);
            cv::applyColorMap(processed_frame,processed_frame,cv::COLORMAP_JET);
            cv::imshow("Depth",processed_frame);

            Qr = qr::GetCode(vision::RGB_image);
            if (!Qr.empty()) {
                if (parseGeoString(Qr,fGoalLat,fGoalLon)) {
                    GoalLat = round(fGoalLat * 1e7);
                    GoalLon = round(fGoalLon * 1e7);
                    //std::cout << "\x1b[A" << "Lat: " << GoalLat << " Lon: " << GoalLon << std::endl;
                }
            }
            OldQr = Qr;

            if (mainT265_lock.try_lock()) {
//                 Yaw = rs_yaw;
//                 YawRad = rs_yaw_rad;
//                 Pitch = rs_pitch;
//                 Roll = rs_roll;
//                 Velocity = rs_velocity;
//                 AngularVelocity = rs_angular_velo;
//                 PosX = rs_x;
//                 PosY = rs_y;
//                 PosZ = rs_z;
                yaw_rad = t265::rs_yaw_rad;
                BotPos = t265::rs_BotPos;
                BotOrientation = t265::rs_BotOrientation;
                mainT265_lock.unlock();
            }

// Create log record
            if (logger::Logging) {
                //LogRec.RecNo = RecCnt; -> Doplni si logger
                logger::LogRec.Pose = t265::Pose; //platformPose;
                logger::LogRec.setVelocity = setVelocity;
                logger::LogRec.setAngularVelocity = setAngularVelocity;
                logger::LogRec.omegaL = omegaL;
                logger::LogRec.omegaR = omegaR;
                //LogRec.Timestamp = milliseconds_count; -> Doplni si logger
                logger::LogRec.UTCTime = GPSData.UTCTime; // UTC time - RMC
                logger::LogRec.UTCDate = GPSData.UTCDate;
                logger::LogRec.Lat = GPSData.Lat;
                logger::LogRec.Lon = GPSData.Lon;
                logger::LogRec.Brg = GPSData.Hdg;
                logger::LogRec.Fix = GPSData.Fix;

                logger::rgbData = vision::RGB_image.data;
                logger::depthData = reinterpret_cast<unsigned char*>(vision::depth_image16.data); // Cast depth data
                logger::WriteLogRec();
            }

            //cv::imshow("RGB",vision::RGB_image);
            vision::NewD455 = false;
        }
        platform::Update();

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
        usleep(2000);
    }

    platform::Idle();
    if (logger::Logging) {
        logger::EndLog();
    }

    std::cout << "-------------------------" << std::endl;

    platform::Shutdown();

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
