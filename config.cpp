#include <iostream>
#include <iomanip>
#include <fstream>
#include "config.h"

json config;

void ReadConfig(void) {
    std::ifstream i("config.json");

    if (i.is_open()) {
        i >> config;
    }
    else {
        config = {
            {"configVersion","0.0.1"}
        };
        WriteConfig();
        std::cout << "> New config file created." << std::endl;
    }
}

void WriteConfig(void) {
    std::ofstream o("config.json");
    o << std::setw(4) << config << std::endl;
}
