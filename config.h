#ifndef CONFIG_H
#define CONFIG_H

#include "nlohmann/json.hpp"

using json = nlohmann::json;

extern json config;

extern void ReadConfig(void);
extern void WriteConfig(void);

#endif
