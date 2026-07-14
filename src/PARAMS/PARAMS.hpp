#pragma once

#include "MASTERc.hpp"
#include <LittleFS.h>

class PARAMS {
public:
    PARAMS();
    ~PARAMS();

    bool load(MASTERc& config);
    void save(const MASTERc& config);
    
    void print_all(const MASTERc& config);
    void get_parameter(const MASTERc& config, const char* name);
    void set_parameter(MASTERc& config, const char* name, const char* value);
};
