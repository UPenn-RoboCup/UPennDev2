#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <vector>
#include <string>
#include <stdlib.h>

extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

// config : global interface for config data
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

class Config {
  public:
    Config();
    ~Config();
    int get_int(std::string field);
    bool get_boolean(std::string field);
    double get_double(std::string field);
    std::string get_string(std::string field);
    std::vector<int> get_int_vector(std::string field);
    std::vector<bool> get_boolean_vector(std::string field);
    std::vector<double> get_double_vector(std::string field);
    std::vector<std::string> get_string_vector(std::string field);
  private:
    lua_State *L;
    void push_field(std::string field);
};

#endif
