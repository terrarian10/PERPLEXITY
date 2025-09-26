#pragma once
#include <fstream>
#include <iostream>

class logger {
  public:
    logger(const char* file)
        : file(file)
        , log_file(file, std::ios::app) {

        };

    inline void inject(const std::string& msg) {
        log_file << msg << std::endl;
    };

  private:
    const char* file;
    std::ofstream log_file;
};