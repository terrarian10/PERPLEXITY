#pragma once

#include "Outtake.hpp"
#include "airCylinder.hpp"
#include "autons.hpp"
#include "consts.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

class magnificent {
    std::vector<std::string> convertToWords(std::string line);
    void compiler(std::string path);
    void compiler_locations(std::string path);
};