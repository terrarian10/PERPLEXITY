
#include "magnificent.hpp"
#include "Outtake.hpp"
#include "airCylinder.hpp"
#include "autons.hpp"
#include "consts.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>

void magnificent::compiler(std::string path) {
    std::ifstream file(path);
    std::vector<std::string> lines;
    std::string line;

    while (std::getline(file, line))
        lines.push_back(line);

    // Now iterate through them
    for (auto& i : lines) {}
}