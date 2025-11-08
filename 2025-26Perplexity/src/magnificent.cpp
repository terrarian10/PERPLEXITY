
#include "magnificent.hpp"
#include "Outtake.hpp"
#include "airCylinder.hpp"
#include "autons.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

std::vector<std::string> magnificent::convertToWords(std::string line) {
    std::vector<std::string> v_ret;
    v_ret.reserve(std::count(line.begin(), line.end(), ' '));
    std::istringstream iss(line);
    std::string word;

    while (iss >> word) {
        v_ret.push_back(word);
    }

    return v_ret;
}

void parse_line(std::vector<std::string> words) {
    if ((words.at(0) + words.at(1)) == "goto") {}
}

void magnificent::compiler(std::string path) {
    std::ifstream file(path);
    std::vector<std::string> lines;
    std::string line;

    while (std::getline(file, line))
        lines.push_back(line);

    // Now iterate through them
    for (auto& i : lines) {

        parse_line(convertToWords(i));
    }
}

void magnificent::compiler_locations(std::string path) {
    std::ifstream file(path);
    std::vector<std::string> lines;
    std::string line;

    while (std::getline(file, line))
        lines.push_back(line);

    // Now iterate through them
    for (auto& i : lines) {}
}