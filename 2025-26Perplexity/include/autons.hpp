#pragma once
#include "lemlib/chassis/chassis.hpp"
#include <cmath>

// clang-format off
// normal person measurements
[[nodiscard]] inline const constexpr long double  operator""_in  (long double inches)          { return inches; }
[[nodiscard]] inline const constexpr long long    operator""_in  (unsigned long long inches)   { return inches; }
[[nodiscard]] inline const constexpr long double  operator""_ft  (long double feet)            { return feet * 12; }
[[nodiscard]] inline const constexpr long long    operator""_ft  (unsigned long long feet)     { return feet * 12; }
[[nodiscard]] inline const constexpr long double  operator""_cm  (long double cm)              { return cm / 2.54; }
[[nodiscard]] inline const constexpr long long    operator""_cm  (unsigned long long cm)       { return cm / 2.54; }
[[nodiscard]] inline const constexpr long double  operator""_m   (long double m)               { return m * 39.37; }
[[nodiscard]] inline const constexpr long long    operator""_m   (unsigned long long m)        { return m * 39.37; }

[[nodiscard]] inline const constexpr long double  operator""_pi  (long double input)           { return input * 3.14159; } 
[[nodiscard]] inline const constexpr long double  operator""_pi  (unsigned long long input)    { return input * 3.14159; }

[[nodiscard]] inline const constexpr long double  operator""_deg (long double deg)             { return operator""_pi(deg) / 180.0; }
[[nodiscard]] inline const constexpr long double  operator""_r   (long double r)               { return r * 180 / 1.0_pi; }
[[nodiscard]] inline const constexpr long double  operator""_r   (unsigned long long r)        { return r * 180 / 1.0_pi; }
[[nodiscard]] inline const constexpr long double  operator""_rt2 (long double r)               { return r * std::sqrt(2); }
[[nodiscard]] inline const constexpr long double  operator""_rt2 (unsigned long long r)        { return r * std::sqrt(2); }

[[nodiscard]] inline const constexpr long long    operator""_sec (long double sec)             { return sec * 1000; }
[[nodiscard]] inline const constexpr long long    operator""_sec (unsigned long long sec)      { return sec * 1000; }

// clang-format on

extern lemlib::Chassis chassis;

void park();
void auton_full(bool isBlue, bool doSkills);
void auto_bottomGoal(bool isBlue);

void autons_positive_red();
void autons_positive_blue();
void autons_negative_red();
void autons_negative_blue();
void auton_one_side(float mx, float my, bool doSkills = false);
void testing_pid();
void auton_skills();