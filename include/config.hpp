#pragma once
#include "main.h"
#include "motors.hpp"

// create all of your devices and a lynx chassis inside of the global namespace
namespace global{
    namespace port{
        constexpr int FR = 1;
        constexpr int FL = 2;
        constexpr int MR = 3;
        constexpr int ML = 4;
        constexpr int BR = 5;
        constexpr int BL = 6;

        constexpr int IMU = 7;
    }

    lynx::chassis chassis {
        {
            {port::FR, pros::E_MOTOR_GEAR_600, false},
            {port::MR, pros::E_MOTOR_GEAR_600, false},
            {port::BR, pros::E_MOTOR_GEAR_600, true}
        },
        {
            {port::FL, pros::E_MOTOR_GEAR_600, true},
            {port::ML, pros::E_MOTOR_GEAR_600, false},
            {port::BL, pros::E_MOTOR_GEAR_600, true}
        },
        2.75,
        1.0,
        500
    };

    pros::Imu imu(port::IMU);
}

//create a pid object below