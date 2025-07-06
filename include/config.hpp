#pragma once
#include "main.h"
#include "lynx.hpp"

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

    namespace immutables{
        constexpr double DEFAULT_NUDGE_MAGNITUDE = 10;
    }

    lynx::drivetrain chassis {
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
        3.25,
        0.75,
        500
    };

    pros::Imu imu(port::IMU);
    pros::Controller con(pros::E_CONTROLLER_MASTER);
}


//create and edit pid objects below
lynx::PID drive_default{
    {1,0,0,0}, //general constants
    50, //refined range
    {2,0,0,0}, //refined constants
    500, //integral threshold (what range the integral is enabled and is allowed to built up)
    1000, //max integral value allowed
    127, //slew (set to 127 to disable)
    10, //what range does the settle timer enable in
    500, //settle timer must run for these many ms to break out of the pid loop
    7 //deadband (at +- what value will the pid stop giving an output)
};

lynx::PID turn_default{
    {3,0,0,0},
    10,
    {4,0,0,0},
    500, 
    1000,
    127,
    3,
    500,
    1
};

lynx::PID arc_default{
    {1,0,0,0}, //general constants
    50, //refined range
    {2,0,0,0}, //refined constants
    500, //integral threshold (what range the integral is enabled and is allowed to built up)
    1000, //max integral value allowed
    127, //slew (set to 127 to disable)
    10, //what range does the settle timer enable in
    500, //settle timer must run for these many ms to break out of the pid loop
    7 //deadband (at +- what value will the pid stop giving an output)
};

lynx::PID heading_correction_default{
    {3,0,0,0},
    10,
    {4,0,0,0},
    500, 
    1000,
    127,
    3,
    500,
    1
};

//create polynomial objects below
lynx::poly drive_timeout {
    {
        5, //x^4
        4, //x^3
        3, //x^2
        2, //x
        1, //constant
    }
};

lynx::poly turn_timeout {
    {
        5, //x^4
        4, //x^3
        3, //x^2
        2, //x
        1, //constant
    }
};

lynx::poly arc_timeout {
    {
        5, //x^4
        4, //x^3
        3, //x^2
        2, //x
        1, //constant
    }
};