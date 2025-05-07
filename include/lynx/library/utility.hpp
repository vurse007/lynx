#pragma once
#include "main.h"
#include <chrono>

namespace lynx{
    class time{
        public:
            u_int16_t start_time = pros::millis();
            bool running = false;

            //start the timer
            void start(){
                start_time = pros::millis();
                running = true;
            }

            //see how long the timer has been running
            u_int32_t elapsed() const{
                if (!running) return 0;
                return pros::millis() - start_time;
            }
    };
}