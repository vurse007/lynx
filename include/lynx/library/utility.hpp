#pragma once
#include "main.h"
#include <chrono>

namespace lynx{
    class timer{
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

            //reset the timer
            void reset(){
                start_time = 0;
                running = false;
            }

            //restart the timer (reset + start)
            void restart(){
                start();
            }

            //checks if a certain amount of time has passed
            bool has_elapsed(u_int32_t ms) const{
                return elapsed() >= ms;
            }

            //stop the timer
            void stop(){
                running = false;
            }
    };
}