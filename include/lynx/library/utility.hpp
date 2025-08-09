#pragma once
#include "main.h"
#include <chrono>
#include "config.hpp"

#define LYNX_NULL std::nullopt

namespace lynx{

    //keeps track of time using pros::millis - make objects to have multiple timers throughout project
    class timer{
        public:
            u_int32_t start_time = pros::millis();
            u_int32_t target_time;
            bool running = false;

            //constructor
            timer(u_int16_t target_time = 0){
                this->target_time = target_time;
            }

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
            bool has_elapsed(u_int32_t ms = 0) const{
                if (ms == 0) ms = target_time;
                return elapsed() >= ms;
            }

            //stop the timer
            void stop(){
                running = false;
            }
    };

    //allows you to save large polynomials and evaluate them when needed - main use is taylor polynomials throughout this project
    class poly{
        private:
            std::vector<long double> coefficients;

        public:
            poly(const std::vector<long double>& coeffs){
                this->coefficients = coeffs;
            }

            void update_coefficients(const std::vector<long double>& coeffs){
                this->coefficients = coeffs;
            }

            long double evaluate(long double x) const{
                long double y = 0.00;
                for (const double& coeff : this->coefficients){
                    y = (y*x) + coeff; //using const to prevent changing of coeffs and & for preventing copy of the vector every time the for loop runs
                }
                return y;
            }

            long double scientific_notation(long double number, double exponent){
                return number * pow(10, exponent);
            }
    };

    //utility functions
    namespace util {
        double wrap_to_180(double angle){
            //wraps an angle to the range of -180 to 180 degrees
            angle = fmod(angle + 180.0, 360.0);
            if (angle < 0) angle += 360.0;
            return angle - 180.0;
        }
        double wrap_to_pi(double angle) {
            angle = std::fmod(angle + M_PI, 2 * M_PI);
            if (angle < 0) angle += 2 * M_PI;
            return angle - M_PI;
        }

        void absolute_logic(double& position, double& target){
            if ((target < 0) && (position > 0)){ //if the target is negative and the position is positive
                if ((position - target) >= 180){ //see if the error is greater than 180 degreees - we need to turn other way if so
                    target = target + 360; //take the target and add a full rotation to it so we now are turning the shorter direction
                    position = global::imu.get_heading(); //update position
                }
            }
            else if ((target > 0) && (position < 0)){ //if the target is positive and the position is negative
                if ((target - position) >= 180){ //see if the error is greater than 180
                    target = target - 360; //add a full rotation to the left this time so we can turn the shorter direction
                    position = global::imu.get_heading();
                }
            }
        }
    }

    //enum class for passing flags into a function to change the way it behaves
    enum class flags {
        none = 0, //0000
        prt_error = 1 << 0, //0001
        prt_time = 1 << 1, //0010
        hc_off = 1 << 2 //0100
    };
    //since we are using enum class for safety we must do some operator functions since the types need to be casted
    inline flags operator|(flags a, flags b){
        return static_cast<flags>(
            static_cast<std::underlying_type_t<flags>>(a) | static_cast<std::underlying_type_t<flags>>(b)
        );
    }
    inline flags operator|=(flags& a, flags b){
        a = a | b;
        return a;
    }
    inline bool has_flag(flags flag, flags flags_to_check){
        return (static_cast<std::underlying_type_t<flags>>(flag) & static_cast<std::underlying_type_t<flags>>(flags_to_check)) != 0;
    }

    timer drive_timer; //both are initialized with default values of target=0 - update later based on the user's needs
    timer turn_timer;
    timer arc_timer;
}