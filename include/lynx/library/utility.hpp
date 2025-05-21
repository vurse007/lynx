#pragma once
#include "main.h"
#include <chrono>

namespace lynx{
    class timer{
        public:
            u_int16_t start_time = pros::millis();
            u_int16_t target_time;
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
            bool has_elapsed(u_int32_t ms) const{
                return elapsed() >= ms;
            }

            //stop the timer
            void stop(){
                running = false;
            }
    };

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
}