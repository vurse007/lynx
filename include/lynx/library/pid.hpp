#pragma once
#include <cmath>
#include <algorithm>

namespace lynx {
    struct constants {
        double kP;
        double kI;
        double kD;
        double kF;

        constants(double kP, double kI, double kD, double kF)
            : kP(kP), kI(kI), kD(kD), kF(kF) {}
    };

    //pid class
    class PID {
        public:
            constants general_constants;
            constants refined_constants;
            double refined_range;

            double current_value;
            double target_value;
            double prev_value;
            double error = 0;
            double prev_error = 0;
            double total_error = 0;
            double derivative = 0;

            double prev_speed = 0;
            double slew;

            double speed = 0;

            static double global_heading; //shared across all pids

            double integral_threshold;
            double max_integral;
            double deadband;

            PID(constants general, double refine_range, constants refined, double integral_threshold, double max_integral, double slew = 127, double deadband)
                :   general_constants(general),
                    refined_constants(refined),
                    refined_range(refine_range),
                    integral_threshold(integral_threshold),
                    max_integral(max_integral),
                    deadband(deadband),
                    slew(slew) {}

            double calculate(double target, double current, double speed_limit){
                // Set current, target, and error
                this->target_value = target;
                this->current_value = current;
                this->error = target_value - current_value;

                // === Deadband ===
                if (fabs(this->error) < this->deadband) {
                    this->error = 0;
                    this->total_error = 0;
                    this->speed = 0;
                    this->prev_error = 0;
                    this->prev_speed = 0;
                    this->prev_value= this->current_value;
                    return 0;
                }

                // === Integral Term ===
                if (fabs(this->error) < this->integral_threshold) {
                    this->total_error += ((this->error + this->prev_error) / 2.0) * 0.005; // trapezoidal integration with dt = 0.005
                }

                // Clamp integral to bounds
                this->total_error = std::clamp(this->total_error, -this->max_integral, this->max_integral);

                // === Derivative on Measurement ===
                this->derivative = -(this->current_value - this->prev_value) / 0.005;

                // === Raw PID Output (before feedforward and clamping) ===
                double rawSpeed;
                if (fabs(this->error) < this->refined_range) {
                    rawSpeed = (this->refined_constants.kP * this->error) +
                            (this->refined_constants.kI * this->total_error) +
                            (this->refined_constants.kD * this->derivative);
                } else {
                    rawSpeed = (this->general_constants.kP * this->error) +
                            (this->general_constants.kI * this->total_error) +
                            (this->general_constants.kD * this->derivative);
                }

                // === Feedforward ===
                if (fabs(this->error) < this->refined_range) {
                    rawSpeed += this->refined_constants.kF * target_value;
                } else {
                    rawSpeed += this->general_constants.kF * target_value;
                }

                // === Anti-windup via back-calculation ===
                double clampedSpeed = std::clamp(rawSpeed, -speed_limit, speed_limit);
                if (fabs(this->error) < this->refined_range){
                    this->total_error += (clampedSpeed - rawSpeed) * this->refined_constants.kI * 0.005; // scaled like integral
                } else {
                    this->total_error += (clampedSpeed - rawSpeed) * this->general_constants.kI * 0.005; // scaled like integral
                }

                // Use clamped speed
                this->speed = clampedSpeed;

                // === Slew Rate Limit ===
                double deltaSpeed = this->speed - this->prev_speed;
                if (deltaSpeed > this->slew) {
                    this->speed = this->prev_speed + this->slew;
                } else if (deltaSpeed < -this->slew) {
                    this->speed = this->prev_speed - this->slew;
                }

                // Final output clamp (again, just to be safe)
                this->speed = std::clamp(this->speed, -speed_limit, speed_limit);

                // === Prepare for next iteration ===
                this->prev_error = this->error;
                this->prev_speed = this->speed;
                this->prev_value = this->current_value;

                return this->speed;
            }
    };
}