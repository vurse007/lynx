#pragma once
#include "main.h"

//pid class
class PID {
    public:
        double kP;
        double kI;
        double kD;
        double kF;

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

        PID(double kP, double kI, double kD, double kF, double integral_threshold, double max_integral, double slew=127, double deadband){
            this->kP = kP;
            this->kI = kI;
            this->kD = kD;
            this->kF = kF;

            this->integral_threshold = integral_threshold;
            this->max_integral = max_integral;
            this->deadband = deadband;

            this->slew = slew;
        }

        double calculate(double target, double current, double speed_limit){
            // Set current, target, and error
            this->target_value = target;
            this->current_value = current;
            this->error = target_value - target;

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
            if (fabs(this->error) < 50) {
                rawSpeed = (this->kP * 1.8 * this->error) +
                        (this->kI * this->total_error) +
                        (this->kD * 0.8 * this->derivative);
            } else {
                rawSpeed = (this->kP * this->error) +
                        (this->kI * this->total_error) +
                        (this->kD * this->derivative);
            }

            // === Feedforward ===
            rawSpeed += this->kF * target_value;

            // === Anti-windup via back-calculation ===
            double clampedSpeed = std::clamp(rawSpeed, -speed_limit, speed_limit);
            double antiWindup = (clampedSpeed - rawSpeed) * this->kI * 0.005; // scaled like integral
            this->total_error += antiWindup;

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