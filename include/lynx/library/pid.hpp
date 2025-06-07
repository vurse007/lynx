#pragma once
#include <cmath>
#include <algorithm>
#include "utility.hpp"
#include "config.hpp"
#include "motors.hpp"

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
            double count_range;
            double settle_timer_target;

            PID(constants general, double refine_range, constants refined, double integral_threshold, double max_integral, double slew, double count_range, double settle_timer_target, double deadband)
                :   general_constants(general),
                    refined_constants(refined),
                    refined_range(refine_range),
                    integral_threshold(integral_threshold),
                    max_integral(max_integral),
                    slew(slew),
                    count_range(count_range),
                    settle_timer_target(settle_timer_target),
                    deadband(deadband) {}

            double calculate(double target, double current, double speed_limit=127){
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

            void reset(){
                this->error = 0;
                this->prev_error = 0;
                this->total_error = 0;
                this->derivative = 0;
                this->prev_speed = 0;
                this->speed = 0;
            }

            bool has_settled(){

                timer settler_timer(this->settle_timer_target);

                if (fabs(this->error) < this->count_range){
                    settler_timer.start();
                }

                if (settler_timer.elapsed()){
                    settler_timer.reset();
                    return true;
                } else{
                    return false;
                }
            }

    };

    void turn_absolute(double target, std::optional<double> chain_pos = LYNX_NULL, std::optional<double> timeout = LYNX_NULL, lynx::PID& pid = turn_default, flags passed_flags = flags::none){
        //do absolute calculations
        double position = global::imu.get_heading();
        if (position > 180) position = ((360-position) * -1); //wrap to -180 to 180

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

        global::chassis.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        pid.reset();

        //find the timeout and set it
        drive_timer.target_time = timeout.value_or(drive_timeout.evaluate(target));
        drive_timer.restart();

        while (true){

            //get the current heading and do a bit of math ;D
            double current_heading = global::imu.get_heading();
            if (current_heading > 180) current_heading = ((360-current_heading) * -1); //wrap to -180 to 180
            if ((target < 0) && (current_heading > 0)){ //if the target is negative and the position is positive
                if ((current_heading - target) >= 180){ //see if the error is greater than 180 degreees - we need to turn other way if so
                    target = target + 360; //take the target and add a full rotation to it so we now are turning the shorter direction
                    current_heading = global::imu.get_heading(); //update position
                }
            }
            else if ((target > 0) && (current_heading < 0)){ //if the target is positive and the position is negative
                if ((target - current_heading) >= 180){ //see if the error is greater than 180
                    target = target - 360; //add a full rotation to the left this time so we can turn the shorter direction
                    current_heading = global::imu.get_heading();
                }
            }

            pid.error = target - current_heading; //make sure the error is updated every single loop - regardless of whether the speed needs to be locked to 0 or not

            //go calculate speed with deadband in mind
            if (fabs(pid.error) < pid.deadband){
                pid.speed = 0; //lock the speed of the object to 0 - making sure nothing at all moves
            }
            else {
                pid.speed = pid.calculate(target, current_heading, 127);
            }

            //output speeds
            global::chassis.left.move(pid.speed);
            global::chassis.right.move(-pid.speed);

            //breaking out of the loop
            if (pid.has_settled()) break; //if the pid settled - break out
            if (drive_timer.has_elapsed()) break; //if the max amt of time has been reached - break out

            //chaining movements together
            if (chain_pos != LYNX_NULL && fabs(pid.error) <= chain_pos) break; //break out if we are the chaining position

            //flags stuff
            if (has_flag(passed_flags, flags::prt_error)){
                //if the function is passed with the "print error" flag
                global::con.print(0, 0, "turn_err: %f", pid.error);
            }

            pros::delay(5);

        }
    }

    void turn_relative(double target, std::optional<double> chain_pos = LYNX_NULL, std::optional<double> timeout = LYNX_NULL, lynx::PID& pid = turn_default, flags passed_flags = flags::none){

    }
}