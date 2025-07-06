#pragma once
#include <cmath>
#include <algorithm>
#include "config.hpp"

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

            static double global_target_heading; //shared across all pids

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

    void turn_absolute(double target, double speed_lim = 127, std::optional<double> chain_pos = LYNX_NULL, std::optional<double> timeout = LYNX_NULL, lynx::PID& pid = turn_default, flags passed_flags = flags::none){
        target = target + chain_pos.value_or(0); //add the chain position to the target - add 0 if chaining is off
        
        pid.global_target_heading = target; //set the global target heading to the target that we passed in
        
        //do absolute calculations
        double position = lynx::util::wrap_to_180(global::imu.get_heading()); //get the current heading and wrap it to -180 to 180

        lynx::util::absolute_logic(position, target); //use the absolute logic function to make sure we are turning the shortest direction
        double net_turn = target - position; //find the net turn distance - do fabs to get magnitude because it is signed on direction

        pid.reset();

        //find the timeout and set it
        turn_timer.target_time = timeout.value_or(turn_timeout.evaluate(fabs(net_turn)));
        turn_timer.restart();

        while (true){

            //get the current heading and do a bit of math ;D
            double current_heading = lynx::util::wrap_to_180(global::imu.get_heading()); //get the current heading and wrap it to -180 to 180
            lynx::util::absolute_logic(current_heading, target); //use the absolute logic function to make sure we are turning the shortest direction

            //pid.error = target - current_heading; //make sure the error is updated every single loop - regardless of whether the speed needs to be locked to 0 or not

            //go calculate speed with deadband in mind
            if (fabs(pid.error) < pid.deadband){
                pid.speed = 0; //lock the speed of the object to 0 - making sure nothing at all moves
            }
            else {
                pid.speed = pid.calculate(target, current_heading, speed_lim);
            }

            //output speeds
            global::chassis.left.move(pid.speed);
            global::chassis.right.move(-pid.speed);

            //breaking out of the loop
            if (pid.has_settled()) break; //if the pid settled - break out
            if (turn_timer.has_elapsed()) break; //if the max amt of time has been reached - break out

            //chaining movements together
            if (chain_pos != LYNX_NULL && fabs(pid.error) <= chain_pos) break; //break out if we are the chaining position

            //flags stuff
            if (has_flag(passed_flags, flags::prt_error)){
                //if the function is passed with the "print error" flag
                global::con.print(0, 0, "turn_err: %f", pid.error);
            }
            if (has_flag(passed_flags, flags::prt_time)){
                //if the function is passed with the "print time" flag
                global::con.print(1, 0, "time: %d", turn_timer.elapsed());
            }

            pros::delay(5);
        }

        global::chassis.move(0, 0);
    
    }

    void turn_relative(double target, double speed_lim = 127, std::optional<double> chain_pos = LYNX_NULL, std::optional<double> timeout = LYNX_NULL, lynx::PID& pid = turn_default, flags passed_flags = flags::none){
        //find the target relatively
        double position = lynx::util::wrap_to_180(global::imu.get_heading()); //get the current heading and wrap it to -180 to 180
        target = position + target;

        target = target + chain_pos.value_or(0); //add the chain position to the target - add 0 if chaining is off

        pid.global_target_heading = lynx::util::wrap_to_180(target); //set the global target heading to the target that we passed in, wrapped to -180 to 180

        double net_turn = target - position; //find the net turn distance - do fabs to get magnitude because it is signed on direction

        pid.reset();

        //find the timeout and set it
        turn_timer.target_time = timeout.value_or(turn_timeout.evaluate(fabs(net_turn)));
        turn_timer.restart();

        while (true){
            //get the current heading
            double current_heading = lynx::util::wrap_to_180(global::imu.get_heading()); //get the current heading and wrap it to -180 to 180

            //pid.error = target - current_heading;

            //go calculate speed with deadband in mind
            if (fabs(pid.error) < pid.deadband){
                pid.speed = 0; //lock the speed of the object to 0 - making sure nothing at all moves
            }
            else {
                pid.speed = pid.calculate(target, current_heading, speed_lim);
            }

            //output speeds
            global::chassis.left.move(pid.speed);
            global::chassis.right.move(-pid.speed);

            //breaking out of the loop
            if (pid.has_settled()) break; //if the pid settled - break out
            if (turn_timer.has_elapsed()) break; //if the max amt of time has been reached - break out

            //chaining movements together
            if (chain_pos != LYNX_NULL && fabs(pid.error) <= chain_pos) break; //break out if we are the chaining position

            //flags stuff
            if (has_flag(passed_flags, flags::prt_error)){
                //if the function is passed with the "print error" flag
                global::con.print(0, 0, "turn_err: %f", pid.error);
            }
            if (has_flag(passed_flags, flags::prt_time)){
                //if the function is passed with the "print time" flag
                global::con.print(1, 0, "time: %d", turn_timer.elapsed());
            }

            pros::delay(5);
        }

        global::chassis.move(0,0);
    }

    void drive(double target, double speed_lim = 127, std::optional<double> chain_pos = LYNX_NULL, std::optional<double> timeout = LYNX_NULL, lynx::PID& pid = drive_default, lynx::PID& heading_correction = heading_correction_default, flags passed_flags = flags::none){

        target = target + chain_pos.value_or(0); //add the chain position to the target - add 0 if chaining is off

        //find start position of the left and right sides
        double init_left_pos = global::chassis.left.get_avg_pos();
        double init_right_pos = global::chassis.right.get_avg_pos();

        //find relative target by adding to the initial positions
        double relative_left_target = init_left_pos + target;
        double relative_right_target = init_right_pos + target;

        pid.reset();
        heading_correction.reset();

        //find the timeout and set it
        drive_timer.target_time = timeout.value_or(drive_timeout.evaluate(fabs(target)));
        drive_timer.restart();

        while(true){
            double position = lynx::util::wrap_to_180(global::imu.get_heading()); //get the current heading and wrap it to -180 to 180
            lynx::util::absolute_logic(position, pid.global_target_heading); //do absolute logic function to optimize turn direction to the target heading

            if (!has_flag(passed_flags, flags::hc_off)){ //if it doesnt have the hcoff flag, hc is on - run the heading correction pid calcs
                //heading_correction.error = pid.global_target_heading - position; //find the error for heading correction
                heading_correction.speed = heading_correction.calculate(pid.global_target_heading, position, speed_lim);
            }

            //drive calculations
            double avg_left_pos = global::chassis.left.get_avg_pos();
            double avg_right_pos = global::chassis.right.get_avg_pos();

            double left_error = relative_left_target - avg_left_pos; //find the error for both sides for printing purposes
            double right_error = relative_right_target - avg_right_pos;

            double left_speed = pid.calculate(relative_left_target, avg_left_pos, speed_lim); //calculate speed for both sides
            double right_speed = pid.calculate(relative_right_target, avg_right_pos, speed_lim); //since the curr pos of both sides is different calculating two speeds could be safer - but in most cases right and left speed will be approaching the same number

            //output speeds
            global::chassis.left.move(left_speed + heading_correction.speed);
            global::chassis.right.move(right_speed - heading_correction.speed);

            //breaking out of the loop
            if (pid.has_settled() && heading_correction.has_settled()) break; //both the straight pid and the heading correction pid must be settled to break out of the loop
            if (drive_timer.has_elapsed()) break; //if the max amount of time has been reached, break out of the loop

            //chaining movements together
            if (chain_pos != LYNX_NULL && (fabs(left_error) + fabs(right_error))/2 <= chain_pos) break; //break out if the chain position is reached

            //checking remaining flags
            if (has_flag(passed_flags, flags::prt_error)){
                global::con.print(0,0, "error: %f", (left_error + right_error)/2); //if they passed in the print error flag, print on the controller
                global::con.print(2,0, "hc err: %f", heading_correction.error);
            }
            if (has_flag(passed_flags, flags::prt_time)){
                global::con.print(1,0, "time: %d", drive_timer.elapsed()); //if they passed in the print time flag, print on the controller
            }

            pros::delay(5);
        }
        global::chassis.move(0,0);
    }

    void arc_right(double target, double radius, double speed_lim = 127, std::optional<double> chain_pos = LYNX_NULL, std::optional<double> timeout = LYNX_NULL, lynx::PID& pid = arc_default, lynx::PID& heading_correction = heading_correction_default, double nudge_magnitude = global::immutables::DEFAULT_NUDGE_MAGNITUDE, flags passed_flags = flags::none){
        double passed_target = target;
        target = target + chain_pos.value_or(0); //add the chain position to the target and save the original target for breaking out

        double right_arc_length = (target/360) * 2 * M_PI * (radius + global::chassis.track_width/2); //find the length of the right arc
        double left_arc_length = (target/360) * 2 * M_PI * (radius - global::chassis.track_width/2); //find the length of the left arc

        double speed_ratio = right_arc_length / left_arc_length; //find the speed ratio between the right and left sides

        double init_left_pos = global::chassis.left.get_avg_pos(); //find the initial position of the left side
        double init_right_pos = global::chassis.right.get_avg_pos(); //find the initial position of the right side

        double relative_left_target = init_left_pos + left_arc_length; //find the relative target for the left side
        double relative_right_target = init_right_pos + right_arc_length; //find the relative target for the right side

        double init_heading = lynx::util::wrap_to_180(global::imu.get_heading()); //get the initial heading and wrap it to -180 to 180
        double unwrapped_init_heading = global::imu.get_heading(); //get the unwrapped initial heading

        pid.reset();
        heading_correction.reset();

        //find the timeout and set it
        arc_timer.target_time = timeout.value_or(arc_timeout.evaluate(fabs(target)));
        arc_timer.restart();

        while (true){
            double current_right_pos = global::chassis.right.get_avg_pos(); //current avg pos of right side
            double current_left_pos = global::chassis.left.get_avg_pos(); //current avg pos of left side

            double left_error = relative_left_target - current_left_pos; //find the error for printing purposes
            double right_error = relative_right_target - current_right_pos;

            double correct_heading = ((((current_left_pos - init_left_pos)+(current_right_pos - init_right_pos))/2)*360) / (2*M_PI*radius); //take the travelled distance and plug into formula to find the correct heading

            double heading = lynx::util::wrap_to_180(init_heading + correct_heading);
            lynx::util::absolute_logic(heading, correct_heading);

            if (!has_flag(passed_flags, flags::hc_off)){
                heading_correction.speed = heading_correction.calculate(init_heading+correct_heading, heading, speed_lim); //run pid on the heading correction values
            }

            double nudge_speed = pid.calculate(relative_right_target, current_right_pos, nudge_magnitude);

            double left_speed = pid.calculate(relative_left_target, current_left_pos, speed_lim); //calculate the right and left speeds before heading correction and nudge
            double right_speed = left_speed * speed_ratio;

            //output speeds with nudge and heading correction
            global::chassis.left.move(left_speed + heading_correction.speed);
            global::chassis.right.move((right_speed - heading_correction.speed) + nudge_speed);

            //breaking out of the loop
            if (pid.has_settled() && heading_correction.has_settled()) break;
            if (arc_timer.has_elapsed()) break;

            //chaining
            if (chain_pos != LYNX_NULL && (fabs(heading - init_heading) >= passed_target)) break;

            //check the rest of the flags
            if (has_flag(passed_flags, flags::prt_error)){
                global::con.print(0,0, "L:%f, R:%f", left_error, right_error);
                global::con.print(2,0, "hc err: %f", heading_correction.error);
            }
            if (has_flag(passed_flags, flags::prt_time)){
                global::con.print(1,0, "time: %d", arc_timer.elapsed());
            }
            
            pros::delay(5);
        }
        global::chassis.move(0,0);
        pid.global_target_heading = lynx::util::wrap_to_180(unwrapped_init_heading + passed_target);
    }

    void arc_left(double target, double radius, double speed_lim = 127, std::optional<double> chain_pos = LYNX_NULL, std::optional<double> timeout = LYNX_NULL, lynx::PID& pid = arc_default, lynx::PID& heading_correction = heading_correction_default, double nudge_magnitude = global::immutables::DEFAULT_NUDGE_MAGNITUDE, flags passed_flags = flags::none){
        double passed_target = target;
        target = target + chain_pos.value_or(0); //add chain pos to target and save og target for breaking out

        double left_arc_length = (target/360) * 2 * M_PI * (radius + global::chassis.track_width/2); //left arc
        double right_arc_length = (target/360) * 2 * M_PI * (radius - global::chassis.track_width/2); //right arc wider than left arc

        double speed_ratio = left_arc_length/right_arc_length; //find speed ratio

        double init_left_pos = global::chassis.left.get_avg_pos(); //find the initial position of the left side
        double init_right_pos = global::chassis.right.get_avg_pos(); //find the initial pos for the right side

        double relative_left_target = init_left_pos + left_arc_length;
        double relative_right_target = init_right_pos + right_arc_length; //find the relative targets

        double init_heading = lynx::util::wrap_to_180(global::imu.get_heading()); //get init heading and wrap to -180 to 180
        double unwrapped_init_heading = global::imu.get_heading(); //save unwrapped init heading

        pid.reset();
        heading_correction.reset();

        //find the timeout and set it
        arc_timer.target_time = timeout.value_or(arc_timeout.evaluate(fabs(target)));
        arc_timer.restart();

        while (true) {
            double current_right_pos = global::chassis.right.get_avg_pos(); //current avg pos of r
            double current_left_pos = global::chassis.left.get_avg_pos(); //current avg pos of l

            double left_error = relative_left_target - current_left_pos; //find error for printing
            double right_error = relative_right_target - current_right_pos;

            double correct_heading = ((((current_left_pos - init_left_pos) + (current_right_pos - init_right_pos)) / 2) * 360) / (2 * M_PI * radius); //take the travelled distance and plug into formula to find the correct heading

            double heading = lynx::util::wrap_to_180(init_heading + correct_heading);
            lynx::util::absolute_logic(heading, correct_heading);

            if (!has_flag(passed_flags, flags::hc_off)){
                heading_correction.speed = heading_correction.calculate(init_heading + correct_heading, heading, speed_lim); //run pid on the heading correction values
            }

            double nudge_speed = pid.calculate(relative_left_target, current_left_pos, nudge_magnitude);

            double left_speed = pid.calculate(relative_left_target, current_left_pos, speed_lim); //calculate the right and left speeds before heading correction and nudge
            double right_speed = left_speed / speed_ratio;

            //output speeds with nudge and heading correction
            global::chassis.left.move((left_speed + heading_correction.speed) + nudge_speed);
            global::chassis.right.move(right_speed - heading_correction.speed);

            //breaking out of the loop
            if (pid.has_settled() && heading_correction.has_settled()) break;
            if (arc_timer.has_elapsed()) break;

            //chaining
            if (chain_pos != LYNX_NULL && (fabs(heading - init_heading) >= passed_target)) break;

            //check the rest of the flags
            if (has_flag(passed_flags, flags::prt_error)){
                global::con.print(0,0, "L:%f, R:%f", left_error, right_error);
                global::con.print(2,0, "hc err: %f", heading_correction.error);
            }
            if (has_flag(passed_flags, flags::prt_time)){
                global::con.print(1,0, "time: %d", arc_timer.elapsed());
            }

            pros::delay(5);
        }
        global::chassis.move(0,0);
        pid.global_target_heading = lynx::util::wrap_to_180(unwrapped_init_heading + passed_target);
    }
}