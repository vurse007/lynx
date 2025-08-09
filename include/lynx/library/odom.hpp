#pragma once
#include <cmath>
#include <algorithm>
#include "config.hpp"

namespace lynx
{   
    namespace odom{

        double prev_horizontal_encoder;
        double prev_vertical_encoder;
        double prev_theta;

        double horizontal_encoder;
        double vertical_encoder;
        double theta;

        double pod_ticks_to_inches(double ticks){
            //convert ticks to inches
            return (ticks/36000)*2*M_PI*global::chassis.odom_wheel_diameter;
        }
        double degrees_to_radians(double degrees){
            return degrees * M_PI / 180.0;
        }
        double radians_to_degrees(double radians){
            return radians * 180.0 / M_PI;
        }

        class point {
            public:
                double x;
                double y;

                double theta = std::numeric_limits<double>::quiet_NaN();

            point(double x, double y, double theta = std::numeric_limits<double>::quiet_NaN())
                : x(x), y(y), theta(theta) {}

            //helper functions for odom calculations
            double distance_to(const point& other)const { //const is a promise to not modify the object
                double deltaX = other.x - x;
                double deltaY = other.y - y;

                return sqrt(deltaX*deltaX + deltaY*deltaY);
            }
            double angle_error(const point& other) const { //fix expected an expression
                return other.theta - theta;
            }
        };

        point current_pos(0.0,0.0,0.0);

        void reset(double new_x=0.0, double new_y=0.0, std::optional<double> new_theta = LYNX_NULL){
            // reset to specified position
            current_pos.x = new_x;
            current_pos.y = new_y;
            current_pos.theta = new_theta.value_or(global::imu.get_heading());

            //reset previous values to current values
            prev_horizontal_encoder = horizontal_encoder;
            prev_vertical_encoder = vertical_encoder;
            prev_theta = current_pos.theta;
        }

        void read_sensors(pros::Rotation horizontal, pros::Rotation vertical, pros::IMU inertial){
            //read the sensors and update the global variables
            horizontal_encoder = horizontal.get_position();
            vertical_encoder = vertical.get_position();
            theta = inertial.get_heading();
        }

        void update(){
            //step 1: get current and previous theta from imu
            prev_theta = current_pos.theta;
            current_pos.theta = degrees_to_radians(theta);

            //step 2: calculate encoder deltas
            double dH = pod_ticks_to_inches(horizontal_encoder) - prev_horizontal_encoder;
            double dV = pod_ticks_to_inches(vertical_encoder) - prev_vertical_encoder;
            //the encoder variables are all in the units of inches
            //using the pod ticks conversion function to convert the input (in ticks)
            //into inches before dealing with prev encoder values

            //step 3: calculate angle change
            double dTheta = util::wrap_to_pi(current_pos.theta - prev_theta);

            //step 4: apply offset corrections
            double corrected_vertical = dV - (global::chassis.odom_vertical_offset * dTheta);
            double corrected_horizontal = dH - (global::chassis.odom_horizontal_offset * dTheta);

            //step 5: calculate average angle
            double avg_theta = (prev_theta + current_pos.theta)/2;
            //we are using the average of the angle during the movement for more
            //accurate coordinate transformations

            //step 6: transform to global coordinates
            double cos_avg = std::cos(avg_theta);
            double sin_avg = std::sin(avg_theta);
            double global_dx = corrected_horizontal * cos_avg - corrected_vertical * sin_avg;
            double global_dy = corrected_horizontal * sin_avg + corrected_vertical * cos_avg;

            //step 7: update global position
            current_pos.x += global_dx;
            current_pos.y += global_dy;

            //step 8: update previous values
            prev_horizontal_encoder = horizontal_encoder;
            prev_vertical_encoder = vertical_encoder;
        }
    }
}