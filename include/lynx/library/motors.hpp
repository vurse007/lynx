#pragma once
#include "main.h"
#include <vector>
#include <memory>

namespace lynx{
    // Structure to hold specs for a single motor
    struct motor_specs {
        int port;
        pros::motor_gearset_e gearset;
        bool reverse;

        motor_specs(int port, pros::motor_gearset_e gearset, bool reverse)
            : port(port), gearset(gearset), reverse(reverse) {}
    };

    // Class to control a group of motors as one unit
    class group {
    private:
        std::vector<std::shared_ptr<pros::Motor>> motors;

    public:
        group(const std::vector<motor_specs>& motor_specs) {
            for (const auto& spec : motor_specs) {
                auto motor = std::make_shared<pros::Motor>(spec.port, spec.gearset, spec.reverse);
                motors.push_back(motor);
            }
        }

        void set_brake_mode(pros::motor_brake_mode_e_t mode) {
            for (auto& motor : motors) {
                motor->set_brake_mode(mode);
            }
        }

        void move(int velocity) {
            for (auto& motor : motors) {
                motor->move(velocity);
            }
        }

        void tare() {
            for (auto& motor : motors) {
                motor->tare_position();
            }
        }

        std::shared_ptr<pros::Motor> get_motor(int index) const {
            if (index >= 0 && index < motors.size()) {
                return motors[index];
            }
            return nullptr;
        }
    };

    //if something out of the ordinary needs to be done to the motors
    inline void apply_to_group(group& motor_group, const std::function<void(std::shared_ptr<pros::Motor>)>& func) {
        for (int i = 0; ; ++i) {
            auto motor = motor_group.get_motor(i);
            if (!motor) break;
            func(motor);
        }
    }

    // chassis class that wraps left and right sides
    class chassis {
    public:

        group left;
        group right;

        const double wheel_diameter;
        const double external_gear_ratio;
        const double track_width;

        chassis(const std::vector<motor_specs>& left_specs, const std::vector<motor_specs>& right_specs, const double wheel_diam, const double ext_gr, const double track_wid)
            : left(left_specs), right(right_specs), wheel_diameter(wheel_diam), external_gear_ratio(ext_gr), track_width(track_wid) {}

        
        // helper functions to do common tasks to motors
        void set_brake_mode(pros::motor_brake_mode_e_t mode) {
            left.set_brake_mode(mode);
            right.set_brake_mode(mode);
        }

        void move(int left_velocity, int right_velocity) {
            left.move(left_velocity);
            right.move(right_velocity);
        }

        void tare() {
            left.tare();
            right.tare();
        }

        void apply_to_chassis(const std::function<void(std::shared_ptr<pros::Motor>)>& func) {
            for (int i = 0; ; ++i) {
                auto leftmotor = left.get_motor(i);
                auto rightmotor = right.get_motor(i);
        
                // Break the loop if both motors are null (end of both sides)
                if (!leftmotor && !rightmotor) break;
        
                // Apply func to the left motor if it exists
                if (leftmotor) {
                    func(leftmotor);
                }
        
                // Apply func to the right motor if it exists
                if (rightmotor) {
                    func(rightmotor);
                }
            }
        }

        //sample use case:
        // my_chassis.apply_to_group(my_chassis.left, [](std::shared_ptr<pros::Motor> motor){ motor->set_voltage_limit(12000); });
        //get rid of my_chassis.left to apply to whole chassis
        //second parameter is a lambda function, needs to have a motor pointer as parameter, to use when applying the function
    };
}