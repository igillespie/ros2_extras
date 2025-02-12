#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <PiPCA9685/PCA9685.h>

namespace rover_arm {

class RoverArmHardware : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }


        // RCLCPP_INFO(rclcpp::get_logger("RoverArmHardware"), "Will look for parameters passed in");
        // // Dump joint information
        // for (size_t i = 0; i < info.joints.size(); ++i) {
        //     const auto &joint = info.joints[i];
        //     std::ostringstream joint_log;
        //     joint_log << "Joint[" << i << "]: " << joint.name << "\n";
        //     joint_log << "  Command Interfaces:";
        //     for (const auto &ci : joint.command_interfaces) {
        //         joint_log << " {name: " << ci.name << "}";
        //     }
        //     joint_log << "\n  State Interfaces:";
        //     for (const auto &si : joint.state_interfaces) {
        //         joint_log << " {name: " << si.name << "}";
        //     }
        //     joint_log << "\n  Parameters:";
        //     for (const auto &param : joint.parameters) {
        //         joint_log << " " << param.first << "=" << param.second;
        //     }
        //     RCLCPP_INFO(rclcpp::get_logger("RoverArmHardware"), "%s", joint_log.str().c_str());
        // }

        // Hard-code joint-to-servo channel mappings
        joint_to_channel_ = {
            {"joint1_base", 4}, //4 to test on the mast
            {"joint2_shoulder", 7},
            {"joint3_elbow", 8},
            {"joint4_wrist_pitch", 9},
            {"joint5_wrist_swivel", 10},
            {"joint6_claw", 11},
        };

        // Initialize PCA9685
        try {
            pca_.set_pwm_freq(50.0); // Set servo frequency to 50Hz
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("RoverArmHardware"), "Failed to initialize PCA9685: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Check if the joint counts match
        if (info.joints.size() != joint_to_channel_.size()) {
            RCLCPP_ERROR(
                rclcpp::get_logger("RoverArmHardware"),
                "Mismatch between URDF joint count (%zu) and hardware interface joint count (%zu).",
                info.joints.size(), joint_to_channel_.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize joint states and commands
        joints_.resize(info.joints.size());
        for (size_t i = 0; i < info.joints.size(); ++i) {
            joints_[i].position = 0.0;
            joints_[i].command = 0.0;

            // Load joint limits
            try {
                joints_[i].lower_limit = std::stod(info.joints[i].parameters.at("lower_limit"));
                joints_[i].upper_limit = std::stod(info.joints[i].parameters.at("upper_limit"));

                // Log successful loading of joint limits
                RCLCPP_INFO(
                    rclcpp::get_logger("RoverArmHardware"),
                    "Loaded limits for joint '%s': lower_limit=%.2f, upper_limit=%.2f",
                    info.joints[i].name.c_str(), joints_[i].lower_limit, joints_[i].upper_limit);
            } catch (const std::out_of_range &) {
                RCLCPP_WARN(
                    rclcpp::get_logger("RoverArmHardware"),
                    "Limits not specified for joint '%s', using default range [-1.0, 1.0].",
                    info.joints[i].name.c_str());
                joints_[i].lower_limit = -1.0;
                joints_[i].upper_limit = 1.0;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("RoverArmHardware"), "RoverArmHardware initialized successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < joints_.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "position", &joints_[i].position));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < joints_.size(); ++i) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "position", &joints_[i].command));
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
        RCLCPP_INFO(rclcpp::get_logger("RoverArmHardware"), "RoverArmHardware activated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
        RCLCPP_INFO(rclcpp::get_logger("RoverArmHardware"), "RoverArmHardware deactivated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
        // Simulate feedback by updating joint positions with the last commanded positions
        for (size_t i = 0; i < joints_.size(); ++i) {
            joints_[i].position = joints_[i].command;

            // Log the simulated position for debugging
            RCLCPP_DEBUG(rclcpp::get_logger("RoverArmHardware"),
                        "Joint[%zu] (%s): Position updated to %.2f",
                        i, info_.joints[i].name.c_str(), joints_[i].position);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
        // Iterate through the joints and send commands to the mapped servo channels
        for (size_t i = 0; i < joints_.size(); ++i) {
            int channel = joint_to_channel_[info_.joints[i].name]; // Get the channel from the mapping

            // Clamp the command within the joint limits
            double clamped_command = std::clamp(joints_[i].command, joints_[i].lower_limit, joints_[i].upper_limit);

            // Map the clamped command (radians) to pulse width (ms)
            double pulse_width = 1.0 + (clamped_command - joints_[i].lower_limit) /
                                        (joints_[i].upper_limit - joints_[i].lower_limit); // Maps to [1.0 ms, 2.0 ms]

            // Use set_pwm_ms to send the command as a proper pulse width
            pca_.set_pwm_ms(channel, pulse_width);

            // Log the command for debugging
            RCLCPP_DEBUG(rclcpp::get_logger("RoverArmHardware"),
                        "Sent PWM command to channel %d for joint %s: %.2f (clamped: %.2f, pulse_width: %.2f ms)",
                        channel, info_.joints[i].name.c_str(), joints_[i].command, clamped_command, pulse_width);
        }
        return hardware_interface::return_type::OK;
    }

private:
    struct Joint {
        double position = 0.0;
        double command = 0.0;
        double lower_limit = -1.0;
        double upper_limit = 1.0;
    };

    PiPCA9685::PCA9685 pca_;
    std::unordered_map<std::string, int> joint_to_channel_;
    std::vector<Joint> joints_;
};

} // namespace rover_arm

PLUGINLIB_EXPORT_CLASS(rover_arm::RoverArmHardware, hardware_interface::SystemInterface)