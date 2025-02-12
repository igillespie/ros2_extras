#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <PiPCA9685/PCA9685.h>

class PCA9685Hardware : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Check if mapping file is provided
        if (info.hardware_parameters.find("pca9685_joint_map") != info.hardware_parameters.end()) {
            std::string mapping_file = info.hardware_parameters.at("pca9685_joint_map");
            if (!load_mapping_from_file(mapping_file)) {
                return hardware_interface::CallbackReturn::ERROR;
            }
        } else if (info.hardware_parameters.find("joints_to_servo_channels") != info.hardware_parameters.end()) {
            if (!load_mapping_inline(info.hardware_parameters.at("joints_to_servo_channels"))) {
                return hardware_interface::CallbackReturn::ERROR;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("PCA9685Hardware"), "No joint mapping found!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize joint states
       for (const auto &[joint, channel] : joint_to_channel_) {
            (void)channel; // Explicitly mark the channel as unused to suppress warnings
            joint_positions_.push_back(0.0);
            joint_commands_.push_back(0.0);
        }

        // Initialize PCA9685
        try {
            pca_.set_pwm_freq(60.0);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("PCA9685Hardware"), "Failed to initialize PCA9685: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(rclcpp::get_logger("PCA9685Hardware"), "Configuring PCA9685 hardware");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(rclcpp::get_logger("PCA9685Hardware"), "Activating PCA9685 hardware");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(rclcpp::get_logger("PCA9685Hardware"), "Deactivating PCA9685 hardware");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
        for (size_t i = 0; i < joint_positions_.size(); ++i) {
            joint_positions_[i] = joint_commands_[i]; // Mocking read behavior
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
        for (size_t i = 0; i < joint_commands_.size(); ++i) {
            int channel = joint_to_channel_.at(info_.joints[i].name);
            // Scale the joint command (0.0 to 1.0) to the PWM range (500 to 2500)
            int pwm_value = static_cast<int>(500 + joint_commands_[i] * (2500 - 500));
            pca_.set_pwm(channel, 0, pwm_value);
        }
        return hardware_interface::return_type::OK;
    }

private:
    PiPCA9685::PCA9685 pca_;
    std::unordered_map<std::string, int> joint_to_channel_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_commands_;

    bool load_mapping_from_file(const std::string &path) {
        try {
            YAML::Node yaml_map = YAML::LoadFile(path);

            for (const auto &entry : yaml_map) {
                std::string joint_name = entry.first.as<std::string>();
                int channel = entry.second.as<int>();
                joint_to_channel_[joint_name] = channel;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("PCA9685Hardware"), "Error loading mapping file: %s", e.what());
            return false;
        }
        return true;
    }

    bool load_mapping_inline(const std::string &inline_map) {
        try {
            YAML::Node yaml_map = YAML::Load(inline_map);

            for (const auto &entry : yaml_map) {
                std::string joint_name = entry.first.as<std::string>();
                int channel = entry.second.as<int>();
                joint_to_channel_[joint_name] = channel;
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("PCA9685Hardware"), "Error parsing inline mapping: %s", e.what());
            return false;
        }
        return true;
    }
};

PLUGINLIB_EXPORT_CLASS(PCA9685Hardware, hardware_interface::SystemInterface)