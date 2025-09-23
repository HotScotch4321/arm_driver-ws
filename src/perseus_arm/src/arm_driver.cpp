#include "perseus_arm/arm_driver.hpp"
#include <pigpiod_if2.h>
#include <cmath>
#include <algorithm>

namespace perseus_arm {

    // Constructor: Initialize connection to -1 (disconnected state)
    mg996Rdriver::mg996Rdriver() : pigpio_connection_(-1) {
    }

    // Destructor: Ensure proper cleanup
    mg996Rdriver::~mg996Rdriver() {
        disconnectFromPigpio();
    }

    // Sets up the configuration for each joint (name, pin, angle limits, pulse width range)
    void mg996Rdriver::setupjointconfig() {
        joint_configs_ = {
            {"base", 12, -M_PI_2, M_PI_2, 500, 2500},      
            {"shoulder", 13, -M_PI_2, M_PI_2, 500, 2500},  
            {"elbow", 14, -M_PI_2, M_PI_2, 500, 2500}      
        };
    }

    // Connects to the pigpio daemon. Returns true if connection is successful, false otherwise.
    bool mg996Rdriver::connectToPigpio() {
        pigpio_connection_ = pigpio_start(NULL, NULL);
        if (pigpio_connection_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("mg996Rdriver"), "Failed to connect to pigpiod.");
            return false;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("mg996Rdriver"), "Connected to pigpiod successfully.");
            return true;
        }
    }

    // Disconnects from the pigpio daemon if currently connected.
    void mg996Rdriver::disconnectFromPigpio() {
        if (pigpio_connection_ >= 0) {
            pigpio_stop(pigpio_connection_);
            pigpio_connection_ = -1;
        }
    }

    // Stops all servos by sending a pulse width of 0 to each joint's pin.
    void mg996Rdriver::stopAllServos() {
        for (const auto& joint : joint_configs_) {
            sendPulseWidth(joint.pin, 0); // 0 pulse width stops the servo
        }
    }

    // Converts a given angle (in radians) to the corresponding pulse width for a specific joint.
    // The angle is normalized between the joint's min and max angle, then mapped to the pulse width range.
    int mg996Rdriver::angleToPulseWidth(double angle, const JointConfig& joint) const {
        double normalised = (angle - joint.min_angle) / (joint.max_angle - joint.min_angle);
        double angle_normalise = std::clamp(normalised, 0.0, 1.0); 

        return joint.pw_min + static_cast<int>(angle_normalise * (joint.pw_max - joint.pw_min));
    }

    // Sends the specified pulse width to the servo on the given GPIO pin using pigpio.
    void mg996Rdriver::sendPulseWidth(int pin, int pulseWidth) {
        int result = set_servo_pulsewidth(pigpio_connection_, pin, pulseWidth);

        if (result < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("mg996Rdriver"), "Failed to send pulse width to pin %d", pin);
        }
    }

    // Called during hardware initialization. Sets up joint configuration and initializes state/command vectors.
    hardware_interface::CallbackReturn mg996Rdriver::on_init(
        const hardware_interface::HardwareInfo & info) {
            // Call base class initialization and check for success
            if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Set up joint configuration parameters
            setupjointconfig();

            // Initialize command and state vectors for each joint
            position_commands_.resize(joint_configs_.size(), 0.0);
            position_states_.resize(joint_configs_.size(), 0.0);

            RCLCPP_INFO(rclcpp::get_logger("mg996Rdriver"), "mg996Rdriver initialized successfully.");
            return hardware_interface::CallbackReturn::SUCCESS;
        }
    
    
    // Called when the hardware is being configured. Attempts to connect to pigpiod daemon.
    hardware_interface::CallbackReturn mg996Rdriver::on_configure(
        const rclcpp_lifecycle::State & previous_state) {
            // Try to connect to pigpiod daemon, log result
            if (!connectToPigpio()) {
                RCLCPP_ERROR(rclcpp::get_logger("mg996Rdriver"), "Failed to connect to pigpiod.");
                return hardware_interface::CallbackReturn::ERROR;
            }
            else {
                RCLCPP_INFO(rclcpp::get_logger("mg996Rdriver"), "Connected to pigpiod successfully.");
            }
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    // Called when the hardware interface is activated (e.g., when starting control).
    hardware_interface::CallbackReturn mg996Rdriver::on_activate(
        const  rclcpp_lifecycle::State& previous_state) {
            RCLCPP_INFO(rclcpp::get_logger("mg996Rdriver"), "mg996Rdriver activated.");
            return hardware_interface::CallbackReturn::SUCCESS;
        }
    
    // Called when the hardware interface is deactivated (e.g., when stopping control).
    hardware_interface::CallbackReturn mg996Rdriver::on_deactivate(
        const rclcpp_lifecycle::State & previous_state) {
            stopAllServos();
            RCLCPP_INFO(rclcpp::get_logger("mg996Rdriver"), "mg996Rdriver deactivated.");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    // Called to clean up resources before shutdown or reconfiguration.
    hardware_interface::CallbackReturn mg996Rdriver::on_cleanup(
        const rclcpp_lifecycle::State & previous_state) {
            // Disconnect from pigpiod daemon
            disconnectFromPigpio();
            RCLCPP_INFO(rclcpp::get_logger("mg996Rdriver"), "Disconnected from pigpiod.");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    // Called when the hardware is being shut down. Cleans up resources.
    hardware_interface::CallbackReturn mg996Rdriver::on_shutdown(
        const rclcpp_lifecycle::State & previous_state) {
            // Disconnect from pigpiod daemon
            disconnectFromPigpio();
            RCLCPP_INFO(rclcpp::get_logger("mg996Rdriver"), "Disconnected from pigpiod.");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

    // Called when an error occurs in the hardware interface.
    hardware_interface::CallbackReturn mg996Rdriver::on_error(
        const rclcpp_lifecycle::State & previous_state) {
            RCLCPP_ERROR(rclcpp::get_logger("mg996Rdriver"), "mg996Rdriver error occurred.");
            return hardware_interface::CallbackReturn::ERROR;
        }

    // Exports state interfaces for each joint (position only).
    // These interfaces allow the controller to read the current state of each joint.
    std::vector<hardware_interface::StateInterface> mg996Rdriver::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < joint_configs_.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
        }
        return state_interfaces;
    }

    // Exports command interfaces for each joint (position command).
    // These interfaces allow the controller to send position commands to each joint.
    std::vector<hardware_interface::CommandInterface> mg996Rdriver::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < joint_configs_.size(); ++i) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        }
        return command_interfaces;
    }

    // Reads the current state from the hardware.
    hardware_interface::return_type mg996Rdriver::read(
        const rclcpp::Time & time, const rclcpp::Duration & period) {
        
        for (size_t i = 0; i < joint_configs_.size(); ++i) {
            // Validate position before publishing
            if (std::isfinite(position_commands_[i])) {
                position_states_[i] = position_commands_[i];
            } else {
                position_states_[i] = 0.0;  
            }
        }
        return hardware_interface::return_type::OK;
    }
    
    // Writes the current position commands to the hardware.
    // Clamps the command to the joint limits, converts to pulse width, and sends to each servo.
    hardware_interface::return_type mg996Rdriver::write(
        const rclcpp::Time & time, const rclcpp::Duration & period) {
        
        RCLCPP_INFO(rclcpp::get_logger("mg996Rdriver"), 
            "WRITE CALLED - Base: %.3f, Shoulder: %.3f, Elbow: %.3f", 
            position_commands_[0], position_commands_[1], position_commands_[2]);
        
        for (size_t i = 0; i < joint_configs_.size(); ++i) {
            const auto& joint = joint_configs_[i];
            double clamped_position = std::clamp(position_commands_[i], joint.min_angle, joint.max_angle);
            int pulse_width = angleToPulseWidth(clamped_position, joint);
            
            RCLCPP_INFO(rclcpp::get_logger("mg996Rdriver"), 
                "Joint %s: cmd=%.3f → pw=%d", joint.name.c_str(), clamped_position, pulse_width);
                
            sendPulseWidth(joint.pin, pulse_width);
        }
        return hardware_interface::return_type::OK;
    }

} 

// Export the hardware interface plugin 
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(perseus_arm::mg996Rdriver, hardware_interface::SystemInterface)
