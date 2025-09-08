#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>
#include <string>
#include <memory>

namespace perseus_arm {

/**
 * @brief Configuration structure for a single servo joint
 * 
 * Contains all parameters needed to control one MG996R servo motor including:
 * - ROS2 joint identification (name must match URDF)
 * - Hardware connection (GPIO pin number)
 * - Physical limits (angle range in radians)  
 * - PWM signal parameters (pulse width range in microseconds)
 */
struct JointConfig {
    std::string name;   // Joint name (e.g., "shoulder_joint") - must match URDF joint names
    int pin;            // GPIO pin number connected to the servo signal wire
    double min_angle;   // Minimum allowed angle (radians) - prevents mechanical damage
    double max_angle;   // Maximum allowed angle (radians) - prevents mechanical damage
    int  pw_min;        // Minimum pulse width (microseconds) corresponding to min_angle
    int  pw_max;        // Maximum pulse width (microseconds) corresponding to max_angle
};

/**
 * @brief ROS2 Hardware Interface for MG996R servo motors via pigpiod
 * 
 * Controls MG996R servo motors (180-degree range) on a Raspberry Pi using pigpiod.
 * 
 * Key features:
 * - Supports 180-degree servo positioning
 * - Angle range: ±90 degrees (-π/2 to π/2 radians)
 */
class mg996Rdriver : public hardware_interface::SystemInterface
{
    public:

        /**
         * @brief Constructor: Initialize the hardware interface in a safe disconnected state
         * 
         * Sets pigpio_connection_ to -1 to indicate no active connection to the pigpiod daemon.
         * This ensures we start in a known state and can detect if connection is established later.
         */
        mg996Rdriver();

        /**
         * @brief Destructor: Ensure proper cleanup when object is destroyed
         * 
         * Automatically disconnects from pigpiod daemon if still connected. This prevents
         * resource leaks and ensures servos are safely stopped when the driver shuts down
         * unexpectedly (e.g., program crash, Ctrl+C, etc.).
         */
        ~mg996Rdriver();

        /**
         * @brief ROS2 Lifecycle: Initialize the hardware interface
         * 
         * Called once during system startup to prepare the hardware interface.
         * This is part of ROS2's managed lifecycle - the controller_manager calls
         * this before any control operations begin.
         * 
         * Steps performed:
         * 1. Call parent class initialization
         * 2. Setup joint configurations (GPIO pins, limits, etc.)
         * 3. Initialize state vectors for position tracking
         * 4. Prepare the interface for configuration phase
         * 
         * @param info Hardware information from URDF/config files
         * @return SUCCESS if initialization completed
         */
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;

        /**
         * @brief ROS2 Lifecycle: Configure hardware connections and resources
         * 
         * Called after initialization to establish actual hardware connections.
         * This is when we connect to the pigpiod daemon and prepare for active control.
         * The controller_manager calls this when transitioning from "unconfigured" to "inactive".
         * 
         * Critical step: Establishes the connection needed for all subsequent servo operations.
         * If this fails, the hardware interface cannot control any servos.
         * 
         * @param previous_state The lifecycle state we're transitioning from
         * @return SUCCESS if configuration completed, ERROR if connection failed
         * 
         * ROS2 Context: Second step in lifecycle - prepares hardware for activation
         */
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        /**
         * @brief ROS2 Lifecycle: Activate hardware for control operations
         * 
         * Called when the controller_manager wants to start active control.
         * Transitions from "inactive" to "active" state, enabling the read/write cycle.
         * After this, the controller will begin calling read() and write() methods continuously.
         * 
         * At this point, servos are ready to receive commands from MoveIt or other controllers.
         * 
         * @param previous_state The lifecycle state we're transitioning from  
         * @return SUCCESS (always succeeds - servos are already configured)
         */
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        /**
         * @brief ROS2 Lifecycle: Deactivate hardware and stop control operations
         * 
         * Called when the controller_manager wants to stop active control.
         * Transitions from "active" to "inactive" state, stopping the read/write cycle.
         * 
         * Critical safety step: Immediately stops all servos to prevent uncontrolled movement.
         * This is called during normal shutdown or when switching controllers.
         * 
         * @param previous_state The lifecycle state we're transitioning from
         * @return SUCCESS after safely stopping all servos
         */
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        /**
         * @brief ROS2 Lifecycle: Clean up resources and prepare for reconfiguration
         * 
         * Called when transitioning from "inactive" to "unconfigured" state.
         * Releases hardware resources (pigpiod connection) while keeping the interface
         * object alive for potential reconfiguration.
         * 
         * Different from shutdown - the interface may be reconfigured and reused.
         * 
         * @param previous_state The lifecycle state we're transitioning from
         * @return SUCCESS after disconnecting from pigpiod
         */
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State & previous_state) override;

        /**
         * @brief ROS2 Lifecycle: Final shutdown and resource cleanup
         * 
         * Called when the hardware interface is being permanently destroyed.
         * This is the final cleanup step - ensures all resources are released
         * and servos are safely stopped before the driver exits.
         * 
         * @param previous_state The lifecycle state we're transitioning from
         * @return SUCCESS after complete cleanup
         */
        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State & previous_state) override;

        /**
         * @brief ROS2 Lifecycle: Handle error conditions
         * 
         * Called when an error occurs during any lifecycle transition or during
         * normal operation. Provides opportunity for error recovery or safe shutdown.
         * 
         * @param previous_state The lifecycle state we're transitioning from
         * @return ERROR to indicate error handling is complete
         */
        hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State & previous_state) override;

        /**
         * @brief Export state interfaces for ROS2 control system
         * 
         * Tells ROS2 control what information this hardware interface can provide about
         * the robot's current state. Controllers use these interfaces to read joint positions
         * for feedback control and monitoring.
         * 
         * For MG996R servos (no encoders): We can only provide position state based on
         * the last commanded position, not actual measured position.
         * 
         * @return Vector of state interfaces 
         */
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /**
         * @brief Export command interfaces for ROS2 control system
         * 
         * Tells ROS2 control what commands this hardware interface can accept.
         * Controllers use these interfaces to send position commands to the robot joints.
         * 
         * Each command interface represents one controllable degree of freedom.
         * 
         * @return Vector of command interfaces 
         */
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        /**
         * @brief Read current hardware state and update state interfaces
         * 
         * Called continuously by ROS2 control to read the current state of the robot.
         * Controllers depend on this information for feedback control loops.
         * 
         * LIMITATION: MG996R servos have no position encoders, so we cannot read actual
         * position. We assume the servo reached the commanded position (open-loop control).
         * 
         * @param time Current ROS time
         * @param period Time since last read call  
         * @return OK if read operation completed successfully
         */
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        /**
         * @brief Write commands to hardware and update servo positions
         * 
         * Called continuously by ROS2 control to send new commands to the robot.
         * This is where the actual servo control happens - converts joint angle commands
         * from controllers (like MoveIt) into PWM signals for the servo motors.
         * 
         * Process for each joint:
         * 1. Clamp command to safe joint limits (prevent mechanical damage)
         * 2. Convert angle (radians) to PWM pulse width (microseconds)
         * 3. Send PWM command to servo via pigpiod
         * 
         * @param time Current ROS time
         * @param period Time since last write call
         * @return OK if write operation completed successfully
         */
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:

        /**
         * @brief Configure the joint parameters for the MG996R servos
         * 
         * Sets up hardware configuration for each servo joint:
         * - Angle limits: ±90° (-π/2 to π/2 radians) for 180° total range
         * - PWM range: 1000-2000μs (standard servo control)
         */
        void setupjointconfig();

        /**
         * @brief Establish connection to the pigpiod daemon for GPIO control
         * 
         * Connects to the pigpiod daemon which provides userspace GPIO access without
         * requiring root privileges. The daemon must be running as a system service.
         * This connection is essential for sending PWM signals to the servo motors.
         * 
         * @return true if connection successful, false if failed
         * 
         * Technical note: pigpiod provides precise hardware-timed PWM signals needed
         * for servo control, avoiding jitter that can occur with software PWM.
         */
        bool connectToPigpio();

        /**
         * @brief Safely disconnect from the pigpiod daemon
         * 
         * Closes the connection to pigpiod daemon if currently connected. This automatically
         * stops all PWM signals and releases GPIO resources. Safe to call multiple times.
         * Essential for proper cleanup during shutdown or reconfiguration.
         */
        void disconnectFromPigpio();

        /**
         * @brief Emergency stop: Immediately stop all servo motors
         * 
         * Sends zero pulse width to all servos, which stops PWM signals and allows
         * servos to move freely (no holding torque). Used for safety during deactivation
         * or emergency situations. Prevents damage if servos are fighting external forces.
         */
        void stopAllServos();

        /**
         * @brief Convert joint angle to PWM pulse width for servo control
         * 
         * Translates a joint angle command (in radians) to the corresponding PWM pulse
         * width (in microseconds) that the servo needs to reach that position.
         * 
         * Process:
         * 1. Normalize angle to 0-1 range based on joint's min/max limits
         * 2. Map normalized value to servo's PWM range (typically 1000-2000μs)
         * 3. Clamp to safe values to prevent servo damage
         * 
         * @param angle Target angle in radians
         * @param joint Joint configuration containing angle and PWM limits
         * @return PWM pulse width in microseconds
         */
        int angleToPulseWidth(double angle, const JointConfig& joint) const;
        
        /**
         * @brief Send PWM command to servo via pigpiod
         * 
         * Transmits the calculated pulse width to the specified GPIO pin using the
         * pigpiod daemon. This generates the actual PWM signal that controls the servo.
         * Includes error checking and logging for diagnostic purposes.
         * 
         * @param pin GPIO pin number connected to servo signal wire
         * @param pulseWidth PWM pulse width in microseconds
         */
        void sendPulseWidth(int pin, int pulseWidth);
    
        std::vector<JointConfig> joint_configs_; // Stores configuration for each joint
        std::vector<double> position_commands_;  // Desired joint positions (radians)
        std::vector<double> position_states_;    // Current joint positions (radians)

        int pigpio_connection_;                  // Connection ID for pigpiod daemon

};

} // namespace perseus_arm