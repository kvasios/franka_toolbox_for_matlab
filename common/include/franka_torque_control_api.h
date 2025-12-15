// Copyright (c) 2025 Franka Robotics GmbH
// Franka Torque Control API - Header
//
// This provides the interface between Simulink code generation and libfranka
// for torque control with function-call subsystem pattern.

#ifndef FRANKA_TORQUE_CONTROL_API_H
#define FRANKA_TORQUE_CONTROL_API_H

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

/**
 * @brief Control states for the Franka torque control context
 */
enum class FrankaControlState {
    Idle = 0,
    Running = 1,
    Error = 2,
    Stopping = 3
};

/**
 * @brief Context class for Franka robot torque control with function-call triggering
 * 
 * This class encapsulates the libfranka robot.control() loop and provides
 * synchronization primitives for the function-call subsystem pattern in Simulink.
 * 
 * Execution flow:
 * 1. Control thread runs robot.control() callback at 1kHz
 * 2. Callback updates state, signals Simulink thread, waits
 * 3. Simulink executes function-call subsystem (user controller)
 * 4. Simulink sets torques, signals control thread to continue
 * 5. Control thread returns torques to libfranka
 */
class FrankaTorqueControlContext {
public:
    /**
     * @brief Default constructor
     */
    FrankaTorqueControlContext();
    
    /**
     * @brief Destructor - ensures clean shutdown
     */
    ~FrankaTorqueControlContext();
    
    // Non-copyable, non-movable (due to threading)
    FrankaTorqueControlContext(const FrankaTorqueControlContext&) = delete;
    FrankaTorqueControlContext& operator=(const FrankaTorqueControlContext&) = delete;
    FrankaTorqueControlContext(FrankaTorqueControlContext&&) = delete;
    FrankaTorqueControlContext& operator=(FrankaTorqueControlContext&&) = delete;
    
    // ========================================================================
    // Lifecycle Methods (called from TLC Start/Terminate)
    // ========================================================================
    
    /**
     * @brief Initialize connection to the robot
     * @param robot_ip IP address of the Franka robot
     */
    void initialize(const std::string& robot_ip);
    
    /**
     * @brief Shutdown and cleanup
     */
    void shutdown();
    
    // ========================================================================
    // Control Methods (called from TLC Outputs)
    // ========================================================================
    
    /**
     * @brief Start the control loop (called on Enable rising edge)
     */
    void startControl();
    
    /**
     * @brief Stop the control loop (called on Enable falling edge)
     */
    void stopControl();
    
    /**
     * @brief Check if control loop is currently running
     */
    bool isControlRunning() const;
    
    /**
     * @brief Check if an error has occurred
     */
    bool hasError() const;
    
    /**
     * @brief Get the last error message
     */
    std::string getErrorMessage() const;
    
    // ========================================================================
    // Synchronization Methods (for function-call pattern)
    // ========================================================================
    
    /**
     * @brief Wait for control thread to signal new state is ready
     * Called by Simulink thread before executing function-call subsystem
     */
    void waitForControlStep();
    
    /**
     * @brief Signal control thread that torques are ready
     * Called by Simulink thread after function-call subsystem completes
     */
    void signalControlContinue();
    
    // ========================================================================
    // Data Access Methods
    // ========================================================================
    
    /**
     * @brief Get measured joint positions
     * @param q Output array (7 elements)
     */
    void getJointPositions(double* q) const;
    
    /**
     * @brief Get measured joint velocities
     * @param dq Output array (7 elements)
     */
    void getJointVelocities(double* dq) const;
    
    /**
     * @brief Set commanded joint torques
     * @param tau_J_d Input array (7 elements)
     */
    void setJointTorques(const double* tau_J_d);
    
private:
    /**
     * @brief Control thread main function
     */
    void controlThreadFunc();
    
    /**
     * @brief The libfranka control callback
     */
    franka::Torques controlCallback(const franka::RobotState& state, 
                                     franka::Duration period);
    
    // Robot connection
    std::string robot_ip_;
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<franka::Model> model_;
    
    // Control state
    std::atomic<FrankaControlState> state_{FrankaControlState::Idle};
    std::atomic<bool> stop_requested_{false};
    std::string error_message_;
    mutable std::mutex error_mutex_;
    
    // Control thread
    std::thread control_thread_;
    
    // Robot state (written by control thread, read by Simulink)
    franka::RobotState robot_state_;
    std::mutex state_mutex_;
    
    // Command (written by Simulink, read by control thread)
    std::array<double, 7> tau_J_d_{};
    std::mutex command_mutex_;
    
    // Synchronization for function-call pattern
    std::mutex sync_mutex_;
    std::condition_variable cv_state_ready_;      // Control -> Simulink
    std::condition_variable cv_command_ready_;    // Simulink -> Control
    bool state_ready_{false};
    bool command_ready_{false};
    
    // First step flag
    bool first_control_step_{true};
};

#endif // FRANKA_TORQUE_CONTROL_API_H

