// Copyright (c) 2025 Franka Robotics GmbH
// Franka Torque Control API - Header
//
// This provides the interface between Simulink code generation and libfranka
// for torque control with function-call subsystem pattern.
//
// Key Architecture:
//   The libfranka robot.control() callback directly invokes the Simulink-generated
//   controller function via a function pointer. No thread synchronization needed -
//   the controller code runs IN the 1kHz control thread.

#ifndef FRANKA_TORQUE_CONTROL_API_H
#define FRANKA_TORQUE_CONTROL_API_H

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

/**
 * @brief Type for the controller callback function
 * 
 * This is a pointer to the Simulink-generated function that executes
 * the function-call subsystem (user's controller).
 */
using ControllerCallback = void (*)(void);

/**
 * @brief Context class for Franka robot torque control
 * 
 * Execution model:
 *   1. startControl() spawns a thread that calls robot.control()
 *   2. Inside robot.control() callback (at 1kHz):
 *      a. Copy robot state to Simulink output signal pointers (q, dq)
 *      b. Call the controller callback (executes user's Simulink controller)
 *      c. Read torques from Simulink input signal pointer (tau_J_d)
 *      d. Return torques to libfranka
 *   3. requestStop() gracefully terminates the control loop
 * 
 * The controller callback runs IN the control thread context - no sync needed.
 */
class FrankaTorqueControlContext {
public:
    FrankaTorqueControlContext();
    ~FrankaTorqueControlContext();
    
    // Non-copyable, non-movable
    FrankaTorqueControlContext(const FrankaTorqueControlContext&) = delete;
    FrankaTorqueControlContext& operator=(const FrankaTorqueControlContext&) = delete;
    FrankaTorqueControlContext(FrankaTorqueControlContext&&) = delete;
    FrankaTorqueControlContext& operator=(FrankaTorqueControlContext&&) = delete;
    
    // ========================================================================
    // Lifecycle (called from TLC Start/Terminate)
    // ========================================================================
    
    /**
     * @brief Initialize connection to robot
     * @param robot_ip IP address of the Franka robot
     */
    void initialize(const std::string& robot_ip);
    
    /**
     * @brief Set the controller callback function
     * @param callback Function pointer to the controller callback
     */
    void setControllerCallback(ControllerCallback callback);
    
    /**
     * @brief Set pointers to Simulink output signals
     * @param q_ptr Pointer to q output [7]
     * @param dq_ptr Pointer to dq output [7]
     */
    void setOutputPointers(double* q_ptr, double* dq_ptr);
    
    /**
     * @brief Set pointers to Simulink input signals
     * @param tau_J_d_ptr Pointer to tau_J_d input [7]
     */
    void setInputPointers(const double* tau_J_d_ptr);
    
    /**
     * @brief Shutdown and cleanup
     */
    void shutdown();
    
    // ========================================================================
    // Control (called from TLC Outputs for enable/disable)
    // ========================================================================
    
    /**
     * @brief Start the control loop
     */
    void startControl();
    
    /**
     * @brief Request graceful stop
     */
    void requestStop();
    
    /**
     * @brief Check if control is currently running
     */
    bool isControlRunning() const;
    
private:
    void controlThreadFunc();
    
    franka::Torques controlCallback(const franka::RobotState& state,
                                     franka::Duration period);
    
    // Robot connection
    std::string robot_ip_;
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<franka::Model> model_;
    
    // Control state
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    
    // Control thread
    std::thread control_thread_;
    
    // Controller callback (points to Simulink-generated function)
    ControllerCallback controller_callback_{nullptr};
    
    // Pointers to Simulink I/O signals
    double* q_out_{nullptr};
    double* dq_out_{nullptr};
    const double* tau_J_d_in_{nullptr};
    
    // First step flag
    bool first_step_{true};
};

#endif // FRANKA_TORQUE_CONTROL_API_H
