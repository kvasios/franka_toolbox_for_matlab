// Copyright (c) 2025 Franka Robotics GmbH
// Franka Robot API - Header
//
// This provides the interface between Simulink code generation and libfranka
// for torque control with function-call subsystem pattern.
//
// Key Architecture:
//   The libfranka robot.control() callback directly invokes the Simulink-generated
//   controller function via a function pointer. No thread synchronization needed -
//   the controller code runs IN the 1kHz control thread.

#ifndef FRANKA_ROBOT_API_H
#define FRANKA_ROBOT_API_H

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "franka_simulink_types.h"

/**
 * @brief Type for the controller callback function
 * 
 * This is a pointer to the Simulink-generated function that executes
 * the function-call subsystem (user's controller).
 */
using ControllerCallback = void (*)(void* user_data, double dt_sec);

/**
 * @brief Context class for Franka robot control
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
class FrankaRobotContext {
public:
    FrankaRobotContext();
    ~FrankaRobotContext();
    
    // Non-copyable, non-movable
    FrankaRobotContext(const FrankaRobotContext&) = delete;
    FrankaRobotContext& operator=(const FrankaRobotContext&) = delete;
    FrankaRobotContext(FrankaRobotContext&&) = delete;
    FrankaRobotContext& operator=(FrankaRobotContext&&) = delete;
    
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
     * @param user_data Opaque pointer passed to callback on each invocation
     */
    void setControllerCallback(ControllerCallback callback, void* user_data);
    
    /**
     * @brief Set pointer to Simulink robot state output bus
     * @param state_ptr Pointer to FrankaRobotStateBus output
     */
    void setStateOutputPointer(FrankaRobotStateBus* state_ptr);
    
    /**
     * @brief Set pointer to Simulink model data output bus
     * @param model_ptr Pointer to FrankaModelDataBus output
     */
    void setModelOutputPointer(FrankaModelDataBus* model_ptr);
    
    /**
     * @brief Set pointer to dt_sec output (control period)
     * @param dt_sec_ptr Pointer to dt output [1] (seconds)
     * 
     * Note: dt_sec is also available in the state bus as 'time' (cumulative),
     * but this separate output provides the per-callback period directly.
     */
    void setDtOutputPointer(double* dt_sec_ptr);
    
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

    // Control thread (runs robot.control())
    std::thread control_thread_;
    
    // Controller callback (points to Simulink-generated function)
    ControllerCallback controller_callback_{nullptr};
    void* controller_user_data_{nullptr};
    
    // Pointers to Simulink I/O signals
    FrankaRobotStateBus* state_out_{nullptr};
    FrankaModelDataBus* model_out_{nullptr};
    double* dt_sec_out_{nullptr};
    const double* tau_J_d_in_{nullptr};
    
    // Helper to copy franka::RobotState to FrankaRobotStateBus
    void copyRobotState(const franka::RobotState& src, FrankaRobotStateBus* dst);
    
    // Helper to compute and copy model data to FrankaModelDataBus
    void computeModelData(const franka::RobotState& state, FrankaModelDataBus* dst);
};

#endif // FRANKA_ROBOT_API_H
