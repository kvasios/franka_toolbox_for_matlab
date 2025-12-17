// Copyright (c) 2025 Franka Robotics GmbH
// Franka Robot API - Header
//
// This provides the interface between Simulink code generation and libfranka
// with support for multiple control modes (torque, joint position/velocity,
// Cartesian pose/velocity).
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
 * @brief Control mode enumeration
 * 
 * Matches the block mask dropdown order for mode selection.
 * 
 * Single-callback modes (0-4) use the robot's internal controller for non-torque modes.
 * Dual-callback modes (5-8) allow user to provide both torque AND motion generator callbacks.
 */
enum class FrankaControlMode {
    /* Single-callback modes */
    Torques = 0,                    ///< Direct torque control
    JointPositions = 1,             ///< Joint position with internal impedance
    JointVelocities = 2,            ///< Joint velocity with internal impedance
    CartesianPose = 3,              ///< Cartesian pose with internal impedance
    CartesianVelocities = 4,        ///< Cartesian velocity with internal impedance
    /* Dual-callback modes (torque + motion generator) */
    TorquesJointPositions = 5,      ///< External torque + joint position motion gen
    TorquesJointVelocities = 6,     ///< External torque + joint velocity motion gen
    TorquesCartesianPose = 7,       ///< External torque + Cartesian pose motion gen
    TorquesCartesianVelocities = 8  ///< External torque + Cartesian velocity motion gen
};

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
     * @brief Set pointers to Simulink input signals (legacy torque-only interface)
     * @param tau_J_d_ptr Pointer to tau_J_d input [7]
     * @deprecated Use mode-specific setters instead
     */
    void setInputPointers(const double* tau_J_d_ptr);
    
    /**
     * @brief Set the control mode
     * @param mode Control mode (Torques, JointPositions, JointVelocities, 
     *                          CartesianPose, CartesianVelocities)
     */
    void setControlMode(FrankaControlMode mode);
    
    /**
     * @brief Set torque command input pointer (mode: Torques)
     * @param tau_J_d_ptr Pointer to tau_J_d input [7] (Nm)
     */
    void setTorqueInputPointer(const double* tau_J_d_ptr);
    
    /**
     * @brief Set joint position command input pointer (mode: JointPositions)
     * @param q_d_ptr Pointer to q_d input [7] (rad)
     */
    void setJointPositionInputPointer(const double* q_d_ptr);
    
    /**
     * @brief Set joint velocity command input pointer (mode: JointVelocities)
     * @param dq_d_ptr Pointer to dq_d input [7] (rad/s)
     */
    void setJointVelocityInputPointer(const double* dq_d_ptr);
    
    /**
     * @brief Set Cartesian pose command input pointers (mode: CartesianPose)
     * @param O_T_EE_d_ptr Pointer to O_T_EE_d input [16] (4x4 col-major, m)
     * @param elbow_d_ptr Pointer to elbow_d input [2] (rad, sign)
     */
    void setCartesianPoseInputPointer(const double* O_T_EE_d_ptr, const double* elbow_d_ptr);
    
    /**
     * @brief Set Cartesian velocity command input pointers (mode: CartesianVelocities)
     * @param O_dP_EE_d_ptr Pointer to O_dP_EE_d input [6] (m/s, rad/s)
     * @param elbow_d_ptr Pointer to elbow_d input [2] (rad, sign)
     */
    void setCartesianVelocityInputPointer(const double* O_dP_EE_d_ptr, const double* elbow_d_ptr);
    
    // ========================================================================
    // Dual-callback mode input setters (torque + motion generator)
    // ========================================================================
    
    /**
     * @brief Set input pointers for Torques+JointPositions dual mode
     * @param tau_J_d_ptr Pointer to tau_J_d input [7] (Nm)
     * @param q_d_ptr Pointer to q_d input [7] (rad)
     */
    void setTorquesJointPositionInputPointers(const double* tau_J_d_ptr, const double* q_d_ptr);
    
    /**
     * @brief Set input pointers for Torques+JointVelocities dual mode
     * @param tau_J_d_ptr Pointer to tau_J_d input [7] (Nm)
     * @param dq_d_ptr Pointer to dq_d input [7] (rad/s)
     */
    void setTorquesJointVelocityInputPointers(const double* tau_J_d_ptr, const double* dq_d_ptr);
    
    /**
     * @brief Set input pointers for Torques+CartesianPose dual mode
     * @param tau_J_d_ptr Pointer to tau_J_d input [7] (Nm)
     * @param O_T_EE_d_ptr Pointer to O_T_EE_d input [16] (4x4 col-major, m)
     * @param elbow_d_ptr Pointer to elbow_d input [2] (rad, sign)
     */
    void setTorquesCartesianPoseInputPointers(const double* tau_J_d_ptr, 
                                               const double* O_T_EE_d_ptr, 
                                               const double* elbow_d_ptr);
    
    /**
     * @brief Set input pointers for Torques+CartesianVelocities dual mode
     * @param tau_J_d_ptr Pointer to tau_J_d input [7] (Nm)
     * @param O_dP_EE_d_ptr Pointer to O_dP_EE_d input [6] (m/s, rad/s)
     * @param elbow_d_ptr Pointer to elbow_d input [2] (rad, sign)
     */
    void setTorquesCartesianVelocityInputPointers(const double* tau_J_d_ptr,
                                                   const double* O_dP_EE_d_ptr,
                                                   const double* elbow_d_ptr);
    
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
    
    // ========================================================================
    // Single-callback mode callbacks
    // ========================================================================
    franka::Torques torqueCallback(const franka::RobotState& state,
                                    franka::Duration period);
    franka::JointPositions jointPositionCallback(const franka::RobotState& state,
                                                  franka::Duration period);
    franka::JointVelocities jointVelocityCallback(const franka::RobotState& state,
                                                   franka::Duration period);
    franka::CartesianPose cartesianPoseCallback(const franka::RobotState& state,
                                                 franka::Duration period);
    franka::CartesianVelocities cartesianVelocityCallback(const franka::RobotState& state,
                                                           franka::Duration period);
    
    // ========================================================================
    // Dual-callback mode callbacks (torque + motion generator)
    // These return Torques for the torque callback part
    // ========================================================================
    franka::Torques dualTorqueCallback(const franka::RobotState& state,
                                        franka::Duration period);
    franka::JointPositions dualJointPositionMotionCallback(const franka::RobotState& state,
                                                            franka::Duration period);
    franka::JointVelocities dualJointVelocityMotionCallback(const franka::RobotState& state,
                                                             franka::Duration period);
    franka::CartesianPose dualCartesianPoseMotionCallback(const franka::RobotState& state,
                                                           franka::Duration period);
    franka::CartesianVelocities dualCartesianVelocityMotionCallback(const franka::RobotState& state,
                                                                     franka::Duration period);
    
    // Common pre-callback logic (state copy, model compute, controller execute)
    // Returns true if control should continue, false if stop requested
    bool executePreCallback(const franka::RobotState& state, franka::Duration period);
    
    // Robot connection
    std::string robot_ip_;
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<franka::Model> model_;
    
    // Control mode
    FrankaControlMode control_mode_{FrankaControlMode::Torques};
    
    // Control state
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};

    // Control thread (runs robot.control())
    std::thread control_thread_;
    
    // Controller callback (points to Simulink-generated function)
    ControllerCallback controller_callback_{nullptr};
    void* controller_user_data_{nullptr};
    
    // Pointers to Simulink output signals
    FrankaRobotStateBus* state_out_{nullptr};
    FrankaModelDataBus* model_out_{nullptr};
    double* dt_sec_out_{nullptr};
    
    // Pointers to Simulink input signals (mode-specific)
    const double* tau_J_d_in_{nullptr};      // Torques mode
    const double* q_d_in_{nullptr};           // JointPositions mode
    const double* dq_d_in_{nullptr};          // JointVelocities mode
    const double* O_T_EE_d_in_{nullptr};      // CartesianPose mode
    const double* O_dP_EE_d_in_{nullptr};     // CartesianVelocities mode
    const double* elbow_d_in_{nullptr};       // Cartesian modes (elbow config)
    
    // Helper to copy franka::RobotState to FrankaRobotStateBus
    void copyRobotState(const franka::RobotState& src, FrankaRobotStateBus* dst);
    
    // Helper to compute and copy model data to FrankaModelDataBus
    void computeModelData(const franka::RobotState& state, FrankaModelDataBus* dst);
};

#endif // FRANKA_ROBOT_API_H
