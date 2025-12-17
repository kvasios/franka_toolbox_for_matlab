// Copyright (c) 2025 Franka Robotics GmbH
// Franka Simulink Types - C struct definitions for Simulink bus compatibility
//
// This header defines C structs that mirror the Simulink bus definitions.
// The struct layout MUST match the order in franka_robot_state_bus.m exactly.
//
// Used by:
//   - franka_robot_api.cpp (to copy robot state)
//   - Generated code (Simulink Coder creates matching struct)

#ifndef FRANKA_SIMULINK_TYPES_H
#define FRANKA_SIMULINK_TYPES_H

#include <cstdint>

/**
 * @brief C struct matching FrankaRobotStateBus Simulink bus
 * 
 * This struct mirrors franka::RobotState but with plain arrays instead of
 * std::array, and with a layout that matches the Simulink bus element order.
 * 
 * IMPORTANT: The field order MUST match franka_robot_state_bus.m exactly!
 * Any mismatch will cause data corruption in Simulink.
 * 
 * NOTE: Matrices (4x4 transforms, 3x3 inertias) are stored in column-major
 * order, matching both libfranka and MATLAB/Simulink conventions.
 */
struct FrankaRobotStateBus {
    // ========================================================================
    // Transformation Matrices (4x4 homogeneous transforms, column-major)
    // ========================================================================
    
    double O_T_EE[4][4];    ///< Measured EE pose in base frame
    double O_T_EE_d[4][4];  ///< Desired EE pose in base frame
    double F_T_EE[4][4];    ///< EE pose in flange frame
    double F_T_NE[4][4];    ///< Nominal EE pose in flange frame
    double NE_T_EE[4][4];   ///< EE pose in nominal EE frame
    double EE_T_K[4][4];    ///< Stiffness frame pose in EE frame
    double O_T_EE_c[4][4];  ///< Commanded EE pose in base frame
    
    // ========================================================================
    // End Effector Inertial Parameters
    // ========================================================================
    
    double m_ee;            ///< Configured EE mass [kg]
    double I_ee[3][3];      ///< EE rotational inertia matrix
    double F_x_Cee[3];      ///< EE center of mass in flange frame [m]
    
    // ========================================================================
    // External Load Inertial Parameters
    // ========================================================================
    
    double m_load;          ///< Configured external load mass [kg]
    double I_load[3][3];    ///< External load rotational inertia matrix
    double F_x_Cload[3];    ///< External load CoM in flange frame [m]
    
    // ========================================================================
    // Total (EE + Load) Inertial Parameters
    // ========================================================================
    
    double m_total;         ///< Total mass (EE + load) [kg]
    double I_total[3][3];   ///< Total rotational inertia matrix
    double F_x_Ctotal[3];   ///< Total CoM in flange frame [m]
    
    // ========================================================================
    // Elbow Configuration
    // ========================================================================
    
    double elbow[2];        ///< Current elbow [q3, flip_direction]
    double elbow_d[2];      ///< Desired elbow
    double elbow_c[2];      ///< Commanded elbow
    double delbow_c[2];     ///< Commanded elbow velocity [rad/s, 0]
    double ddelbow_c[2];    ///< Commanded elbow acceleration [rad/s^2, 0]
    
    // ========================================================================
    // Joint-Space Signals (7 DOF)
    // ========================================================================
    
    double tau_J[7];        ///< Measured joint torques [Nm]
    double tau_J_d[7];      ///< Desired joint torques (no gravity) [Nm]
    double dtau_J[7];       ///< Joint torque derivatives [Nm/s]
    double q[7];            ///< Measured joint positions [rad]
    double q_d[7];          ///< Desired joint positions [rad]
    double dq[7];           ///< Measured joint velocities [rad/s]
    double dq_d[7];         ///< Desired joint velocities [rad/s]
    double ddq_d[7];        ///< Desired joint accelerations [rad/s^2]
    double theta[7];        ///< Motor positions [rad]
    double dtheta[7];       ///< Motor velocities [rad/s]
    
    // ========================================================================
    // Contact and Collision Detection
    // ========================================================================
    
    double joint_contact[7];        ///< Joint contact levels (auto-reset)
    double cartesian_contact[6];    ///< Cartesian contact levels (auto-reset)
    double joint_collision[7];      ///< Joint collision levels (latched)
    double cartesian_collision[6];  ///< Cartesian collision levels (latched)
    
    // ========================================================================
    // External Force Estimates
    // ========================================================================
    
    double tau_ext_hat_filtered[7]; ///< Estimated external joint torques [Nm]
    double O_F_ext_hat_K[6];        ///< External wrench in base frame [N,Nm]
    double K_F_ext_hat_K[6];        ///< External wrench in K frame [N,Nm]
    
    // ========================================================================
    // Cartesian Motion Signals
    // ========================================================================
    
    double O_dP_EE_d[6];    ///< Desired EE twist in base frame [m/s, rad/s]
    double O_ddP_O[3];      ///< Base linear acceleration [m/s^2]
    double O_dP_EE_c[6];    ///< Commanded EE twist [m/s, rad/s]
    double O_ddP_EE_c[6];   ///< Commanded EE acceleration [m/s^2, rad/s^2]
    
    // ========================================================================
    // Status Signals
    // ========================================================================
    
    double control_command_success_rate;  ///< Command success rate [0-1]
    int32_t robot_mode;                   ///< Robot mode (see RobotMode enum)
    double time;                          ///< Time since robot start [s]
};

/**
 * @brief Robot mode enumeration (matches franka::RobotMode)
 * 
 * Used for robot_mode field in FrankaRobotStateBus.
 */
enum FrankaRobotMode {
    FRANKA_ROBOT_MODE_OTHER = 0,
    FRANKA_ROBOT_MODE_IDLE = 1,
    FRANKA_ROBOT_MODE_MOVE = 2,
    FRANKA_ROBOT_MODE_GUIDING = 3,
    FRANKA_ROBOT_MODE_REFLEX = 4,
    FRANKA_ROBOT_MODE_USER_STOPPED = 5,
    FRANKA_ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY = 6
};

#endif // FRANKA_SIMULINK_TYPES_H
