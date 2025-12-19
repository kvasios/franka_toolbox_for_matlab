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
    // 
    // NOTE: Stored as flat 1D arrays (16 elements) to match Simulink's
    // column-major memory layout. This allows linear indexing in generated
    // code while Simulink displays them as [4 4] matrices.
    // ========================================================================
    
    double O_T_EE[16];      ///< Measured EE pose in base frame (4x4 col-major)
    double O_T_EE_d[16];    ///< Desired EE pose in base frame (4x4 col-major)
    double F_T_EE[16];      ///< EE pose in flange frame (4x4 col-major)
    double F_T_NE[16];      ///< Nominal EE pose in flange frame (4x4 col-major)
    double NE_T_EE[16];     ///< EE pose in nominal EE frame (4x4 col-major)
    double EE_T_K[16];      ///< Stiffness frame pose in EE frame (4x4 col-major)
    double O_T_EE_c[16];    ///< Commanded EE pose in base frame (4x4 col-major)
    
    // ========================================================================
    // End Effector Inertial Parameters
    // ========================================================================
    
    double m_ee;            ///< Configured EE mass [kg]
    double I_ee[9];         ///< EE rotational inertia matrix (3x3 col-major)
    double F_x_Cee[3];      ///< EE center of mass in flange frame [m]
    
    // ========================================================================
    // External Load Inertial Parameters
    // ========================================================================
    
    double m_load;          ///< Configured external load mass [kg]
    double I_load[9];       ///< External load rotational inertia matrix (3x3 col-major)
    double F_x_Cload[3];    ///< External load CoM in flange frame [m]
    
    // ========================================================================
    // Total (EE + Load) Inertial Parameters
    // ========================================================================
    
    double m_total;         ///< Total mass (EE + load) [kg]
    double I_total[9];      ///< Total rotational inertia matrix (3x3 col-major)
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

/**
 * @brief C struct matching FrankaModelDataBus Simulink bus
 * 
 * Contains computed dynamics and kinematics from libfranka Model class.
 * These are derived quantities computed from the robot state each cycle.
 * 
 * IMPORTANT: Field order MUST match franka_model_data_bus.m exactly!
 */
struct FrankaModelDataBus {
    // ========================================================================
    // Dynamics
    // 
    // NOTE: Matrices stored as flat 1D arrays (column-major) to match
    // Simulink's memory layout for proper linear indexing in generated code.
    // ========================================================================
    
    double mass[49];        ///< Mass matrix M(q) [kg*m^2] (7x7 col-major)
    double coriolis[7];     ///< Coriolis force vector c(q,dq) [Nm]
    double gravity[7];      ///< Gravity vector g(q) [Nm]
    
    // ========================================================================
    // Jacobians (End Effector)
    // ========================================================================
    
    double jacobian[42];         ///< EE Jacobian in base frame (6x7 col-major)
    double jacobian_body[42];    ///< EE body Jacobian in EE frame (6x7 col-major)
};

/**
 * @brief C struct matching FrankaRobotSettingsBus Simulink bus
 * 
 * Contains all configurable robot settings that are applied when
 * the Enable input transitions from 0 to 1.
 * 
 * IMPORTANT: Field order MUST match franka_robot_settings_bus.m exactly!
 */
struct FrankaRobotSettingsBus {
    // ========================================================================
    // Control Parameters
    // ========================================================================
    
    double rate_limiter;         ///< Enable rate limiting (0=false, 1=true)
    double cutoff_frequency;     ///< Low-pass filter cutoff frequency [Hz]
    
    // ========================================================================
    // Impedance Settings
    // ========================================================================
    
    double joint_impedance_stiffness[7];     ///< Joint stiffness [Nm/rad]
    double cartesian_impedance_stiffness[6]; ///< Cartesian stiffness [N/m, Nm/rad]
    
    // ========================================================================
    // End Effector Frame Configuration (4x4 transforms, column-major)
    // 
    // NOTE: Stored as flat 1D arrays to match Simulink's memory layout.
    // ========================================================================
    
    double NE_T_EE[16];     ///< Nominal EE to EE transformation (4x4 col-major)
    double EE_T_K[16];      ///< EE to stiffness frame transformation (4x4 col-major)
    
    // ========================================================================
    // Collision Thresholds - Torque (7 joints)
    // ========================================================================
    
    double lower_torque_thresholds_acceleration[7]; ///< Lower torque threshold during accel [Nm]
    double upper_torque_thresholds_acceleration[7]; ///< Upper torque threshold during accel [Nm]
    double lower_torque_thresholds_nominal[7];      ///< Lower torque threshold nominal [Nm]
    double upper_torque_thresholds_nominal[7];      ///< Upper torque threshold nominal [Nm]
    
    // ========================================================================
    // Collision Thresholds - Force (6 Cartesian DOF)
    // ========================================================================
    
    double lower_force_thresholds_acceleration[6]; ///< Lower force threshold during accel [N]
    double upper_force_thresholds_acceleration[6]; ///< Upper force threshold during accel [N]
    double lower_force_thresholds_nominal[6];      ///< Lower force threshold nominal [N]
    double upper_force_thresholds_nominal[6];      ///< Upper force threshold nominal [N]
    
    // ========================================================================
    // Load Inertia Parameters
    // ========================================================================
    
    double load_mass;                  ///< External load mass [kg]
    double load_center_of_mass[3];     ///< Load center of mass in flange frame [m]
    double load_inertia_matrix[9];     ///< Load rotational inertia [kg*m^2] (3x3 col-major)
};

#endif // FRANKA_SIMULINK_TYPES_H
