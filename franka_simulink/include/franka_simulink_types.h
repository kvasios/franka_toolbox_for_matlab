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
 * @brief C struct matching FrankaErrorsBus Simulink bus
 * 
 * Contains 41 boolean error flags from franka::Errors.
 * Multiple errors can be active simultaneously.
 * 
 * IMPORTANT: Field order MUST match franka_errors_bus.m exactly!
 */
struct FrankaErrorsBus {
    // Position and Velocity Limit Violations
    uint8_t joint_position_limits_violation;
    uint8_t cartesian_position_limits_violation;
    uint8_t self_collision_avoidance_violation;
    uint8_t joint_velocity_violation;
    uint8_t cartesian_velocity_violation;
    
    // Force and Collision Detection
    uint8_t force_control_safety_violation;
    uint8_t joint_reflex;
    uint8_t cartesian_reflex;
    
    // Internal Motion Generator Errors
    uint8_t max_goal_pose_deviation_violation;
    uint8_t max_path_pose_deviation_violation;
    uint8_t cartesian_velocity_profile_safety_violation;
    
    // Joint Motion Generator Errors
    uint8_t joint_position_motion_generator_start_pose_invalid;
    uint8_t joint_motion_generator_position_limits_violation;
    uint8_t joint_motion_generator_velocity_limits_violation;
    uint8_t joint_motion_generator_velocity_discontinuity;
    uint8_t joint_motion_generator_acceleration_discontinuity;
    
    // Cartesian Motion Generator Errors
    uint8_t cartesian_position_motion_generator_start_pose_invalid;
    uint8_t cartesian_motion_generator_elbow_limit_violation;
    uint8_t cartesian_motion_generator_velocity_limits_violation;
    uint8_t cartesian_motion_generator_velocity_discontinuity;
    uint8_t cartesian_motion_generator_acceleration_discontinuity;
    uint8_t cartesian_motion_generator_elbow_sign_inconsistent;
    uint8_t cartesian_motion_generator_start_elbow_invalid;
    uint8_t cartesian_motion_generator_joint_position_limits_violation;
    uint8_t cartesian_motion_generator_joint_velocity_limits_violation;
    uint8_t cartesian_motion_generator_joint_velocity_discontinuity;
    uint8_t cartesian_motion_generator_joint_acceleration_discontinuity;
    uint8_t cartesian_position_motion_generator_invalid_frame;
    
    // Controller and Communication Errors
    uint8_t force_controller_desired_force_tolerance_violation;
    uint8_t controller_torque_discontinuity;
    uint8_t start_elbow_sign_inconsistent;
    uint8_t communication_constraints_violation;
    uint8_t power_limit_violation;
    
    // Planning and System Errors
    uint8_t joint_p2p_insufficient_torque_for_planning;
    uint8_t tau_j_range_violation;
    uint8_t instability_detected;
    uint8_t joint_move_in_wrong_direction;
    
    // Spline and Via Point Motion Errors
    uint8_t cartesian_spline_motion_generator_violation;
    uint8_t joint_via_motion_generator_planning_joint_limit_violation;
    
    // Base Acceleration Errors
    uint8_t base_acceleration_initialization_timeout;
    uint8_t base_acceleration_invalid_reading;
};

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
    // Error State (franka::Errors)
    // ========================================================================
    
    FrankaErrorsBus current_errors;      ///< Current error state (41 boolean flags)
    FrankaErrorsBus last_motion_errors;  ///< Errors that aborted previous motion
    
    // ========================================================================
    // Status Signals
    // ========================================================================
    
    double control_command_success_rate;  ///< Command success rate [0-1]
    int32_t robot_mode;                   ///< Robot mode (see RobotMode enum)
    double time;                          ///< Time since robot start [s]
    
    // ========================================================================
    // Connection Status (Simulink block lifecycle, not from libfranka)
    // ========================================================================
    
    int32_t connection_status;            ///< Connection lifecycle (see FrankaConnectionStatus)
    int32_t last_connection_error_code;   ///< Last connection error type (see FrankaConnectionErrorCode)
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
 * @brief Connection status enumeration
 * 
 * Tracks the lifecycle state of the robot connection from Simulink's perspective.
 * This allows the user to see what's happening even before a successful connection.
 */
enum FrankaConnectionStatus {
    FRANKA_CONNECTION_DISCONNECTED = 0,     ///< Not connected, not attempting
    FRANKA_CONNECTION_CONNECTING = 1,       ///< Connection attempt in progress
    FRANKA_CONNECTION_CONNECTED = 2,        ///< Successfully connected, idle
    FRANKA_CONNECTION_CONTROL_RUNNING = 3,  ///< Active control loop running
    FRANKA_CONNECTION_ERROR = 4             ///< Last operation failed (see error_code)
};

/**
 * @brief Connection error code enumeration for libfranka exceptions
 * 
 * Maps libfranka exception types to integer codes for Simulink visibility.
 * When connection_status is FRANKA_CONNECTION_ERROR, this indicates the cause.
 * 
 * See also: FrankaConnectionErrorCode.m for MATLAB enumeration
 */
enum FrankaConnectionErrorCode {
    FRANKA_CONNECTION_ERROR_NONE = 0,                  ///< No error
    FRANKA_CONNECTION_ERROR_NETWORK = 1,               ///< NetworkException: Connection/timeout error
    FRANKA_CONNECTION_ERROR_PROTOCOL = 2,              ///< ProtocolException: Invalid robot response
    FRANKA_CONNECTION_ERROR_INCOMPATIBLE_VERSION = 3,  ///< IncompatibleVersionException: Version mismatch
    FRANKA_CONNECTION_ERROR_CONTROL = 4,               ///< ControlException: Motion/torque control error
    FRANKA_CONNECTION_ERROR_COMMAND = 5,               ///< CommandException: Command execution error
    FRANKA_CONNECTION_ERROR_REALTIME = 6,              ///< RealtimeException: RT priority failed
    FRANKA_CONNECTION_ERROR_INVALID_OPERATION = 7,     ///< InvalidOperationException: Invalid operation
    FRANKA_CONNECTION_ERROR_MODEL = 8,                 ///< ModelException: Model loading error
    FRANKA_CONNECTION_ERROR_UNKNOWN = 9                ///< Unknown/other exception
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

// ============================================================================
// GRIPPER TYPES
// ============================================================================

/**
 * @brief Gripper command enumeration
 * 
 * Commands are triggered by rising edge on the corresponding trigger input.
 */
enum FrankaGripperCommand {
    FRANKA_GRIPPER_CMD_NONE = 0,     ///< No command (idle)
    FRANKA_GRIPPER_CMD_HOMING = 1,   ///< Perform homing to estimate max width
    FRANKA_GRIPPER_CMD_GRASP = 2,    ///< Grasp an object
    FRANKA_GRIPPER_CMD_MOVE = 3,     ///< Move to target width
    FRANKA_GRIPPER_CMD_STOP = 4      ///< Stop current motion
};

/**
 * @brief Gripper command status enumeration
 */
enum FrankaGripperStatus {
    FRANKA_GRIPPER_STATUS_IDLE = 0,           ///< No command in progress
    FRANKA_GRIPPER_STATUS_BUSY = 1,           ///< Command in progress
    FRANKA_GRIPPER_STATUS_SUCCESS = 2,        ///< Last command succeeded
    FRANKA_GRIPPER_STATUS_FAILED = 3,         ///< Last command returned false
    FRANKA_GRIPPER_STATUS_ERROR = 4           ///< Last command threw exception
};

/**
 * @brief C struct matching FrankaGripperStateBus Simulink bus
 * 
 * This struct mirrors franka::GripperState with additional status fields
 * for command tracking.
 * 
 * IMPORTANT: Field order MUST match franka_gripper_state_bus.m exactly!
 */
struct FrankaGripperStateBus {
    // ========================================================================
    // Gripper State (from franka::GripperState)
    // ========================================================================
    
    double width;           ///< Current gripper opening width [m]
    double max_width;       ///< Maximum gripper opening width [m] (from homing)
    double is_grasped;      ///< Whether an object is grasped (0=no, 1=yes)
    double temperature;     ///< Current gripper temperature [°C]
    double time;            ///< Time since robot start [s]
    
    // ========================================================================
    // Command Status
    // ========================================================================
    
    int32_t command_status;     ///< Current status (FrankaGripperStatus enum)
    int32_t last_command;       ///< Last executed command (FrankaGripperCommand enum)
    int32_t command_success;    ///< Result of last command (0=failed, 1=success)
    int32_t error_code;         ///< Error code if command_status == ERROR
};

/**
 * @brief C struct matching FrankaGripperCommandBus Simulink bus
 * 
 * Contains all parameters for gripper commands. Parameters are read
 * when the corresponding command trigger rises.
 * 
 * IMPORTANT: Field order MUST match franka_gripper_command_bus.m exactly!
 */
struct FrankaGripperCommandBus {
    // ========================================================================
    // Grasp Command Parameters (used when grasp trigger rises)
    // ========================================================================
    
    double grasp_width;         ///< Target grasp width [m]
    double grasp_speed;         ///< Closing speed [m/s]
    double grasp_force;         ///< Grasping force [N]
    double grasp_epsilon_inner; ///< Inner tolerance for grasp detection [m]
    double grasp_epsilon_outer; ///< Outer tolerance for grasp detection [m]
    
    // ========================================================================
    // Move Command Parameters (used when move trigger rises)
    // ========================================================================
    
    double move_width;          ///< Target move width [m]
    double move_speed;          ///< Movement speed [m/s]
};

// ============================================================================
// VACUUM GRIPPER TYPES
// ============================================================================

/**
 * @brief Vacuum gripper command enumeration
 * 
 * Commands are triggered by rising edge on the corresponding trigger input.
 */
enum FrankaVacuumGripperCommand {
    FRANKA_VACUUM_CMD_NONE = 0,      ///< No command (idle)
    FRANKA_VACUUM_CMD_VACUUM = 1,    ///< Activate vacuum to grip object
    FRANKA_VACUUM_CMD_DROP_OFF = 2,  ///< Release object (drop off)
    FRANKA_VACUUM_CMD_STOP = 3       ///< Stop current operation
};

/**
 * @brief Vacuum gripper device status (matches franka::VacuumGripperDeviceStatus)
 */
enum FrankaVacuumGripperDeviceStatus {
    FRANKA_VACUUM_DEVICE_GREEN = 0,   ///< Device working optimally
    FRANKA_VACUUM_DEVICE_YELLOW = 1,  ///< Device working with warnings
    FRANKA_VACUUM_DEVICE_ORANGE = 2,  ///< Device working with severe warnings
    FRANKA_VACUUM_DEVICE_RED = 3      ///< Device not working properly
};

/**
 * @brief Vacuum gripper production setup profile (matches franka::VacuumGripper::ProductionSetupProfile)
 */
enum FrankaVacuumGripperProfile {
    FRANKA_VACUUM_PROFILE_P0 = 0,
    FRANKA_VACUUM_PROFILE_P1 = 1,
    FRANKA_VACUUM_PROFILE_P2 = 2,
    FRANKA_VACUUM_PROFILE_P3 = 3
};

/**
 * @brief Vacuum gripper command status enumeration
 */
enum FrankaVacuumGripperStatus {
    FRANKA_VACUUM_STATUS_IDLE = 0,        ///< No command in progress
    FRANKA_VACUUM_STATUS_BUSY = 1,        ///< Command in progress
    FRANKA_VACUUM_STATUS_SUCCESS = 2,     ///< Last command succeeded
    FRANKA_VACUUM_STATUS_FAILED = 3,      ///< Last command returned false
    FRANKA_VACUUM_STATUS_ERROR = 4        ///< Last command threw exception
};

/**
 * @brief C struct matching FrankaVacuumGripperStateBus Simulink bus
 * 
 * This struct mirrors franka::VacuumGripperState with additional status fields.
 * 
 * IMPORTANT: Field order MUST match franka_vacuum_gripper_state_bus.m exactly!
 */
struct FrankaVacuumGripperStateBus {
    // ========================================================================
    // Vacuum Gripper State (from franka::VacuumGripperState)
    // ========================================================================
    
    double in_control_range;    ///< Vacuum within setpoint area (0=no, 1=yes)
    double part_detached;       ///< Part detached after suction cycle (0=no, 1=yes)
    double part_present;        ///< Part is present/gripped (0=no, 1=yes)
    int32_t device_status;      ///< Device status (FrankaVacuumGripperDeviceStatus enum)
    double actual_power;        ///< Current power consumption [%]
    double vacuum;              ///< Current vacuum level [mbar]
    double time;                ///< Time since robot start [s]
    
    // ========================================================================
    // Command Status
    // ========================================================================
    
    int32_t command_status;     ///< Current status (FrankaVacuumGripperStatus enum)
    int32_t last_command;       ///< Last executed command (FrankaVacuumGripperCommand enum)
    int32_t command_success;    ///< Result of last command (0=failed, 1=success)
    int32_t error_code;         ///< Error code if command_status == ERROR
};

/**
 * @brief C struct matching FrankaVacuumGripperCommandBus Simulink bus
 * 
 * Contains all parameters for vacuum gripper commands.
 * 
 * IMPORTANT: Field order MUST match franka_vacuum_gripper_command_bus.m exactly!
 */
struct FrankaVacuumGripperCommandBus {
    // ========================================================================
    // Vacuum Command Parameters (used when vacuum trigger rises)
    // ========================================================================
    
    double vacuum_setpoint;     ///< Vacuum setpoint [10*mbar] (e.g., 50 = 500 mbar)
    double vacuum_timeout;      ///< Vacuum timeout [ms]
    int32_t vacuum_profile;     ///< Production profile (0=P0, 1=P1, 2=P2, 3=P3)
    
    // ========================================================================
    // Drop Off Command Parameters (used when drop_off trigger rises)
    // ========================================================================
    
    double dropoff_timeout;     ///< Drop off timeout [ms]
};

#endif // FRANKA_SIMULINK_TYPES_H
