/*
 * franka_robot_sfunction.cpp - C++ Level-2 S-Function for Franka robot control
 *
 * This block wraps libfranka's robot.control() with multiple control mode support.
 * It outputs a function-call signal to trigger an external controller subsystem.
 *
 * Parameters:
 *   control_mode - Control mode selection (0-8):
 *
 *     Single-callback modes (robot's internal controller for non-torque):
 *       0: Torques             - Direct torque control
 *       1: JointPositions      - Joint position with internal impedance
 *       2: JointVelocities     - Joint velocity with internal impedance
 *       3: CartesianPose       - Cartesian pose with internal impedance
 *       4: CartesianVelocities - Cartesian velocity with internal impedance
 *
 *     Dual-callback modes (user torque + motion generator):
 *       5: Torques + JointPositions      - External torque + joint position motion gen
 *       6: Torques + JointVelocities     - External torque + joint velocity motion gen
 *       7: Torques + CartesianPose       - External torque + Cartesian pose motion gen
 *       8: Torques + CartesianVelocities - External torque + Cartesian velocity motion gen
 *
 * Inputs (mode-dependent):
 *
 *   Common inputs (all modes):
 *     0. Enable         (1x1)                  - Rising edge starts control, falling edge stops
 *     1. robot_ip       (16x1)                 - Robot IP as ASCII chars
 *     2. settings       (FrankaRobotSettingsBus) - Robot settings (applied on enable)
 *     3. error_recovery (1x1)                  - Rising edge triggers automatic error recovery
 *     4. read_once      (1x1)                  - Rising edge reads robot state (when not in control)
 *
 *   Single-callback modes (0-4):
 *     5. command   (varies)               - Mode 0: tau_J_d (7x1)   [Nm]
 *                                           Mode 1: q_d (7x1)       [rad]
 *                                           Mode 2: dq_d (7x1)      [rad/s]
 *                                           Mode 3: O_T_EE_d (16x1) [m] (4x4 col-major)
 *                                           Mode 4: O_dP_EE_d (6x1) [m/s, rad/s]
 *     6. elbow_d   (2x1)                  - Elbow config (modes 3-4 only)
 *
 *   Dual-callback modes (5-8):
 *     5. tau_J_d    (7x1)                  - Commanded joint torques [Nm]
 *     6. motion_cmd (varies)               - Mode 5: q_d (7x1)       [rad]
 *                                            Mode 6: dq_d (7x1)      [rad/s]
 *                                            Mode 7: O_T_EE_d (16x1) [m] (4x4 col-major)
 *                                            Mode 8: O_dP_EE_d (6x1) [m/s, rad/s]
 *     7. elbow_d    (2x1)                  - Elbow config (modes 7-8 only)
 *
 * Outputs:
 *   0. fcall       (function-call)       - Triggers controller at 1kHz [MUST BE FIRST]
 *   1. robot_state (FrankaRobotStateBus) - Complete robot state from libfranka
 *   2. model_data  (FrankaModelDataBus)  - Computed dynamics/kinematics
 *   3. dt_sec      (1x1)                 - Control period from callback [s]
 *
 * Bus Definitions:
 *   The bus outputs use types that must exist in base workspace:
 *     FrankaRobotStateBus = franka_robot_state_bus();
 *     FrankaModelDataBus = franka_model_data_bus();
 *   Or simply call: franka_setup_bus();
 *
 * Copyright (c) 2025 Franka Robotics GmbH
 */

/* NOTE:
 * This S-function is intentionally NOT named "franka_robot" to avoid
 * collisions with the MATLAB API MEX file `franka_robot.mex*`.
 */
#define S_FUNCTION_NAME  franka_robot_sfunction
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include <string>
#include <cstring>

/* Parameters */
#define NUM_PARAMS          1
#define PARAM_CONTROL_MODE  0

/* Control mode enum (matches block mask dropdown order)
 * 
 * Single-callback modes (use robot's internal controller for non-torque modes):
 *   0: Torques              - Direct torque control
 *   1: JointPositions       - Joint position with internal impedance
 *   2: JointVelocities      - Joint velocity with internal impedance
 *   3: CartesianPose        - Cartesian pose with internal impedance
 *   4: CartesianVelocities  - Cartesian velocity with internal impedance
 * 
 * Dual-callback modes (user provides both torque AND motion generator):
 *   5: Torques + JointPositions      - External torque + joint position motion gen
 *   6: Torques + JointVelocities     - External torque + joint velocity motion gen
 *   7: Torques + CartesianPose       - External torque + Cartesian pose motion gen
 *   8: Torques + CartesianVelocities - External torque + Cartesian velocity motion gen
 */
enum FrankaControlMode {
    /* Single-callback modes */
    CTRL_TORQUES = 0,
    CTRL_JOINT_POSITIONS = 1,
    CTRL_JOINT_VELOCITIES = 2,
    CTRL_CARTESIAN_POSE = 3,
    CTRL_CARTESIAN_VELOCITIES = 4,
    /* Dual-callback modes (torque + motion generator) */
    CTRL_TORQUES_JOINT_POSITIONS = 5,
    CTRL_TORQUES_JOINT_VELOCITIES = 6,
    CTRL_TORQUES_CARTESIAN_POSE = 7,
    CTRL_TORQUES_CARTESIAN_VELOCITIES = 8
};

/* Input port indices - vary by mode */
#define IN_ENABLE           0
#define IN_ROBOT_IP         1
#define IN_SETTINGS         2  /* Robot settings bus (FrankaRobotSettingsBus) */
#define IN_ERROR_RECOVERY   3  /* Trigger: automatic error recovery (1x1) */
#define IN_READ_ONCE        4  /* Trigger: read robot state once (1x1) */
/* Single-callback modes (0-4): */
#define IN_COMMAND          5  /* Command input (tau_J_d, q_d, dq_d, O_T_EE_d, or O_dP_EE_d) */
#define IN_ELBOW            6  /* Elbow input (only for Cartesian single modes 3-4) */
/* Dual-callback modes (5-8): */
#define IN_TAU_J_D          5  /* Torque input for dual modes */
#define IN_MOTION_CMD       6  /* Motion generator input for dual modes */
#define IN_ELBOW_DUAL       7  /* Elbow input (only for Cartesian dual modes 7-8) */

/* Output port indices */
#define OUT_FCALL       0   /* Function-call output - MUST BE FIRST */
#define OUT_STATE       1   /* Robot state bus (FrankaRobotStateBus) */
#define OUT_MODEL       2   /* Model data bus (FrankaModelDataBus) */
#define OUT_DT_SEC      3   /* Control period [s] */
#define NUM_OUTPUTS     4

/* DWork indices */
#define DWORK_PREV_ENABLE           0
#define DWORK_PREV_ERROR_RECOVERY   1
#define DWORK_PREV_READ_ONCE        2
#define NUM_DWORK                   3

/* Parameter validation handled by Simulink parameter mismatch check */

/* ========================================================================
 * mdlInitializeSizes - Initialize block sizes
 * ======================================================================== */
static void mdlInitializeSizes(SimStruct *S)
{
    /* One parameter: control_mode */
    ssSetNumSFcnParams(S, NUM_PARAMS);
    
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by Simulink */
    }
#endif

    /* Parameter is not tunable at runtime */
    ssSetSFcnParamTunable(S, PARAM_CONTROL_MODE, SS_PRM_NOT_TUNABLE);
    
    /* Read control mode parameter */
    int control_mode = static_cast<int>(mxGetScalar(ssGetSFcnParam(S, PARAM_CONTROL_MODE)));
    
    /* States */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    
    /* ====================================================================
     * INPUT PORTS - Dynamic sizing based on control mode
     * ==================================================================== */
    
    /* Determine number of input ports based on mode:
     * Base ports (all modes): Enable, robot_ip, settings, error_recovery, read_once = 5
     * - Modes 0-2 (single, joint-space): +1 (command) = 6 ports
     * - Modes 3-4 (single, Cartesian): +2 (command, elbow) = 7 ports
     * - Modes 5-6 (dual, joint-space): +2 (tau, motion) = 7 ports
     * - Modes 7-8 (dual, Cartesian): +3 (tau, motion, elbow) = 8 ports
     */
    int num_inputs;
    switch (control_mode) {
        case CTRL_TORQUES:
        case CTRL_JOINT_POSITIONS:
        case CTRL_JOINT_VELOCITIES:
            num_inputs = 6;  /* Enable, robot_ip, settings, error_recovery, read_once, command */
            break;
        case CTRL_CARTESIAN_POSE:
        case CTRL_CARTESIAN_VELOCITIES:
            num_inputs = 7;  /* + elbow */
            break;
        case CTRL_TORQUES_JOINT_POSITIONS:
        case CTRL_TORQUES_JOINT_VELOCITIES:
            num_inputs = 7;  /* Enable, robot_ip, settings, error_recovery, read_once, tau_J_d, motion_cmd */
            break;
        case CTRL_TORQUES_CARTESIAN_POSE:
        case CTRL_TORQUES_CARTESIAN_VELOCITIES:
            num_inputs = 8;  /* + elbow */
            break;
        default:
            num_inputs = 6;
            break;
    }
    
    if (!ssSetNumInputPorts(S, num_inputs)) return;
    
    /* Port 0: Enable (1x1) */
    ssSetInputPortWidth(S, IN_ENABLE, 1);
    ssSetInputPortDataType(S, IN_ENABLE, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_ENABLE, 1);
    ssSetInputPortRequiredContiguous(S, IN_ENABLE, 1);
    
    /* Port 1: robot_ip (16 ASCII chars, null-terminated) */
    ssSetInputPortWidth(S, IN_ROBOT_IP, 16);
    ssSetInputPortDataType(S, IN_ROBOT_IP, SS_UINT8);
    ssSetInputPortDirectFeedThrough(S, IN_ROBOT_IP, 1);
    ssSetInputPortRequiredContiguous(S, IN_ROBOT_IP, 1);
    
    /* Port 2: settings (FrankaRobotSettingsBus)
     * 
     * Bus input configuration:
     * - The bus object 'FrankaRobotSettingsBus' must exist in base workspace
     * - Register the bus as a data type and use it for the port
     * - Input as nonvirtual bus (struct in generated code)
     */
#if defined(MATLAB_MEX_FILE)
    {
        DTypeId settingsBusTypeId;
        ssRegisterTypeFromNamedObject(S, "FrankaRobotSettingsBus", &settingsBusTypeId);
        ssSetInputPortDataType(S, IN_SETTINGS, settingsBusTypeId);
    }
#endif
    ssSetInputPortWidth(S, IN_SETTINGS, 1);
    ssSetBusInputAsStruct(S, IN_SETTINGS, 1);
    ssSetInputPortDirectFeedThrough(S, IN_SETTINGS, 1);
    ssSetInputPortRequiredContiguous(S, IN_SETTINGS, 1);
    
    /* Port 3: error_recovery trigger (1x1)
     * Rising edge triggers automatic error recovery (only when not in control)
     */
    ssSetInputPortWidth(S, IN_ERROR_RECOVERY, 1);
    ssSetInputPortDataType(S, IN_ERROR_RECOVERY, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_ERROR_RECOVERY, 1);
    ssSetInputPortRequiredContiguous(S, IN_ERROR_RECOVERY, 1);
    
    /* Port 4: read_once trigger (1x1)
     * Rising edge reads robot state once (only when not in control)
     */
    ssSetInputPortWidth(S, IN_READ_ONCE, 1);
    ssSetInputPortDataType(S, IN_READ_ONCE, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_READ_ONCE, 1);
    ssSetInputPortRequiredContiguous(S, IN_READ_ONCE, 1);
    
    /* Configure remaining ports based on mode */
    switch (control_mode) {
        /* ================================================================
         * SINGLE-CALLBACK MODES (0-4)
         * Port 2: command, Port 3: elbow (Cartesian only)
         * ================================================================ */
        case CTRL_TORQUES:
            /* Port 2: tau_J_d (7x1) */
            ssSetInputPortWidth(S, IN_COMMAND, 7);
            ssSetInputPortDataType(S, IN_COMMAND, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_COMMAND, 1);
            ssSetInputPortRequiredContiguous(S, IN_COMMAND, 1);
            break;
            
        case CTRL_JOINT_POSITIONS:
            /* Port 2: q_d (7x1) */
            ssSetInputPortWidth(S, IN_COMMAND, 7);
            ssSetInputPortDataType(S, IN_COMMAND, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_COMMAND, 1);
            ssSetInputPortRequiredContiguous(S, IN_COMMAND, 1);
            break;
            
        case CTRL_JOINT_VELOCITIES:
            /* Port 2: dq_d (7x1) */
            ssSetInputPortWidth(S, IN_COMMAND, 7);
            ssSetInputPortDataType(S, IN_COMMAND, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_COMMAND, 1);
            ssSetInputPortRequiredContiguous(S, IN_COMMAND, 1);
            break;
            
        case CTRL_CARTESIAN_POSE:
            /* Port 2: O_T_EE_d (16x1, 4x4 col-major) */
            ssSetInputPortWidth(S, IN_COMMAND, 16);
            ssSetInputPortDataType(S, IN_COMMAND, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_COMMAND, 1);
            ssSetInputPortRequiredContiguous(S, IN_COMMAND, 1);
            /* Port 3: elbow_d (2x1) */
            ssSetInputPortWidth(S, IN_ELBOW, 2);
            ssSetInputPortDataType(S, IN_ELBOW, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_ELBOW, 1);
            ssSetInputPortRequiredContiguous(S, IN_ELBOW, 1);
            break;
            
        case CTRL_CARTESIAN_VELOCITIES:
            /* Port 2: O_dP_EE_d (6x1) */
            ssSetInputPortWidth(S, IN_COMMAND, 6);
            ssSetInputPortDataType(S, IN_COMMAND, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_COMMAND, 1);
            ssSetInputPortRequiredContiguous(S, IN_COMMAND, 1);
            /* Port 3: elbow_d (2x1) */
            ssSetInputPortWidth(S, IN_ELBOW, 2);
            ssSetInputPortDataType(S, IN_ELBOW, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_ELBOW, 1);
            ssSetInputPortRequiredContiguous(S, IN_ELBOW, 1);
            break;
            
        /* ================================================================
         * DUAL-CALLBACK MODES (5-8): Torque + Motion Generator
         * Port 2: tau_J_d, Port 3: motion_cmd, Port 4: elbow (Cartesian only)
         * ================================================================ */
        case CTRL_TORQUES_JOINT_POSITIONS:
            /* Port 2: tau_J_d (7x1) */
            ssSetInputPortWidth(S, IN_TAU_J_D, 7);
            ssSetInputPortDataType(S, IN_TAU_J_D, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_TAU_J_D, 1);
            ssSetInputPortRequiredContiguous(S, IN_TAU_J_D, 1);
            /* Port 3: q_d (7x1) */
            ssSetInputPortWidth(S, IN_MOTION_CMD, 7);
            ssSetInputPortDataType(S, IN_MOTION_CMD, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_MOTION_CMD, 1);
            ssSetInputPortRequiredContiguous(S, IN_MOTION_CMD, 1);
            break;
            
        case CTRL_TORQUES_JOINT_VELOCITIES:
            /* Port 2: tau_J_d (7x1) */
            ssSetInputPortWidth(S, IN_TAU_J_D, 7);
            ssSetInputPortDataType(S, IN_TAU_J_D, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_TAU_J_D, 1);
            ssSetInputPortRequiredContiguous(S, IN_TAU_J_D, 1);
            /* Port 3: dq_d (7x1) */
            ssSetInputPortWidth(S, IN_MOTION_CMD, 7);
            ssSetInputPortDataType(S, IN_MOTION_CMD, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_MOTION_CMD, 1);
            ssSetInputPortRequiredContiguous(S, IN_MOTION_CMD, 1);
            break;
            
        case CTRL_TORQUES_CARTESIAN_POSE:
            /* Port 2: tau_J_d (7x1) */
            ssSetInputPortWidth(S, IN_TAU_J_D, 7);
            ssSetInputPortDataType(S, IN_TAU_J_D, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_TAU_J_D, 1);
            ssSetInputPortRequiredContiguous(S, IN_TAU_J_D, 1);
            /* Port 3: O_T_EE_d (16x1, 4x4 col-major) */
            ssSetInputPortWidth(S, IN_MOTION_CMD, 16);
            ssSetInputPortDataType(S, IN_MOTION_CMD, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_MOTION_CMD, 1);
            ssSetInputPortRequiredContiguous(S, IN_MOTION_CMD, 1);
            /* Port 4: elbow_d (2x1) */
            ssSetInputPortWidth(S, IN_ELBOW_DUAL, 2);
            ssSetInputPortDataType(S, IN_ELBOW_DUAL, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_ELBOW_DUAL, 1);
            ssSetInputPortRequiredContiguous(S, IN_ELBOW_DUAL, 1);
            break;
            
        case CTRL_TORQUES_CARTESIAN_VELOCITIES:
            /* Port 2: tau_J_d (7x1) */
            ssSetInputPortWidth(S, IN_TAU_J_D, 7);
            ssSetInputPortDataType(S, IN_TAU_J_D, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_TAU_J_D, 1);
            ssSetInputPortRequiredContiguous(S, IN_TAU_J_D, 1);
            /* Port 3: O_dP_EE_d (6x1) */
            ssSetInputPortWidth(S, IN_MOTION_CMD, 6);
            ssSetInputPortDataType(S, IN_MOTION_CMD, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_MOTION_CMD, 1);
            ssSetInputPortRequiredContiguous(S, IN_MOTION_CMD, 1);
            /* Port 4: elbow_d (2x1) */
            ssSetInputPortWidth(S, IN_ELBOW_DUAL, 2);
            ssSetInputPortDataType(S, IN_ELBOW_DUAL, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_ELBOW_DUAL, 1);
            ssSetInputPortRequiredContiguous(S, IN_ELBOW_DUAL, 1);
            break;
            
        default:
            /* Fallback: torque mode */
            ssSetInputPortWidth(S, IN_COMMAND, 7);
            ssSetInputPortDataType(S, IN_COMMAND, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, IN_COMMAND, 1);
            ssSetInputPortRequiredContiguous(S, IN_COMMAND, 1);
            break;
    }
    
    /* ====================================================================
     * OUTPUT PORTS
     * ==================================================================== */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    
    /* Port 0: Function-call output (MUST BE FIRST!) */
    ssSetOutputPortWidth(S, OUT_FCALL, 1);
    ssSetOutputPortDataType(S, OUT_FCALL, SS_FCN_CALL);
    
    /* Port 1: robot_state (FrankaRobotStateBus)
     * 
     * Bus output configuration:
     * - The bus object 'FrankaRobotStateBus' must exist in base workspace
     * - Register the bus as a data type and use it for the port
     * - Output as nonvirtual bus (struct in generated code)
     * 
     * For code generation, the TLC uses the C struct FrankaRobotStateBus
     * from franka_simulink_types.h which matches the bus layout.
     */
#if defined(MATLAB_MEX_FILE)
    {
        /* Register bus object as a data type */
        DTypeId busTypeId;
        ssRegisterTypeFromNamedObject(S, "FrankaRobotStateBus", &busTypeId);
        ssSetOutputPortDataType(S, OUT_STATE, busTypeId);
    }
#endif
    ssSetOutputPortWidth(S, OUT_STATE, 1);
    ssSetBusOutputObjectName(S, OUT_STATE, (void*)"FrankaRobotStateBus");
    ssSetBusOutputAsStruct(S, OUT_STATE, 1);

    /* Port 2: model_data (FrankaModelDataBus)
     * 
     * Computed dynamics and kinematics from libfranka Model class.
     */
#if defined(MATLAB_MEX_FILE)
    {
        DTypeId modelBusTypeId;
        ssRegisterTypeFromNamedObject(S, "FrankaModelDataBus", &modelBusTypeId);
        ssSetOutputPortDataType(S, OUT_MODEL, modelBusTypeId);
    }
#endif
    ssSetOutputPortWidth(S, OUT_MODEL, 1);
    ssSetBusOutputObjectName(S, OUT_MODEL, (void*)"FrankaModelDataBus");
    ssSetBusOutputAsStruct(S, OUT_MODEL, 1);

    /* Port 3: dt_sec (1x1) - control period from callback */
    ssSetOutputPortWidth(S, OUT_DT_SEC, 1);
    ssSetOutputPortDataType(S, OUT_DT_SEC, SS_DOUBLE);
    
    /* ====================================================================
     * SAMPLE TIME
     * ==================================================================== */
    ssSetNumSampleTimes(S, 1);
    
    /* ====================================================================
     * WORK VECTORS
     * ==================================================================== */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    /* DWork for edge detection */
    ssSetNumDWork(S, NUM_DWORK);
    ssSetDWorkWidth(S, DWORK_PREV_ENABLE, 1);
    ssSetDWorkDataType(S, DWORK_PREV_ENABLE, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_ENABLE, "PrevEnable");
    
    ssSetDWorkWidth(S, DWORK_PREV_ERROR_RECOVERY, 1);
    ssSetDWorkDataType(S, DWORK_PREV_ERROR_RECOVERY, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_ERROR_RECOVERY, "PrevErrorRecovery");
    
    ssSetDWorkWidth(S, DWORK_PREV_READ_ONCE, 1);
    ssSetDWorkDataType(S, DWORK_PREV_READ_ONCE, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_READ_ONCE, "PrevReadOnce");
    
    /* Options */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    ssSetOptions(S, SS_OPTION_CALL_TERMINATE_ON_EXIT);
}

/* ========================================================================
 * mdlInitializeSampleTimes - Initialize sample times
 * ======================================================================== */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    /* Inherited sample time */
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    
    /* Register function-call output port */
    ssSetCallSystemOutput(S, OUT_FCALL);
}

/* ========================================================================
 * mdlStart - Initialize
 * ======================================================================== */
#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
    /* Initialize DWork */
    real_T *prevEnable = (real_T*)ssGetDWork(S, DWORK_PREV_ENABLE);
    *prevEnable = 0.0;
    
    real_T *prevErrorRecovery = (real_T*)ssGetDWork(S, DWORK_PREV_ERROR_RECOVERY);
    *prevErrorRecovery = 0.0;
    
    real_T *prevReadOnce = (real_T*)ssGetDWork(S, DWORK_PREV_READ_ONCE);
    *prevReadOnce = 0.0;
}
#endif

/* ========================================================================
 * mdlOutputs - Compute outputs
 * ======================================================================== */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* For simulation only - actual implementation is in TLC.
     * 
     * The robot_state bus output is handled by Simulink's bus infrastructure.
     * We just need to ensure dt_sec is zeroed for simulation.
     */
    real_T *dt_sec = ssGetOutputPortRealSignal(S, OUT_DT_SEC);
    dt_sec[0] = 0.0;
    
    /* Note: The bus output (OUT_STATE) is automatically initialized to zero
     * by Simulink when using bus objects. No manual zeroing needed.
     */
    
    UNUSED_ARG(tid);
}

/* ========================================================================
 * mdlTerminate - Cleanup
 * ======================================================================== */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S);
}

/* ========================================================================
 * mdlRTW - Write parameters to RTW file for code generation
 * ======================================================================== */
#if defined(MATLAB_MEX_FILE)
#define MDL_RTW
static void mdlRTW(SimStruct *S)
{
    /* Write control_mode parameter to RTW file for TLC access */
    int control_mode = static_cast<int>(mxGetScalar(ssGetSFcnParam(S, PARAM_CONTROL_MODE)));
    
    if (!ssWriteRTWParamSettings(S, 1,
            SSWRITE_VALUE_NUM, "ControlMode", (real_T)control_mode)) {
        return; /* Error message already set by ssWriteRTWParamSettings */
    }
}
#endif

/* ========================================================================
 * Required S-function trailer
 * ======================================================================== */
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
