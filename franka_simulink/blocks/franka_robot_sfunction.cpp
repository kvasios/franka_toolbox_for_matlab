/*
 * franka_robot_sfunction.cpp - C++ Level-2 S-Function for Franka robot control
 *
 * This block wraps libfranka's robot.control() with multiple control mode support.
 * It outputs a function-call signal to trigger an external controller subsystem.
 *
 * Parameters:
 *   control_mode - Control mode selection (0-4):
 *     0: Torques           - Direct torque control
 *     1: JointPositions    - Joint position control with internal impedance
 *     2: JointVelocities   - Joint velocity control with internal impedance
 *     3: CartesianPose     - Cartesian pose control with internal impedance
 *     4: CartesianVelocities - Cartesian velocity control with internal impedance
 *
 * Inputs:
 *   0. Enable    (1x1)   - Rising edge starts control, falling edge stops
 *   1. robot_ip  (16x1)  - Robot IP as ASCII chars (use String Constant + String to ASCII)
 *   2. command   (varies)- Mode-dependent command input:
 *                          Mode 0: tau_J_d (7x1)    - Commanded joint torques [Nm]
 *                          Mode 1: q_d (7x1)        - Commanded joint positions [rad]
 *                          Mode 2: dq_d (7x1)       - Commanded joint velocities [rad/s]
 *                          Mode 3: O_T_EE_d (16x1)  - Commanded EE pose (4x4 col-major) [m]
 *                          Mode 4: O_dP_EE_d (6x1)  - Commanded EE velocity [m/s, rad/s]
 *   3. elbow_d   (2x1)   - Elbow configuration (only for modes 3-4)
 *                          [elbow_position, elbow_sign]
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

/* Control mode enum (matches block mask dropdown order) */
enum FrankaControlMode {
    CTRL_TORQUES = 0,
    CTRL_JOINT_POSITIONS = 1,
    CTRL_JOINT_VELOCITIES = 2,
    CTRL_CARTESIAN_POSE = 3,
    CTRL_CARTESIAN_VELOCITIES = 4
};

/* Input port indices */
#define IN_ENABLE     0
#define IN_ROBOT_IP   1
#define IN_COMMAND    2  /* Command input (tau_J_d, q_d, dq_d, O_T_EE_d, or O_dP_EE_d) */
#define IN_ELBOW      3  /* Elbow input (only for Cartesian modes) */
#define NUM_INPUTS_BASE    3  /* Modes 0-2: Enable, robot_ip, command */
#define NUM_INPUTS_CART    4  /* Modes 3-4: Enable, robot_ip, command, elbow */

/* Output port indices */
#define OUT_FCALL       0   /* Function-call output - MUST BE FIRST */
#define OUT_STATE       1   /* Robot state bus (FrankaRobotStateBus) */
#define OUT_MODEL       2   /* Model data bus (FrankaModelDataBus) */
#define OUT_DT_SEC      3   /* Control period [s] */
#define NUM_OUTPUTS     4

/* DWork indices */
#define DWORK_PREV_ENABLE 0
#define NUM_DWORK         1

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
    int num_inputs = (control_mode == CTRL_CARTESIAN_POSE || 
                      control_mode == CTRL_CARTESIAN_VELOCITIES) 
                     ? NUM_INPUTS_CART : NUM_INPUTS_BASE;
    
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
    
    /* Port 2: Command input (size depends on control mode) */
    int command_size;
    switch (control_mode) {
        case CTRL_TORQUES:           command_size = 7;  break;  /* tau_J_d */
        case CTRL_JOINT_POSITIONS:   command_size = 7;  break;  /* q_d */
        case CTRL_JOINT_VELOCITIES:  command_size = 7;  break;  /* dq_d */
        case CTRL_CARTESIAN_POSE:    command_size = 16; break;  /* O_T_EE_d (4x4) */
        case CTRL_CARTESIAN_VELOCITIES: command_size = 6; break; /* O_dP_EE_d (6x1) */
        default:                     command_size = 7;  break;  /* Default to torques */
    }
    ssSetInputPortWidth(S, IN_COMMAND, command_size);
    ssSetInputPortDataType(S, IN_COMMAND, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_COMMAND, 1);
    ssSetInputPortRequiredContiguous(S, IN_COMMAND, 1);
    
    /* Port 3: Elbow input (only for Cartesian modes) */
    if (num_inputs == NUM_INPUTS_CART) {
        ssSetInputPortWidth(S, IN_ELBOW, 2);  /* elbow_d: [position, sign] */
        ssSetInputPortDataType(S, IN_ELBOW, SS_DOUBLE);
        ssSetInputPortDirectFeedThrough(S, IN_ELBOW, 1);
        ssSetInputPortRequiredContiguous(S, IN_ELBOW, 1);
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
