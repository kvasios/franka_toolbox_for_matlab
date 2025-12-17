/*
 * franka_robot_sfunction.cpp - C++ Level-2 S-Function for Franka robot control
 *
 * This block wraps libfranka's robot.control() with torque callback.
 * It outputs a function-call signal to trigger an external controller subsystem.
 *
 * Inputs:
 *   0. Enable    (1x1)   - Rising edge starts control, falling edge stops
 *   1. robot_ip  (16x1)  - Robot IP as ASCII chars (use String Constant + String to ASCII)
 *   2. tau_J_d   (7x1)   - Commanded joint torques (read from controller subsystem)
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

/* No parameters required - all inputs come from ports */
#define NUM_PARAMS     0

/* Input port indices */
#define IN_ENABLE     0
#define IN_ROBOT_IP   1
#define IN_TAU_J_D    2
#define NUM_INPUTS    3

/* Output port indices */
#define OUT_FCALL       0   /* Function-call output - MUST BE FIRST */
#define OUT_STATE       1   /* Robot state bus (FrankaRobotStateBus) */
#define OUT_MODEL       2   /* Model data bus (FrankaModelDataBus) */
#define OUT_DT_SEC      3   /* Control period [s] */
#define NUM_OUTPUTS     4

/* DWork indices */
#define DWORK_PREV_ENABLE 0
#define NUM_DWORK         1

/* No parameters to check - all inputs come from ports */

/* ========================================================================
 * mdlInitializeSizes - Initialize block sizes
 * ======================================================================== */
static void mdlInitializeSizes(SimStruct *S)
{
    /* No parameters - all inputs come from ports */
    ssSetNumSFcnParams(S, NUM_PARAMS);
    
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by Simulink */
    }
#endif
    
    /* States */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    
    /* ====================================================================
     * INPUT PORTS
     * ==================================================================== */
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
    
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
    
    /* Port 2: tau_J_d (7x1) */
    ssSetInputPortWidth(S, IN_TAU_J_D, 7);
    ssSetInputPortDataType(S, IN_TAU_J_D, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_TAU_J_D, 1);
    ssSetInputPortRequiredContiguous(S, IN_TAU_J_D, 1);
    
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
    /* robot_ip comes from input port - no parameters to write */
    UNUSED_ARG(S);
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
