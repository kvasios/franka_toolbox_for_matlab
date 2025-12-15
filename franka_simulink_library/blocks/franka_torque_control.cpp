/*
 * franka_torque_control.cpp - C++ Level-2 S-Function for Franka robot torque control
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
 *   0. fcall     (function-call) - Triggers controller subsystem at 1kHz [MUST BE FIRST]
 *   1. q         (7x1) - Measured joint positions
 *   2. dq        (7x1) - Measured joint velocities
 *
 * Copyright (c) 2025 Franka Robotics GmbH
 */

#define S_FUNCTION_NAME  franka_torque_control
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
#define OUT_FCALL     0   /* Function-call output - MUST BE FIRST */
#define OUT_Q         1
#define OUT_DQ        2
#define NUM_OUTPUTS   3

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
    
    /* Port 1: q (7x1) */
    ssSetOutputPortWidth(S, OUT_Q, 7);
    ssSetOutputPortDataType(S, OUT_Q, SS_DOUBLE);
    
    /* Port 2: dq (7x1) */
    ssSetOutputPortWidth(S, OUT_DQ, 7);
    ssSetOutputPortDataType(S, OUT_DQ, SS_DOUBLE);
    
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
    /* 1kHz discrete sample time */
    ssSetSampleTime(S, 0, 0.001);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
    
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
    /* For simulation only - actual implementation is in TLC */
    real_T *q  = ssGetOutputPortRealSignal(S, OUT_Q);
    real_T *dq = ssGetOutputPortRealSignal(S, OUT_DQ);
    
    /* Output zeros in simulation */
    for (int i = 0; i < 7; i++) {
        q[i]  = 0.0;
        dq[i] = 0.0;
    }
    
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

