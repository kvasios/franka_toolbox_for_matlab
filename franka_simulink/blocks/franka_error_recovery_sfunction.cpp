/*
 * franka_error_recovery_sfunction.cpp - Automatic Error Recovery S-Function
 *
 * Attempts automatic error recovery on the robot at the specified IP.
 * Can only succeed when no control is currently running.
 *
 * Inputs:
 *   0. robot_ip (16x1 uint8) - Robot IP address as ASCII chars
 *   1. trigger  (1x1)        - Rising edge triggers recovery attempt
 *
 * Outputs:
 *   0. success (1x1)         - 1 if recovery succeeded, 0 otherwise
 *
 * Behavior:
 *   - On rising edge of trigger, calls FrankaRobotManager::automaticErrorRecovery(ip)
 *   - Returns success=1 if robot was in error state and recovered
 *   - Returns success=0 if:
 *     - Control is currently running (cannot recover during control)
 *     - Robot was not in error state
 *     - Robot is not connected
 *     - Recovery failed
 *
 * Copyright (c) 2025 Franka Robotics GmbH
 */

#define S_FUNCTION_NAME  franka_error_recovery_sfunction
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <cstring>

/* Input port indices */
#define IN_ROBOT_IP  0
#define IN_TRIGGER   1
#define NUM_INPUTS   2

/* Output port indices */
#define OUT_SUCCESS  0
#define NUM_OUTPUTS  1

/* DWork indices */
#define DWORK_PREV_TRIGGER 0
#define NUM_DWORK          1

/* ========================================================================
 * mdlInitializeSizes
 * ======================================================================== */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;

    /* Port 0: robot_ip (16 ASCII chars) */
    ssSetInputPortWidth(S, IN_ROBOT_IP, 16);
    ssSetInputPortDataType(S, IN_ROBOT_IP, SS_UINT8);
    ssSetInputPortDirectFeedThrough(S, IN_ROBOT_IP, 1);
    ssSetInputPortRequiredContiguous(S, IN_ROBOT_IP, 1);

    /* Port 1: trigger (1x1) */
    ssSetInputPortWidth(S, IN_TRIGGER, 1);
    ssSetInputPortDataType(S, IN_TRIGGER, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_TRIGGER, 1);
    ssSetInputPortRequiredContiguous(S, IN_TRIGGER, 1);

    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;

    /* Port 0: success (1x1) */
    ssSetOutputPortWidth(S, OUT_SUCCESS, 1);
    ssSetOutputPortDataType(S, OUT_SUCCESS, SS_DOUBLE);

    ssSetNumSampleTimes(S, 1);

    /* DWork for edge detection */
    ssSetNumDWork(S, NUM_DWORK);
    ssSetDWorkWidth(S, DWORK_PREV_TRIGGER, 1);
    ssSetDWorkDataType(S, DWORK_PREV_TRIGGER, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_TRIGGER, "PrevTrigger");

    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    ssSetOptions(S, SS_OPTION_WORKS_WITH_CODE_REUSE |
                    SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}

/* ========================================================================
 * mdlInitializeSampleTimes
 * ======================================================================== */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

/* ========================================================================
 * mdlStart
 * ======================================================================== */
#define MDL_START
static void mdlStart(SimStruct *S)
{
    real_T *prevTrigger = (real_T*)ssGetDWork(S, DWORK_PREV_TRIGGER);
    *prevTrigger = 0.0;
}

/* ========================================================================
 * mdlOutputs - Simulation only
 * ======================================================================== */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *success = ssGetOutputPortRealSignal(S, OUT_SUCCESS);
    
    /* In simulation, just output 0 (no actual robot connection) */
    *success = 0.0;
    
    UNUSED_ARG(tid);
}

/* ========================================================================
 * mdlTerminate
 * ======================================================================== */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S);
}

/* ========================================================================
 * Required S-function trailer
 * ======================================================================== */
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
