/*
 * franka_read_state_sfunction.cpp - Read Robot State S-Function
 *
 * Reads the robot state once from the robot at the specified IP.
 * Can only succeed when no control is currently running.
 *
 * Inputs:
 *   0. robot_ip (16x1 uint8) - Robot IP address as ASCII chars
 *   1. trigger  (1x1)        - Rising edge triggers state read
 *
 * Outputs:
 *   0. robot_state (FrankaRobotStateBus) - Robot state data
 *   1. model_data  (FrankaModelDataBus)  - Computed dynamics/kinematics
 *   2. success     (1x1)                 - 1 if read succeeded, 0 otherwise
 *
 * Behavior:
 *   - On rising edge of trigger, calls FrankaRobotManager::readOnce(ip, ...)
 *   - Outputs the robot state and model data if successful
 *   - Returns success=0 if:
 *     - Control is currently running (cannot readOnce during control)
 *     - Robot is not connected
 *     - Read failed
 *
 * Copyright (c) 2025 Franka Robotics GmbH
 */

#define S_FUNCTION_NAME  franka_read_state_sfunction
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <cstring>

/* Input port indices */
#define IN_ROBOT_IP  0
#define IN_TRIGGER   1
#define NUM_INPUTS   2

/* Output port indices */
#define OUT_STATE    0
#define OUT_MODEL    1
#define OUT_SUCCESS  2
#define NUM_OUTPUTS  3

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

    /* Port 0: robot_state (FrankaRobotStateBus) */
#if defined(MATLAB_MEX_FILE)
    {
        DTypeId busTypeId;
        ssRegisterTypeFromNamedObject(S, "FrankaRobotStateBus", &busTypeId);
        ssSetOutputPortDataType(S, OUT_STATE, busTypeId);
    }
#endif
    ssSetOutputPortWidth(S, OUT_STATE, 1);
    ssSetBusOutputObjectName(S, OUT_STATE, (void*)"FrankaRobotStateBus");
    ssSetBusOutputAsStruct(S, OUT_STATE, 1);

    /* Port 1: model_data (FrankaModelDataBus) */
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

    /* Port 2: success (1x1) */
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
    
    /* In simulation, just output 0 (no actual robot connection)
     * Bus outputs are automatically zeroed by Simulink.
     */
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
