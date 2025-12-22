/*
 * franka_vacuum_gripper_sfunction.cpp - C++ Level-2 S-Function for Franka vacuum gripper control
 *
 * This block wraps libfranka's VacuumGripper class with async command execution.
 * Commands are triggered by rising edge signals.
 *
 * Inputs:
 *   0. robot_ip      (16x1 uint8)                      - Robot IP as ASCII chars
 *   1. command       (FrankaVacuumGripperCommandBus)   - Command parameters bus
 *   2. vacuum        (1x1)                             - Rising edge triggers vacuum
 *   3. drop_off      (1x1)                             - Rising edge triggers drop off
 *   4. stop          (1x1)                             - Rising edge triggers stop
 *   5. read_state    (1x1)                             - Rising edge forces state read
 *
 * Outputs:
 *   0. gripper_state (FrankaVacuumGripperStateBus)     - Complete vacuum gripper state with status
 *
 * Bus Definitions:
 *   The bus I/O uses types that must exist in base workspace:
 *     FrankaVacuumGripperStateBus = franka_vacuum_gripper_state_bus();
 *     FrankaVacuumGripperCommandBus = franka_vacuum_gripper_command_bus();
 *   Or simply call: franka_setup_bus();
 *
 * Copyright (c) 2025 Franka Robotics GmbH
 */

#define S_FUNCTION_NAME  franka_vacuum_gripper_sfunction
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include <string>
#include <cstring>

/* Parameters - none for this block */
#define NUM_PARAMS  0

/* Input port indices */
#define IN_ROBOT_IP     0   /* Robot IP as ASCII chars (16x1 uint8) */
#define IN_COMMAND      1   /* Command parameters bus */
#define IN_VACUUM       2   /* Vacuum trigger (1x1) */
#define IN_DROP_OFF     3   /* Drop off trigger (1x1) */
#define IN_STOP         4   /* Stop trigger (1x1) */
#define IN_READ_STATE   5   /* Read state trigger (1x1) */
#define NUM_INPUTS      6

/* Output port indices */
#define OUT_STATE       0   /* Vacuum gripper state bus */
#define NUM_OUTPUTS     1

/* DWork indices for edge detection */
#define DWORK_PREV_VACUUM       0
#define DWORK_PREV_DROP_OFF     1
#define DWORK_PREV_STOP         2
#define DWORK_PREV_READ_STATE   3
#define NUM_DWORK               4

/* ========================================================================
 * mdlInitializeSizes - Initialize block sizes
 * ======================================================================== */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }
#endif

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    
    /* ====================================================================
     * INPUT PORTS
     * ==================================================================== */
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
    
    /* Port 0: robot_ip (16 ASCII chars) */
    ssSetInputPortWidth(S, IN_ROBOT_IP, 16);
    ssSetInputPortDataType(S, IN_ROBOT_IP, SS_UINT8);
    ssSetInputPortDirectFeedThrough(S, IN_ROBOT_IP, 1);
    ssSetInputPortRequiredContiguous(S, IN_ROBOT_IP, 1);
    
    /* Port 1: command (FrankaVacuumGripperCommandBus) */
#if defined(MATLAB_MEX_FILE)
    {
        DTypeId commandBusTypeId;
        ssRegisterTypeFromNamedObject(S, "FrankaVacuumGripperCommandBus", &commandBusTypeId);
        ssSetInputPortDataType(S, IN_COMMAND, commandBusTypeId);
    }
#endif
    ssSetInputPortWidth(S, IN_COMMAND, 1);
    ssSetBusInputAsStruct(S, IN_COMMAND, 1);
    ssSetInputPortDirectFeedThrough(S, IN_COMMAND, 1);
    ssSetInputPortRequiredContiguous(S, IN_COMMAND, 1);
    
    /* Port 2: vacuum trigger (1x1) */
    ssSetInputPortWidth(S, IN_VACUUM, 1);
    ssSetInputPortDataType(S, IN_VACUUM, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_VACUUM, 1);
    ssSetInputPortRequiredContiguous(S, IN_VACUUM, 1);
    
    /* Port 3: drop_off trigger (1x1) */
    ssSetInputPortWidth(S, IN_DROP_OFF, 1);
    ssSetInputPortDataType(S, IN_DROP_OFF, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_DROP_OFF, 1);
    ssSetInputPortRequiredContiguous(S, IN_DROP_OFF, 1);
    
    /* Port 4: stop trigger (1x1) */
    ssSetInputPortWidth(S, IN_STOP, 1);
    ssSetInputPortDataType(S, IN_STOP, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_STOP, 1);
    ssSetInputPortRequiredContiguous(S, IN_STOP, 1);
    
    /* Port 5: read_state trigger (1x1) */
    ssSetInputPortWidth(S, IN_READ_STATE, 1);
    ssSetInputPortDataType(S, IN_READ_STATE, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_READ_STATE, 1);
    ssSetInputPortRequiredContiguous(S, IN_READ_STATE, 1);
    
    /* ====================================================================
     * OUTPUT PORTS
     * ==================================================================== */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    
    /* Port 0: gripper_state (FrankaVacuumGripperStateBus) */
#if defined(MATLAB_MEX_FILE)
    {
        DTypeId stateBusTypeId;
        ssRegisterTypeFromNamedObject(S, "FrankaVacuumGripperStateBus", &stateBusTypeId);
        ssSetOutputPortDataType(S, OUT_STATE, stateBusTypeId);
    }
#endif
    ssSetOutputPortWidth(S, OUT_STATE, 1);
    ssSetBusOutputObjectName(S, OUT_STATE, (void*)"FrankaVacuumGripperStateBus");
    ssSetBusOutputAsStruct(S, OUT_STATE, 1);
    
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
    
    ssSetDWorkWidth(S, DWORK_PREV_VACUUM, 1);
    ssSetDWorkDataType(S, DWORK_PREV_VACUUM, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_VACUUM, "PrevVacuum");
    
    ssSetDWorkWidth(S, DWORK_PREV_DROP_OFF, 1);
    ssSetDWorkDataType(S, DWORK_PREV_DROP_OFF, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_DROP_OFF, "PrevDropOff");
    
    ssSetDWorkWidth(S, DWORK_PREV_STOP, 1);
    ssSetDWorkDataType(S, DWORK_PREV_STOP, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_STOP, "PrevStop");
    
    ssSetDWorkWidth(S, DWORK_PREV_READ_STATE, 1);
    ssSetDWorkDataType(S, DWORK_PREV_READ_STATE, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_READ_STATE, "PrevReadState");
    
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    ssSetOptions(S, SS_OPTION_CALL_TERMINATE_ON_EXIT);
}

/* ========================================================================
 * mdlInitializeSampleTimes - Initialize sample times
 * ======================================================================== */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

/* ========================================================================
 * mdlStart - Initialize
 * ======================================================================== */
#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
    real_T *prevVacuum = (real_T*)ssGetDWork(S, DWORK_PREV_VACUUM);
    *prevVacuum = 0.0;
    
    real_T *prevDropOff = (real_T*)ssGetDWork(S, DWORK_PREV_DROP_OFF);
    *prevDropOff = 0.0;
    
    real_T *prevStop = (real_T*)ssGetDWork(S, DWORK_PREV_STOP);
    *prevStop = 0.0;
    
    real_T *prevReadState = (real_T*)ssGetDWork(S, DWORK_PREV_READ_STATE);
    *prevReadState = 0.0;
}
#endif

/* ========================================================================
 * mdlOutputs - Compute outputs
 * ======================================================================== */
static void mdlOutputs(SimStruct *S, int_T tid)
{
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
 * Required S-function trailer
 * ======================================================================== */
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
