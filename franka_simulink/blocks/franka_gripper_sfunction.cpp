/*
 * franka_gripper_sfunction.cpp - C++ Level-2 S-Function for Franka gripper control
 *
 * This block wraps libfranka's Gripper class with async command execution.
 * Commands are triggered by rising edge signals (like read_once on the robot block).
 *
 * Inputs:
 *   0. robot_ip      (16x1 uint8)              - Robot IP as ASCII chars
 *   1. command       (FrankaGripperCommandBus) - Command parameters bus
 *   2. homing        (1x1)                     - Rising edge triggers homing
 *   3. grasp         (1x1)                     - Rising edge triggers grasp
 *   4. move          (1x1)                     - Rising edge triggers move
 *   5. stop          (1x1)                     - Rising edge triggers stop
 *   6. read_state    (1x1)                     - Rising edge forces state read
 *
 * Outputs:
 *   0. gripper_state (FrankaGripperStateBus)   - Complete gripper state with status
 *
 * Bus Definitions:
 *   The bus I/O uses types that must exist in base workspace:
 *     FrankaGripperStateBus = franka_gripper_state_bus();
 *     FrankaGripperCommandBus = franka_gripper_command_bus();
 *   Or simply call: franka_setup_bus();
 *
 * Copyright (c) 2025 Franka Robotics GmbH
 */

#define S_FUNCTION_NAME  franka_gripper_sfunction
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include <string>
#include <cstring>

/* Parameters - none for this block, all configuration via bus input */
#define NUM_PARAMS  0

/* Input port indices */
#define IN_ROBOT_IP     0   /* Robot IP as ASCII chars (16x1 uint8) */
#define IN_COMMAND      1   /* Command parameters bus (FrankaGripperCommandBus) */
#define IN_HOMING       2   /* Homing trigger (1x1) */
#define IN_GRASP        3   /* Grasp trigger (1x1) */
#define IN_MOVE         4   /* Move trigger (1x1) */
#define IN_STOP         5   /* Stop trigger (1x1) */
#define IN_READ_STATE   6   /* Read state trigger (1x1) */
#define NUM_INPUTS      7

/* Output port indices */
#define OUT_STATE       0   /* Gripper state bus (FrankaGripperStateBus) */
#define NUM_OUTPUTS     1

/* DWork indices for edge detection */
#define DWORK_PREV_HOMING       0
#define DWORK_PREV_GRASP        1
#define DWORK_PREV_MOVE         2
#define DWORK_PREV_STOP         3
#define DWORK_PREV_READ_STATE   4
#define NUM_DWORK               5

/* ========================================================================
 * mdlInitializeSizes - Initialize block sizes
 * ======================================================================== */
static void mdlInitializeSizes(SimStruct *S)
{
    /* Parameters: none */
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
    
    /* Port 0: robot_ip (16 ASCII chars, null-terminated) */
    ssSetInputPortWidth(S, IN_ROBOT_IP, 16);
    ssSetInputPortDataType(S, IN_ROBOT_IP, SS_UINT8);
    ssSetInputPortDirectFeedThrough(S, IN_ROBOT_IP, 1);
    ssSetInputPortRequiredContiguous(S, IN_ROBOT_IP, 1);
    
    /* Port 1: command (FrankaGripperCommandBus) */
#if defined(MATLAB_MEX_FILE)
    {
        DTypeId commandBusTypeId;
        ssRegisterTypeFromNamedObject(S, "FrankaGripperCommandBus", &commandBusTypeId);
        ssSetInputPortDataType(S, IN_COMMAND, commandBusTypeId);
    }
#endif
    ssSetInputPortWidth(S, IN_COMMAND, 1);
    ssSetBusInputAsStruct(S, IN_COMMAND, 1);
    ssSetInputPortDirectFeedThrough(S, IN_COMMAND, 1);
    ssSetInputPortRequiredContiguous(S, IN_COMMAND, 1);
    
    /* Port 2: homing trigger (1x1) */
    ssSetInputPortWidth(S, IN_HOMING, 1);
    ssSetInputPortDataType(S, IN_HOMING, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_HOMING, 1);
    ssSetInputPortRequiredContiguous(S, IN_HOMING, 1);
    
    /* Port 3: grasp trigger (1x1) */
    ssSetInputPortWidth(S, IN_GRASP, 1);
    ssSetInputPortDataType(S, IN_GRASP, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_GRASP, 1);
    ssSetInputPortRequiredContiguous(S, IN_GRASP, 1);
    
    /* Port 4: move trigger (1x1) */
    ssSetInputPortWidth(S, IN_MOVE, 1);
    ssSetInputPortDataType(S, IN_MOVE, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_MOVE, 1);
    ssSetInputPortRequiredContiguous(S, IN_MOVE, 1);
    
    /* Port 5: stop trigger (1x1) */
    ssSetInputPortWidth(S, IN_STOP, 1);
    ssSetInputPortDataType(S, IN_STOP, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_STOP, 1);
    ssSetInputPortRequiredContiguous(S, IN_STOP, 1);
    
    /* Port 6: read_state trigger (1x1) */
    ssSetInputPortWidth(S, IN_READ_STATE, 1);
    ssSetInputPortDataType(S, IN_READ_STATE, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_READ_STATE, 1);
    ssSetInputPortRequiredContiguous(S, IN_READ_STATE, 1);
    
    /* ====================================================================
     * OUTPUT PORTS
     * ==================================================================== */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    
    /* Port 0: gripper_state (FrankaGripperStateBus) */
#if defined(MATLAB_MEX_FILE)
    {
        DTypeId stateBusTypeId;
        ssRegisterTypeFromNamedObject(S, "FrankaGripperStateBus", &stateBusTypeId);
        ssSetOutputPortDataType(S, OUT_STATE, stateBusTypeId);
    }
#endif
    ssSetOutputPortWidth(S, OUT_STATE, 1);
    ssSetBusOutputObjectName(S, OUT_STATE, (void*)"FrankaGripperStateBus");
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
    
    ssSetDWorkWidth(S, DWORK_PREV_HOMING, 1);
    ssSetDWorkDataType(S, DWORK_PREV_HOMING, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_HOMING, "PrevHoming");
    
    ssSetDWorkWidth(S, DWORK_PREV_GRASP, 1);
    ssSetDWorkDataType(S, DWORK_PREV_GRASP, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_GRASP, "PrevGrasp");
    
    ssSetDWorkWidth(S, DWORK_PREV_MOVE, 1);
    ssSetDWorkDataType(S, DWORK_PREV_MOVE, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_MOVE, "PrevMove");
    
    ssSetDWorkWidth(S, DWORK_PREV_STOP, 1);
    ssSetDWorkDataType(S, DWORK_PREV_STOP, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_STOP, "PrevStop");
    
    ssSetDWorkWidth(S, DWORK_PREV_READ_STATE, 1);
    ssSetDWorkDataType(S, DWORK_PREV_READ_STATE, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_READ_STATE, "PrevReadState");
    
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
}

/* ========================================================================
 * mdlStart - Initialize
 * ======================================================================== */
#define MDL_START
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
    /* Initialize DWork for edge detection */
    real_T *prevHoming = (real_T*)ssGetDWork(S, DWORK_PREV_HOMING);
    *prevHoming = 0.0;
    
    real_T *prevGrasp = (real_T*)ssGetDWork(S, DWORK_PREV_GRASP);
    *prevGrasp = 0.0;
    
    real_T *prevMove = (real_T*)ssGetDWork(S, DWORK_PREV_MOVE);
    *prevMove = 0.0;
    
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
    /* For simulation only - actual implementation is in TLC.
     * Bus output is automatically initialized to zero by Simulink.
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
 * Required S-function trailer
 * ======================================================================== */
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
