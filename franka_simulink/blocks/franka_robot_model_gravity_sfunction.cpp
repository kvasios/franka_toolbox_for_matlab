/*
 * franka_robot_model_gravity_sfunction.cpp - C++ Level-2 S-Function
 *
 * Computes the gravity compensation torques g(q) using libfranka's Model class.
 *
 * Parameters:
 *   0. use_custom_gravity - 0 = default {0,0,-9.81}, 1 = use input
 *
 * Inputs (use_custom_gravity = 0):
 *   0. robot_ip   (16x1 uint8)  - Robot IP as ASCII chars
 *   1. q          (7x1 double)  - Joint positions [rad]
 *   2. m_total    (1x1 double)  - Total load mass [kg]
 *   3. F_x_Ctotal (3x1 double)  - Total load CoM in flange frame [m]
 *
 * Inputs (use_custom_gravity = 1):
 *   0-3. Same as above
 *   4. gravity_earth (3x1 double) - Custom gravity vector [m/s^2]
 *
 * Outputs:
 *   0. gravity    (7x1 double)  - Gravity compensation torques [Nm]
 *
 * Copyright (c) 2025 Franka Robotics GmbH
 */

#define S_FUNCTION_NAME  franka_robot_model_gravity_sfunction
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

/* Parameters */
#define PARAM_USE_CUSTOM_GRAVITY  0
#define NUM_PARAMS                1

/* Input port indices */
#define IN_ROBOT_IP       0
#define IN_Q              1
#define IN_M_TOTAL        2
#define IN_F_X_CTOTAL     3
#define IN_GRAVITY_EARTH  4   /* Only if use_custom_gravity = 1 */

/* Output port indices */
#define OUT_GRAVITY       0
#define NUM_OUTPUTS       1

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

    /* Make parameter non-tunable */
    ssSetSFcnParamTunable(S, PARAM_USE_CUSTOM_GRAVITY, SS_PRM_NOT_TUNABLE);

    /* Get parameter */
    int use_custom_gravity = (int)mxGetScalar(ssGetSFcnParam(S, PARAM_USE_CUSTOM_GRAVITY));
    int numInputs = use_custom_gravity ? 5 : 4;

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    
    /* INPUT PORTS */
    if (!ssSetNumInputPorts(S, numInputs)) return;
    
    /* Port 0: robot_ip (16 ASCII chars) */
    ssSetInputPortWidth(S, IN_ROBOT_IP, 16);
    ssSetInputPortDataType(S, IN_ROBOT_IP, SS_UINT8);
    ssSetInputPortDirectFeedThrough(S, IN_ROBOT_IP, 1);
    ssSetInputPortRequiredContiguous(S, IN_ROBOT_IP, 1);
    
    /* Port 1: q - Joint positions (7x1) */
    ssSetInputPortWidth(S, IN_Q, 7);
    ssSetInputPortDataType(S, IN_Q, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_Q, 1);
    ssSetInputPortRequiredContiguous(S, IN_Q, 1);
    
    /* Port 2: m_total - Load mass (1x1) */
    ssSetInputPortWidth(S, IN_M_TOTAL, 1);
    ssSetInputPortDataType(S, IN_M_TOTAL, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_M_TOTAL, 1);
    ssSetInputPortRequiredContiguous(S, IN_M_TOTAL, 1);
    
    /* Port 3: F_x_Ctotal - Load CoM (3x1) */
    ssSetInputPortWidth(S, IN_F_X_CTOTAL, 3);
    ssSetInputPortDataType(S, IN_F_X_CTOTAL, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_F_X_CTOTAL, 1);
    ssSetInputPortRequiredContiguous(S, IN_F_X_CTOTAL, 1);
    
    /* Port 4: gravity_earth (optional, 3x1) */
    if (use_custom_gravity) {
        ssSetInputPortWidth(S, IN_GRAVITY_EARTH, 3);
        ssSetInputPortDataType(S, IN_GRAVITY_EARTH, SS_DOUBLE);
        ssSetInputPortDirectFeedThrough(S, IN_GRAVITY_EARTH, 1);
        ssSetInputPortRequiredContiguous(S, IN_GRAVITY_EARTH, 1);
    }
    
    /* OUTPUT PORTS */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    
    /* Port 0: gravity vector (7x1) */
    ssSetOutputPortWidth(S, OUT_GRAVITY, 7);
    ssSetOutputPortDataType(S, OUT_GRAVITY, SS_DOUBLE);
    
    /* SAMPLE TIME */
    ssSetNumSampleTimes(S, 1);
    
    /* WORK VECTORS */
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetNumDWork(S, 0);
    
    /* Options */
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
 * mdlOutputs - Compute outputs (simulation only)
 * ======================================================================== */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *gravity = ssGetOutputPortRealSignal(S, OUT_GRAVITY);
    
    for (int i = 0; i < 7; i++) {
        gravity[i] = 0.0;
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
 * Required S-function trailer
 * ======================================================================== */
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
