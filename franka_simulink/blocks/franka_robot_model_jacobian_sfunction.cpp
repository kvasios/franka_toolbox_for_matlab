/*
 * franka_robot_model_jacobian_sfunction.cpp - C++ Level-2 S-Function
 *
 * Computes the 6x7 Jacobian matrix using libfranka's Model class.
 *
 * Parameters:
 *   0. jacobian_type - 0 = zeroJacobian (base frame), 1 = bodyJacobian
 *   1. frame         - 0-6 = Joint1-7, 7 = Flange, 8 = EndEffector, 9 = Stiffness
 *
 * Inputs:
 *   0. robot_ip (16x1 uint8)  - Robot IP as ASCII chars
 *   1. q        (7x1 double)  - Joint positions [rad]
 *   2. F_T_EE   (4x4 double)  - End effector in flange frame
 *   3. EE_T_K   (4x4 double)  - Stiffness frame in EE frame
 *
 * Outputs:
 *   0. jacobian (6x7 double)  - Jacobian matrix
 *
 * Copyright (c) 2025 Franka Robotics GmbH
 */

#define S_FUNCTION_NAME  franka_robot_model_jacobian_sfunction
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

/* Parameters */
#define PARAM_JACOBIAN_TYPE  0
#define PARAM_FRAME          1
#define NUM_PARAMS           2

/* Input port indices */
#define IN_ROBOT_IP   0
#define IN_Q          1
#define IN_F_T_EE     2
#define IN_EE_T_K     3
#define NUM_INPUTS    4

/* Output port indices */
#define OUT_JACOBIAN  0
#define NUM_OUTPUTS   1

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

    /* Make parameters non-tunable */
    ssSetSFcnParamTunable(S, PARAM_JACOBIAN_TYPE, SS_PRM_NOT_TUNABLE);
    ssSetSFcnParamTunable(S, PARAM_FRAME, SS_PRM_NOT_TUNABLE);

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    
    /* INPUT PORTS */
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
    
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
    
    /* Port 2: F_T_EE (4x4) */
    ssSetInputPortMatrixDimensions(S, IN_F_T_EE, 4, 4);
    ssSetInputPortDataType(S, IN_F_T_EE, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_F_T_EE, 1);
    ssSetInputPortRequiredContiguous(S, IN_F_T_EE, 1);
    
    /* Port 3: EE_T_K (4x4) */
    ssSetInputPortMatrixDimensions(S, IN_EE_T_K, 4, 4);
    ssSetInputPortDataType(S, IN_EE_T_K, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_EE_T_K, 1);
    ssSetInputPortRequiredContiguous(S, IN_EE_T_K, 1);
    
    /* OUTPUT PORTS */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    
    /* Port 0: jacobian (6x7) */
    ssSetOutputPortMatrixDimensions(S, OUT_JACOBIAN, 6, 7);
    ssSetOutputPortDataType(S, OUT_JACOBIAN, SS_DOUBLE);
    
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
    real_T *jacobian = ssGetOutputPortRealSignal(S, OUT_JACOBIAN);
    
    for (int i = 0; i < 42; i++) {
        jacobian[i] = 0.0;
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
    int jacobian_type = static_cast<int>(mxGetScalar(ssGetSFcnParam(S, PARAM_JACOBIAN_TYPE)));
    int frame = static_cast<int>(mxGetScalar(ssGetSFcnParam(S, PARAM_FRAME)));
    
    if (!ssWriteRTWParamSettings(S, 2,
            SSWRITE_VALUE_NUM, "jacobian_type", (real_T)jacobian_type,
            SSWRITE_VALUE_NUM, "frame", (real_T)frame)) {
        return;
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

