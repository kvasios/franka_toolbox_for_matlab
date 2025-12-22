/*
 * franka_robot_model_mass_sfunction.cpp - C++ Level-2 S-Function
 *
 * Computes the 7x7 mass matrix M(q) using libfranka's Model class.
 *
 * Inputs:
 *   0. robot_ip   (16x1 uint8)  - Robot IP as ASCII chars
 *   1. q          (7x1 double)  - Joint positions [rad]
 *   2. I_total    (3x3 double)  - Total load inertia [kg*m^2]
 *   3. m_total    (1x1 double)  - Total load mass [kg]
 *   4. F_x_Ctotal (3x1 double)  - Total load CoM in flange frame [m]
 *
 * Outputs:
 *   0. mass       (7x7 double)  - Mass matrix M(q) [kg*m^2]
 *
 * Copyright (c) 2025 Franka Robotics GmbH
 */

#define S_FUNCTION_NAME  franka_robot_model_mass_sfunction
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

/* Parameters - none */
#define NUM_PARAMS  0

/* Input port indices */
#define IN_ROBOT_IP     0   /* Robot IP (16x1 uint8) */
#define IN_Q            1   /* Joint positions (7x1) */
#define IN_I_TOTAL      2   /* Load inertia (3x3) */
#define IN_M_TOTAL      3   /* Load mass (1x1) */
#define IN_F_X_CTOTAL   4   /* Load CoM (3x1) */
#define NUM_INPUTS      5

/* Output port indices */
#define OUT_MASS        0   /* Mass matrix (7x7) */
#define NUM_OUTPUTS     1

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
    
    /* Port 2: I_total - Load inertia (3x3 = 9 elements) */
    ssSetInputPortMatrixDimensions(S, IN_I_TOTAL, 3, 3);
    ssSetInputPortDataType(S, IN_I_TOTAL, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_I_TOTAL, 1);
    ssSetInputPortRequiredContiguous(S, IN_I_TOTAL, 1);
    
    /* Port 3: m_total - Load mass (1x1) */
    ssSetInputPortWidth(S, IN_M_TOTAL, 1);
    ssSetInputPortDataType(S, IN_M_TOTAL, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_M_TOTAL, 1);
    ssSetInputPortRequiredContiguous(S, IN_M_TOTAL, 1);
    
    /* Port 4: F_x_Ctotal - Load CoM (3x1) */
    ssSetInputPortWidth(S, IN_F_X_CTOTAL, 3);
    ssSetInputPortDataType(S, IN_F_X_CTOTAL, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_F_X_CTOTAL, 1);
    ssSetInputPortRequiredContiguous(S, IN_F_X_CTOTAL, 1);
    
    /* OUTPUT PORTS */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    
    /* Port 0: mass matrix (7x7 = 49 elements) */
    ssSetOutputPortMatrixDimensions(S, OUT_MASS, 7, 7);
    ssSetOutputPortDataType(S, OUT_MASS, SS_DOUBLE);
    
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
    /* For simulation only - actual implementation is in TLC */
    real_T *mass = ssGetOutputPortRealSignal(S, OUT_MASS);
    
    /* Zero output for simulation */
    for (int i = 0; i < 49; i++) {
        mass[i] = 0.0;
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
