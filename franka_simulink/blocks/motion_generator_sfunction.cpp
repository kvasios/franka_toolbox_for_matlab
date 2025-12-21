/*
 * motion_generator_sfunction.cpp - Generic Joint Motion Generator S-Function
 *
 * A fully-inlined Level-2 C++ S-Function that generates smooth joint-space
 * trajectories from q_start to q_goal using synchronized polynomial profiles.
 *
 * This block is completely independent of libfranka and works in both:
 *   - Simulink simulation mode (interpreted)
 *   - Code generation (fully inlined via TLC)
 *
 * Algorithm based on:
 *   Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and
 *   Control of Robots (Kogan Page Science Paper edition).
 *
 * Inputs:
 *   0. enable       (1x1)  - Rising edge (0→1) triggers motion start
 *   1. q_start      (7x1)  - Starting joint positions [rad]
 *   2. q_goal       (7x1)  - Goal joint positions [rad]
 *   3. speed_factor (1x1)  - Speed factor in range (0, 1]
 *   4. dt           (1x1)  - Time step for trajectory advancement [s]
 *
 * Outputs:
 *   0. q_d             (7x1)  - Desired joint positions [rad]
 *   1. motion_finished (1x1)  - 1 when motion is complete, 0 otherwise
 *
 * Sample Time: Inherited (uses Simulink's sample time for time advancement)
 *
 * Copyright (c) 2025 Franka Robotics GmbH
 */

#define S_FUNCTION_NAME  motion_generator_sfunction
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include <cmath>
#include <algorithm>
#include <array>

/* Number of joints */
#define NUM_JOINTS 7

/* Input port indices */
#define IN_ENABLE       0
#define IN_Q_START      1
#define IN_Q_GOAL       2
#define IN_SPEED_FACTOR 3
#define IN_DT           4
#define NUM_INPUTS      5

/* Output port indices */
#define OUT_Q_D             0
#define OUT_MOTION_FINISHED 1
#define NUM_OUTPUTS         2

/* DWork indices - store motion generator state */
#define DWORK_PREV_ENABLE    0   /* Previous enable value (edge detection) */
#define DWORK_MOTION_ACTIVE  1   /* Is motion currently active? */
#define DWORK_TIME           2   /* Elapsed time since motion start */
#define DWORK_Q_START        3   /* Captured start positions (7 values) */
#define DWORK_DELTA_Q        4   /* q_goal - q_start (7 values) */
#define DWORK_DQ_MAX_SYNC    5   /* Synchronized max velocities (7 values) */
#define DWORK_T_1_SYNC       6   /* End of acceleration phase times (7 values) */
#define DWORK_T_2_SYNC       7   /* Start of deceleration phase times (7 values) */
#define DWORK_T_F_SYNC       8   /* Final times (7 values) */
#define DWORK_Q_1            9   /* Position at end of acceleration (7 values) */
#define DWORK_SIGN_DELTA_Q   10  /* Sign of delta_q (7 values, stored as double) */
#define NUM_DWORK            11

/* Motion finished threshold */
static const double kDeltaQMotionFinished = 1e-6;

/* Default velocity and acceleration limits (can be scaled by speed_factor) */
static const double kDqMax[NUM_JOINTS] = {2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5};
static const double kDdqMaxStart[NUM_JOINTS] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};
static const double kDdqMaxGoal[NUM_JOINTS] = {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0};

/* ========================================================================
 * Helper: Compute synchronized trajectory parameters
 * ======================================================================== */
static void calculateSynchronizedValues(
    const double* delta_q,
    double speed_factor,
    double* dq_max_sync,
    double* t_1_sync,
    double* t_2_sync,
    double* t_f_sync,
    double* q_1,
    double* sign_delta_q)
{
    /* Apply speed factor to limits */
    double dq_max[NUM_JOINTS], ddq_max_start[NUM_JOINTS], ddq_max_goal[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++) {
        dq_max[i] = kDqMax[i] * speed_factor;
        ddq_max_start[i] = kDdqMaxStart[i] * speed_factor;
        ddq_max_goal[i] = kDdqMaxGoal[i] * speed_factor;
    }

    double dq_max_reach[NUM_JOINTS];
    double t_f[NUM_JOINTS];
    double delta_t_2[NUM_JOINTS];
    double t_1[NUM_JOINTS];

    /* Compute sign of delta_q */
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (delta_q[i] > 0.0) {
            sign_delta_q[i] = 1.0;
        } else if (delta_q[i] < 0.0) {
            sign_delta_q[i] = -1.0;
        } else {
            sign_delta_q[i] = 0.0;
        }
    }

    /* Initialize arrays */
    for (int i = 0; i < NUM_JOINTS; i++) {
        dq_max_reach[i] = dq_max[i];
        t_f[i] = 0.0;
        delta_t_2[i] = 0.0;
        t_1[i] = 0.0;
        dq_max_sync[i] = 0.0;
        t_1_sync[i] = 0.0;
        t_2_sync[i] = 0.0;
        t_f_sync[i] = 0.0;
        q_1[i] = 0.0;
    }

    /* First pass: compute individual joint times */
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (std::abs(delta_q[i]) > kDeltaQMotionFinished) {
            double threshold = (3.0 / 4.0) * (dq_max[i] * dq_max[i] / ddq_max_start[i]) +
                              (3.0 / 4.0) * (dq_max[i] * dq_max[i] / ddq_max_goal[i]);
            
            if (std::abs(delta_q[i]) < threshold) {
                dq_max_reach[i] = std::sqrt(
                    (4.0 / 3.0) * std::abs(delta_q[i]) *
                    (ddq_max_start[i] * ddq_max_goal[i]) /
                    (ddq_max_start[i] + ddq_max_goal[i])
                );
            }
            t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start[i];
            delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal[i];
            t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q[i]) / dq_max_reach[i];
        }
    }

    /* Find maximum time (synchronization point) */
    double max_t_f = 0.0;
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (t_f[i] > max_t_f) {
            max_t_f = t_f[i];
        }
    }

    /* Second pass: synchronize all joints to max_t_f */
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (std::abs(delta_q[i]) > kDeltaQMotionFinished) {
            double a = 1.5 / 2.0 * (ddq_max_goal[i] + ddq_max_start[i]);
            double b = -1.0 * max_t_f * ddq_max_goal[i] * ddq_max_start[i];
            double c = std::abs(delta_q[i]) * ddq_max_goal[i] * ddq_max_start[i];
            double delta = b * b - 4.0 * a * c;
            if (delta < 0.0) {
                delta = 0.0;
            }
            dq_max_sync[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
            t_1_sync[i] = 1.5 * dq_max_sync[i] / ddq_max_start[i];
            double delta_t_2_sync = 1.5 * dq_max_sync[i] / ddq_max_goal[i];
            t_f_sync[i] = t_1_sync[i] / 2.0 + delta_t_2_sync / 2.0 + 
                          std::abs(delta_q[i] / dq_max_sync[i]);
            t_2_sync[i] = t_f_sync[i] - delta_t_2_sync;
            q_1[i] = dq_max_sync[i] * sign_delta_q[i] * (0.5 * t_1_sync[i]);
        }
    }
}

/* ========================================================================
 * Helper: Calculate desired position at time t
 * Returns true if motion is finished
 * ======================================================================== */
static bool calculateDesiredValues(
    double t,
    const double* q_start,
    const double* delta_q,
    const double* dq_max_sync,
    const double* t_1_sync,
    const double* t_2_sync,
    const double* t_f_sync,
    const double* q_1,
    const double* sign_delta_q,
    double* q_d)
{
    bool all_finished = true;

    for (int i = 0; i < NUM_JOINTS; i++) {
        double delta_q_d_i;
        bool joint_finished = false;

        if (std::abs(delta_q[i]) < kDeltaQMotionFinished) {
            delta_q_d_i = 0.0;
            joint_finished = true;
        } else {
            double t_d = t_2_sync[i] - t_1_sync[i];
            double delta_t_2_sync = t_f_sync[i] - t_2_sync[i];

            if (t < t_1_sync[i]) {
                /* Acceleration phase (polynomial) */
                delta_q_d_i = -1.0 / std::pow(t_1_sync[i], 3.0) * dq_max_sync[i] * 
                              sign_delta_q[i] * (0.5 * t - t_1_sync[i]) * std::pow(t, 3.0);
            } else if (t >= t_1_sync[i] && t < t_2_sync[i]) {
                /* Constant velocity phase */
                delta_q_d_i = q_1[i] + (t - t_1_sync[i]) * dq_max_sync[i] * sign_delta_q[i];
            } else if (t >= t_2_sync[i] && t < t_f_sync[i]) {
                /* Deceleration phase (polynomial) */
                delta_q_d_i = delta_q[i] + 0.5 *
                    (1.0 / std::pow(delta_t_2_sync, 3.0) *
                     (t - t_1_sync[i] - 2.0 * delta_t_2_sync - t_d) *
                     std::pow((t - t_1_sync[i] - t_d), 3.0) +
                     (2.0 * t - 2.0 * t_1_sync[i] - delta_t_2_sync - 2.0 * t_d)) *
                    dq_max_sync[i] * sign_delta_q[i];
            } else {
                /* Motion complete */
                delta_q_d_i = delta_q[i];
                joint_finished = true;
            }
        }

        q_d[i] = q_start[i] + delta_q_d_i;
        
        if (!joint_finished) {
            all_finished = false;
        }
    }

    return all_finished;
}

/* ========================================================================
 * mdlInitializeSizes - Initialize block sizes
 * ======================================================================== */
static void mdlInitializeSizes(SimStruct *S)
{
    /* No parameters */
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    /* States */
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    /* Input ports */
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;

    /* Port 0: enable (1x1) */
    ssSetInputPortWidth(S, IN_ENABLE, 1);
    ssSetInputPortDataType(S, IN_ENABLE, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_ENABLE, 1);
    ssSetInputPortRequiredContiguous(S, IN_ENABLE, 1);

    /* Port 1: q_start (7x1) */
    ssSetInputPortWidth(S, IN_Q_START, NUM_JOINTS);
    ssSetInputPortDataType(S, IN_Q_START, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_Q_START, 1);
    ssSetInputPortRequiredContiguous(S, IN_Q_START, 1);

    /* Port 2: q_goal (7x1) */
    ssSetInputPortWidth(S, IN_Q_GOAL, NUM_JOINTS);
    ssSetInputPortDataType(S, IN_Q_GOAL, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_Q_GOAL, 1);
    ssSetInputPortRequiredContiguous(S, IN_Q_GOAL, 1);

    /* Port 3: speed_factor (1x1) */
    ssSetInputPortWidth(S, IN_SPEED_FACTOR, 1);
    ssSetInputPortDataType(S, IN_SPEED_FACTOR, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_SPEED_FACTOR, 1);
    ssSetInputPortRequiredContiguous(S, IN_SPEED_FACTOR, 1);

    /* Port 4: dt (1x1) - Time step for trajectory advancement [s] */
    ssSetInputPortWidth(S, IN_DT, 1);
    ssSetInputPortDataType(S, IN_DT, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, IN_DT, 1);
    ssSetInputPortRequiredContiguous(S, IN_DT, 1);

    /* Output ports */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;

    /* Port 0: q_d (7x1) */
    ssSetOutputPortWidth(S, OUT_Q_D, NUM_JOINTS);
    ssSetOutputPortDataType(S, OUT_Q_D, SS_DOUBLE);

    /* Port 1: motion_finished (1x1) */
    ssSetOutputPortWidth(S, OUT_MOTION_FINISHED, 1);
    ssSetOutputPortDataType(S, OUT_MOTION_FINISHED, SS_DOUBLE);

    /* Sample time */
    ssSetNumSampleTimes(S, 1);

    /* DWork vectors for state storage */
    ssSetNumDWork(S, NUM_DWORK);

    ssSetDWorkWidth(S, DWORK_PREV_ENABLE, 1);
    ssSetDWorkDataType(S, DWORK_PREV_ENABLE, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_PREV_ENABLE, "PrevEnable");

    ssSetDWorkWidth(S, DWORK_MOTION_ACTIVE, 1);
    ssSetDWorkDataType(S, DWORK_MOTION_ACTIVE, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_MOTION_ACTIVE, "MotionActive");

    ssSetDWorkWidth(S, DWORK_TIME, 1);
    ssSetDWorkDataType(S, DWORK_TIME, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_TIME, "Time");

    ssSetDWorkWidth(S, DWORK_Q_START, NUM_JOINTS);
    ssSetDWorkDataType(S, DWORK_Q_START, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_Q_START, "QStart");

    ssSetDWorkWidth(S, DWORK_DELTA_Q, NUM_JOINTS);
    ssSetDWorkDataType(S, DWORK_DELTA_Q, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_DELTA_Q, "DeltaQ");

    ssSetDWorkWidth(S, DWORK_DQ_MAX_SYNC, NUM_JOINTS);
    ssSetDWorkDataType(S, DWORK_DQ_MAX_SYNC, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_DQ_MAX_SYNC, "DqMaxSync");

    ssSetDWorkWidth(S, DWORK_T_1_SYNC, NUM_JOINTS);
    ssSetDWorkDataType(S, DWORK_T_1_SYNC, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_T_1_SYNC, "T1Sync");

    ssSetDWorkWidth(S, DWORK_T_2_SYNC, NUM_JOINTS);
    ssSetDWorkDataType(S, DWORK_T_2_SYNC, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_T_2_SYNC, "T2Sync");

    ssSetDWorkWidth(S, DWORK_T_F_SYNC, NUM_JOINTS);
    ssSetDWorkDataType(S, DWORK_T_F_SYNC, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_T_F_SYNC, "TfSync");

    ssSetDWorkWidth(S, DWORK_Q_1, NUM_JOINTS);
    ssSetDWorkDataType(S, DWORK_Q_1, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_Q_1, "Q1");

    ssSetDWorkWidth(S, DWORK_SIGN_DELTA_Q, NUM_JOINTS);
    ssSetDWorkDataType(S, DWORK_SIGN_DELTA_Q, SS_DOUBLE);
    ssSetDWorkName(S, DWORK_SIGN_DELTA_Q, "SignDeltaQ");

    /* Options */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);
    ssSetOptions(S, SS_OPTION_WORKS_WITH_CODE_REUSE |
                    SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}

/* ========================================================================
 * mdlInitializeSampleTimes - Initialize sample times
 * ======================================================================== */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    /* IMPORTANT:
     * This block is typically used inside Triggered / Function-Call subsystems.
     * Such subsystems require contained blocks to inherit sample time (-1) so
     * they execute only when the subsystem is triggered.
     *
     * We still keep the motion generator's *internal* timebase fixed at 1kHz
     * (see dt in mdlOutputs), making the trajectory invariant to base model
     * sample time while remaining compatible with triggered execution.
     */
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

/* ========================================================================
 * mdlStart - Initialize DWork
 * ======================================================================== */
#define MDL_START
static void mdlStart(SimStruct *S)
{
    real_T *prevEnable = (real_T*)ssGetDWork(S, DWORK_PREV_ENABLE);
    real_T *motionActive = (real_T*)ssGetDWork(S, DWORK_MOTION_ACTIVE);
    real_T *time = (real_T*)ssGetDWork(S, DWORK_TIME);
    real_T *q_start = (real_T*)ssGetDWork(S, DWORK_Q_START);
    real_T *delta_q = (real_T*)ssGetDWork(S, DWORK_DELTA_Q);
    real_T *dq_max_sync = (real_T*)ssGetDWork(S, DWORK_DQ_MAX_SYNC);
    real_T *t_1_sync = (real_T*)ssGetDWork(S, DWORK_T_1_SYNC);
    real_T *t_2_sync = (real_T*)ssGetDWork(S, DWORK_T_2_SYNC);
    real_T *t_f_sync = (real_T*)ssGetDWork(S, DWORK_T_F_SYNC);
    real_T *q_1 = (real_T*)ssGetDWork(S, DWORK_Q_1);
    real_T *sign_delta_q = (real_T*)ssGetDWork(S, DWORK_SIGN_DELTA_Q);

    *prevEnable = 0.0;
    *motionActive = 0.0;
    *time = 0.0;

    for (int i = 0; i < NUM_JOINTS; i++) {
        q_start[i] = 0.0;
        delta_q[i] = 0.0;
        dq_max_sync[i] = 0.0;
        t_1_sync[i] = 0.0;
        t_2_sync[i] = 0.0;
        t_f_sync[i] = 0.0;
        q_1[i] = 0.0;
        sign_delta_q[i] = 0.0;
    }
}

/* ========================================================================
 * mdlOutputs - Compute outputs
 * ======================================================================== */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* Get inputs */
    const real_T *enable_in = ssGetInputPortRealSignal(S, IN_ENABLE);
    const real_T *q_start_in = ssGetInputPortRealSignal(S, IN_Q_START);
    const real_T *q_goal_in = ssGetInputPortRealSignal(S, IN_Q_GOAL);
    const real_T *speed_factor_in = ssGetInputPortRealSignal(S, IN_SPEED_FACTOR);
    const real_T *dt_in = ssGetInputPortRealSignal(S, IN_DT);

    /* Get outputs */
    real_T *q_d_out = ssGetOutputPortRealSignal(S, OUT_Q_D);
    real_T *motion_finished_out = ssGetOutputPortRealSignal(S, OUT_MOTION_FINISHED);

    /* Get DWork */
    real_T *prevEnable = (real_T*)ssGetDWork(S, DWORK_PREV_ENABLE);
    real_T *motionActive = (real_T*)ssGetDWork(S, DWORK_MOTION_ACTIVE);
    real_T *time = (real_T*)ssGetDWork(S, DWORK_TIME);
    real_T *q_start = (real_T*)ssGetDWork(S, DWORK_Q_START);
    real_T *delta_q = (real_T*)ssGetDWork(S, DWORK_DELTA_Q);
    real_T *dq_max_sync = (real_T*)ssGetDWork(S, DWORK_DQ_MAX_SYNC);
    real_T *t_1_sync = (real_T*)ssGetDWork(S, DWORK_T_1_SYNC);
    real_T *t_2_sync = (real_T*)ssGetDWork(S, DWORK_T_2_SYNC);
    real_T *t_f_sync = (real_T*)ssGetDWork(S, DWORK_T_F_SYNC);
    real_T *q_1 = (real_T*)ssGetDWork(S, DWORK_Q_1);
    real_T *sign_delta_q = (real_T*)ssGetDWork(S, DWORK_SIGN_DELTA_Q);

    /* Get time step from input */
    real_T dt = dt_in[0];
    
    /* Edge detection */
    real_T enable = enable_in[0];
    bool rising_edge = (enable > 0.5) && (*prevEnable <= 0.5);
    bool falling_edge = (enable <= 0.5) && (*prevEnable > 0.5);
    *prevEnable = enable;

    /* Handle state transitions */
    if (rising_edge) {
        /* Initialize motion on rising edge */
        *motionActive = 1.0;
        *time = 0.0;

        /* Capture start position and compute delta */
        for (int i = 0; i < NUM_JOINTS; i++) {
            q_start[i] = q_start_in[i];
            delta_q[i] = q_goal_in[i] - q_start_in[i];
        }

        /* Clamp speed factor to valid range */
        double speed_factor = speed_factor_in[0];
        if (speed_factor <= 0.0) speed_factor = 0.01;
        if (speed_factor > 1.0) speed_factor = 1.0;

        /* Calculate synchronized trajectory parameters */
        calculateSynchronizedValues(
            delta_q, speed_factor,
            dq_max_sync, t_1_sync, t_2_sync, t_f_sync, q_1, sign_delta_q);
    }
    else if (falling_edge) {
        /* Stop motion on falling edge (keep current position) */
        *motionActive = 0.0;
    }

    /* Compute output */
    if (*motionActive > 0.5) {
        /* Advance time FIRST (like libfranka examples).
         * This ensures trajectory time matches actual elapsed time.
         * On first callback, dt is typically 0 (no previous callback),
         * so time stays at 0 for the initial output.
         */
        if (dt > 0.0) {
            *time += dt;
        }
        
        /* Compute desired position at current time */
        bool finished = calculateDesiredValues(
            *time, q_start, delta_q,
            dq_max_sync, t_1_sync, t_2_sync, t_f_sync, q_1, sign_delta_q,
            q_d_out);

        *motion_finished_out = finished ? 1.0 : 0.0;
    }
    else {
        /* Not active: pass-through the current q_start input.
         *
         * IMPORTANT for codegen + function-call subsystems:
         * This block may execute before any rising-edge "start" capture occurs.
         * If we output the stored q_start (initialized to zeros), we can produce
         * a one-cycle discontinuity (0 -> actual joints) at the start of control.
         *
         * Keeping the stored q_start synced with the input also ensures that a
         * subsequent rising-edge capture starts from the latest upstream value.
         */
        for (int i = 0; i < NUM_JOINTS; i++) {
            q_start[i] = q_start_in[i];
            q_d_out[i] = q_start_in[i];
        }
        *motion_finished_out = 0.0;
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
