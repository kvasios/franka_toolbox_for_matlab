// Copyright (c) 2025 Franka Robotics GmbH
// Franka Robot API - Implementation

#include "franka_robot_api.h"

#include <cmath>
#include <iostream>

// ============================================================================
// Constructor / Destructor
// ============================================================================

FrankaRobotContext::FrankaRobotContext() = default;

FrankaRobotContext::~FrankaRobotContext() {
    shutdown();
}

// ============================================================================
// Lifecycle Methods
// ============================================================================

void FrankaRobotContext::initialize(const std::string& robot_ip) {
    robot_ip_ = robot_ip;
    
    try {
        robot_ = std::make_unique<franka::Robot>(robot_ip_);
        
        // Attempt automatic error recovery
        try {
            robot_->automaticErrorRecovery();
        } catch (const franka::Exception&) {
            // Robot might already be in good state
        }
        
        model_ = std::make_unique<franka::Model>(robot_->loadModel());
        
    } catch (const franka::Exception& e) {
        std::cerr << "Failed to connect to robot: " << e.what() << std::endl;
        throw;
    }
}

void FrankaRobotContext::setControllerCallback(ControllerCallback callback, void* user_data) {
    controller_callback_ = callback;
    controller_user_data_ = user_data;
}

void FrankaRobotContext::setStateOutputPointer(FrankaRobotStateBus* state_ptr) {
    state_out_ = state_ptr;
}

void FrankaRobotContext::setDtOutputPointer(double* dt_sec_ptr) {
    dt_sec_out_ = dt_sec_ptr;
}

void FrankaRobotContext::setInputPointers(const double* tau_J_d_ptr) {
    tau_J_d_in_ = tau_J_d_ptr;
}

void FrankaRobotContext::shutdown() {
    if (running_) {
        requestStop();
    }

    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    model_.reset();
    robot_.reset();
}

// ============================================================================
// Control Methods
// ============================================================================

void FrankaRobotContext::startControl() {
    if (running_) {
        return;
    }
    
    stop_requested_ = false;
    running_ = true;

    control_thread_ = std::thread(&FrankaRobotContext::controlThreadFunc, this);
}

void FrankaRobotContext::requestStop() {
    stop_requested_ = true;
}

bool FrankaRobotContext::isControlRunning() const {
    return running_;
}

// ============================================================================
// Control Thread
// ============================================================================

void FrankaRobotContext::controlThreadFunc() {
    try {
        robot_->control(
            [this](const franka::RobotState& state, franka::Duration period) 
                -> franka::Torques {
                return this->controlCallback(state, period);
            },
            /*limit_rate=*/true,
            /*cutoff_frequency=*/100.0
        );
    } catch (const franka::Exception& e) {
        std::cerr << "Control exception: " << e.what() << std::endl;
    }
    
    running_ = false;
}

franka::Torques FrankaRobotContext::controlCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    // Check for stop request
    if (stop_requested_) {
        return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
    }
    
    // ========================================================================
    // Step 1: Copy robot state to Simulink output bus
    // ========================================================================
    if (state_out_) {
        copyRobotState(state, state_out_);
    }

    // ========================================================================
    // Step 2: Compute dt_sec (franka::Duration) and execute controller
    // ========================================================================
    // dt_sec is time since previous callback:
    //   - First callback: period=0 (no previous callback)
    //   - Subsequent callbacks: typically ~0.001s, can be ~0.002s on jitter
    //
    // Following libfranka convention, we pass dt_sec to the controller callback
    // which advances taskTime0 BEFORE running the controller. On first callback,
    // dt_sec=0 so taskTime0 stays at 0, matching libfranka examples.
    const double dt_sec = period.toSec();
    if (dt_sec_out_) {
        *dt_sec_out_ = dt_sec;
    }
    
    if (controller_callback_) {
        controller_callback_(controller_user_data_, dt_sec);
    }
    
    // ========================================================================
    // Step 3: Read computed torques from Simulink input
    // ========================================================================
    std::array<double, 7> tau_cmd{};
    if (tau_J_d_in_) {
        std::copy(tau_J_d_in_, tau_J_d_in_ + 7, tau_cmd.begin());
    }

    // Safety: guard against NaN/Inf torques (often indicates wiring/state issues)
    // to avoid sending garbage to the robot.
    {
        bool ok = true;
        for (double v : tau_cmd) {
            if (!std::isfinite(v)) {
                ok = false;
                break;
            }
        }
        if (!ok) {
            static std::atomic<bool> warned{false};
            if (!warned.exchange(true)) {
                std::cerr << "FrankaRobotContext: non-finite tau_J_d detected; "
                             "zeroing torques. Check Simulink signal wiring / controller execution."
                          << std::endl;
            }
            tau_cmd.fill(0.0);
        }
    }
    
    return franka::Torques(tau_cmd);
}

// ============================================================================
// State Copying
// ============================================================================

void FrankaRobotContext::copyRobotState(const franka::RobotState& src, 
                                         FrankaRobotStateBus* dst) {
    // Helper macro to copy std::array to C array (1D)
    #define COPY_ARRAY(field) \
        std::copy(src.field.begin(), src.field.end(), dst->field)
    
    // Helper macro to copy std::array to 2D C array (contiguous memory)
    // Both libfranka and Simulink use column-major order, so direct copy works
    #define COPY_MATRIX_4x4(field) \
        std::copy(src.field.begin(), src.field.end(), &dst->field[0][0])
    
    #define COPY_MATRIX_3x3(field) \
        std::copy(src.field.begin(), src.field.end(), &dst->field[0][0])
    
    // ------------------------------------------------------------------------
    // Transformation Matrices (4x4)
    // ------------------------------------------------------------------------
    COPY_MATRIX_4x4(O_T_EE);
    COPY_MATRIX_4x4(O_T_EE_d);
    COPY_MATRIX_4x4(F_T_EE);
    COPY_MATRIX_4x4(F_T_NE);
    COPY_MATRIX_4x4(NE_T_EE);
    COPY_MATRIX_4x4(EE_T_K);
    COPY_MATRIX_4x4(O_T_EE_c);
    
    // ------------------------------------------------------------------------
    // End Effector Inertial Parameters
    // ------------------------------------------------------------------------
    dst->m_ee = src.m_ee;
    COPY_MATRIX_3x3(I_ee);
    COPY_ARRAY(F_x_Cee);
    
    // ------------------------------------------------------------------------
    // External Load Inertial Parameters
    // ------------------------------------------------------------------------
    dst->m_load = src.m_load;
    COPY_MATRIX_3x3(I_load);
    COPY_ARRAY(F_x_Cload);
    
    // ------------------------------------------------------------------------
    // Total Inertial Parameters
    // ------------------------------------------------------------------------
    dst->m_total = src.m_total;
    COPY_MATRIX_3x3(I_total);
    COPY_ARRAY(F_x_Ctotal);
    
    // ------------------------------------------------------------------------
    // Elbow Configuration
    // ------------------------------------------------------------------------
    COPY_ARRAY(elbow);
    COPY_ARRAY(elbow_d);
    COPY_ARRAY(elbow_c);
    COPY_ARRAY(delbow_c);
    COPY_ARRAY(ddelbow_c);
    
    // ------------------------------------------------------------------------
    // Joint-Space Signals
    // ------------------------------------------------------------------------
    COPY_ARRAY(tau_J);
    COPY_ARRAY(tau_J_d);
    COPY_ARRAY(dtau_J);
    COPY_ARRAY(q);
    COPY_ARRAY(q_d);
    COPY_ARRAY(dq);
    COPY_ARRAY(dq_d);
    COPY_ARRAY(ddq_d);
    COPY_ARRAY(theta);
    COPY_ARRAY(dtheta);
    
    // ------------------------------------------------------------------------
    // Contact and Collision Detection
    // ------------------------------------------------------------------------
    COPY_ARRAY(joint_contact);
    COPY_ARRAY(cartesian_contact);
    COPY_ARRAY(joint_collision);
    COPY_ARRAY(cartesian_collision);
    
    // ------------------------------------------------------------------------
    // External Force Estimates
    // ------------------------------------------------------------------------
    COPY_ARRAY(tau_ext_hat_filtered);
    COPY_ARRAY(O_F_ext_hat_K);
    COPY_ARRAY(K_F_ext_hat_K);
    
    // ------------------------------------------------------------------------
    // Cartesian Motion Signals
    // ------------------------------------------------------------------------
    COPY_ARRAY(O_dP_EE_d);
    COPY_ARRAY(O_ddP_O);
    COPY_ARRAY(O_dP_EE_c);
    COPY_ARRAY(O_ddP_EE_c);
    
    // ------------------------------------------------------------------------
    // Status Signals
    // ------------------------------------------------------------------------
    dst->control_command_success_rate = src.control_command_success_rate;
    dst->robot_mode = static_cast<int32_t>(src.robot_mode);
    dst->time = src.time.toSec();
    
    #undef COPY_ARRAY
    #undef COPY_MATRIX_4x4
    #undef COPY_MATRIX_3x3
}
