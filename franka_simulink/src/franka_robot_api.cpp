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

void FrankaRobotContext::setModelOutputPointer(FrankaModelDataBus* model_ptr) {
    model_out_ = model_ptr;
}

void FrankaRobotContext::setDtOutputPointer(double* dt_sec_ptr) {
    dt_sec_out_ = dt_sec_ptr;
}

void FrankaRobotContext::setInputPointers(const double* tau_J_d_ptr) {
    tau_J_d_in_ = tau_J_d_ptr;
}

void FrankaRobotContext::setControlMode(FrankaControlMode mode) {
    control_mode_ = mode;
}

void FrankaRobotContext::setTorqueInputPointer(const double* tau_J_d_ptr) {
    tau_J_d_in_ = tau_J_d_ptr;
}

void FrankaRobotContext::setJointPositionInputPointer(const double* q_d_ptr) {
    q_d_in_ = q_d_ptr;
}

void FrankaRobotContext::setJointVelocityInputPointer(const double* dq_d_ptr) {
    dq_d_in_ = dq_d_ptr;
}

void FrankaRobotContext::setCartesianPoseInputPointer(const double* O_T_EE_d_ptr, 
                                                       const double* elbow_d_ptr) {
    O_T_EE_d_in_ = O_T_EE_d_ptr;
    elbow_d_in_ = elbow_d_ptr;
}

void FrankaRobotContext::setCartesianVelocityInputPointer(const double* O_dP_EE_d_ptr,
                                                           const double* elbow_d_ptr) {
    O_dP_EE_d_in_ = O_dP_EE_d_ptr;
    elbow_d_in_ = elbow_d_ptr;
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
        switch (control_mode_) {
            case FrankaControlMode::Torques:
                robot_->control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::Torques {
                        return this->torqueCallback(state, period);
                    },
                    /*limit_rate=*/true,
                    /*cutoff_frequency=*/100.0
                );
                break;
                
            case FrankaControlMode::JointPositions:
                robot_->control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::JointPositions {
                        return this->jointPositionCallback(state, period);
                    },
                    franka::ControllerMode::kJointImpedance,
                    /*limit_rate=*/true,
                    /*cutoff_frequency=*/100.0
                );
                break;
                
            case FrankaControlMode::JointVelocities:
                robot_->control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::JointVelocities {
                        return this->jointVelocityCallback(state, period);
                    },
                    franka::ControllerMode::kJointImpedance,
                    /*limit_rate=*/true,
                    /*cutoff_frequency=*/100.0
                );
                break;
                
            case FrankaControlMode::CartesianPose:
                robot_->control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::CartesianPose {
                        return this->cartesianPoseCallback(state, period);
                    },
                    franka::ControllerMode::kCartesianImpedance,
                    /*limit_rate=*/true,
                    /*cutoff_frequency=*/100.0
                );
                break;
                
            case FrankaControlMode::CartesianVelocities:
                robot_->control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::CartesianVelocities {
                        return this->cartesianVelocityCallback(state, period);
                    },
                    franka::ControllerMode::kCartesianImpedance,
                    /*limit_rate=*/true,
                    /*cutoff_frequency=*/100.0
                );
                break;
        }
    } catch (const franka::Exception& e) {
        std::cerr << "Control exception: " << e.what() << std::endl;
    }
    
    running_ = false;
}

// ============================================================================
// Common Pre-Callback Logic
// ============================================================================

bool FrankaRobotContext::executePreCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    // Check for stop request
    if (stop_requested_) {
        return false;
    }
    
    // ========================================================================
    // Step 1: Copy robot state to Simulink output bus
    // ========================================================================
    if (state_out_) {
        copyRobotState(state, state_out_);
    }
    
    // ========================================================================
    // Step 1b: Compute and copy model data (dynamics/kinematics)
    // ========================================================================
    if (model_out_ && model_) {
        computeModelData(state, model_out_);
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
    
    return true;
}

// Helper to check and sanitize array values (guard against NaN/Inf)
template<size_t N>
static void sanitizeArray(std::array<double, N>& arr, const char* name) {
    bool ok = true;
    for (double v : arr) {
        if (!std::isfinite(v)) {
            ok = false;
            break;
        }
    }
    if (!ok) {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) {
            std::cerr << "FrankaRobotContext: non-finite " << name << " detected; "
                         "zeroing values. Check Simulink signal wiring / controller execution."
                      << std::endl;
        }
        arr.fill(0.0);
    }
}

// ============================================================================
// Mode-Specific Callbacks
// ============================================================================

franka::Torques FrankaRobotContext::torqueCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (!executePreCallback(state, period)) {
        return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
    }
    
    // Read commanded torques from Simulink input
    std::array<double, 7> tau_cmd{};
    if (tau_J_d_in_) {
        std::copy(tau_J_d_in_, tau_J_d_in_ + 7, tau_cmd.begin());
    }
    sanitizeArray(tau_cmd, "tau_J_d");
    
    return franka::Torques(tau_cmd);
}

franka::JointPositions FrankaRobotContext::jointPositionCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (!executePreCallback(state, period)) {
        // Return current position on stop (smooth stop)
        return franka::MotionFinished(franka::JointPositions(state.q_d));
    }
    
    // Read commanded joint positions from Simulink input
    std::array<double, 7> q_cmd{};
    if (q_d_in_) {
        std::copy(q_d_in_, q_d_in_ + 7, q_cmd.begin());
    } else {
        // Default to current commanded position if no input connected
        q_cmd = state.q_d;
    }
    sanitizeArray(q_cmd, "q_d");
    
    return franka::JointPositions(q_cmd);
}

franka::JointVelocities FrankaRobotContext::jointVelocityCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (!executePreCallback(state, period)) {
        // Return zero velocity on stop (smooth deceleration handled by robot)
        return franka::MotionFinished(franka::JointVelocities({0, 0, 0, 0, 0, 0, 0}));
    }
    
    // Read commanded joint velocities from Simulink input
    std::array<double, 7> dq_cmd{};
    if (dq_d_in_) {
        std::copy(dq_d_in_, dq_d_in_ + 7, dq_cmd.begin());
    }
    sanitizeArray(dq_cmd, "dq_d");
    
    return franka::JointVelocities(dq_cmd);
}

franka::CartesianPose FrankaRobotContext::cartesianPoseCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (!executePreCallback(state, period)) {
        // Return current pose on stop (smooth stop)
        return franka::MotionFinished(franka::CartesianPose(state.O_T_EE_d, state.elbow_d));
    }
    
    // Read commanded Cartesian pose from Simulink input (4x4 col-major)
    std::array<double, 16> pose_cmd{};
    if (O_T_EE_d_in_) {
        std::copy(O_T_EE_d_in_, O_T_EE_d_in_ + 16, pose_cmd.begin());
    } else {
        // Default to current commanded pose if no input connected
        pose_cmd = state.O_T_EE_d;
    }
    sanitizeArray(pose_cmd, "O_T_EE_d");
    
    // Read elbow configuration (required for Cartesian control)
    std::array<double, 2> elbow_cmd{};
    if (elbow_d_in_) {
        std::copy(elbow_d_in_, elbow_d_in_ + 2, elbow_cmd.begin());
    } else {
        // Default to current elbow configuration
        elbow_cmd = state.elbow_d;
    }
    sanitizeArray(elbow_cmd, "elbow_d");
    
    return franka::CartesianPose(pose_cmd, elbow_cmd);
}

franka::CartesianVelocities FrankaRobotContext::cartesianVelocityCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (!executePreCallback(state, period)) {
        // Return zero velocity on stop
        return franka::MotionFinished(franka::CartesianVelocities({0, 0, 0, 0, 0, 0}, state.elbow_d));
    }
    
    // Read commanded Cartesian velocity from Simulink input [vx, vy, vz, wx, wy, wz]
    std::array<double, 6> vel_cmd{};
    if (O_dP_EE_d_in_) {
        std::copy(O_dP_EE_d_in_, O_dP_EE_d_in_ + 6, vel_cmd.begin());
    }
    sanitizeArray(vel_cmd, "O_dP_EE_d");
    
    // Read elbow configuration (required for Cartesian control)
    std::array<double, 2> elbow_cmd{};
    if (elbow_d_in_) {
        std::copy(elbow_d_in_, elbow_d_in_ + 2, elbow_cmd.begin());
    } else {
        // Default to current elbow configuration
        elbow_cmd = state.elbow_d;
    }
    sanitizeArray(elbow_cmd, "elbow_d");
    
    return franka::CartesianVelocities(vel_cmd, elbow_cmd);
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

// ============================================================================
// Model Data Computation
// ============================================================================

void FrankaRobotContext::computeModelData(const franka::RobotState& state,
                                           FrankaModelDataBus* dst) {
    // ------------------------------------------------------------------------
    // Dynamics: Mass matrix M(q)
    // ------------------------------------------------------------------------
    {
        auto M = model_->mass(state);
        std::copy(M.begin(), M.end(), &dst->mass[0][0]);
    }
    
    // ------------------------------------------------------------------------
    // Dynamics: Coriolis force vector c(q,dq)
    // ------------------------------------------------------------------------
    {
        auto c = model_->coriolis(state);
        std::copy(c.begin(), c.end(), dst->coriolis);
    }
    
    // ------------------------------------------------------------------------
    // Dynamics: Gravity vector g(q)
    // ------------------------------------------------------------------------
    {
        auto g = model_->gravity(state);
        std::copy(g.begin(), g.end(), dst->gravity);
    }
    
    // ------------------------------------------------------------------------
    // Kinematics: End effector Jacobian in base frame (zero Jacobian)
    // ------------------------------------------------------------------------
    {
        auto J = model_->zeroJacobian(franka::Frame::kEndEffector, state);
        std::copy(J.begin(), J.end(), &dst->jacobian[0][0]);
    }
    
    // ------------------------------------------------------------------------
    // Kinematics: End effector body Jacobian (in EE frame)
    // ------------------------------------------------------------------------
    {
        auto J_body = model_->bodyJacobian(franka::Frame::kEndEffector, state);
        std::copy(J_body.begin(), J_body.end(), &dst->jacobian_body[0][0]);
    }
}
