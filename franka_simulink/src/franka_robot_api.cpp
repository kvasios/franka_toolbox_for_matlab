// Copyright (c) 2025 Franka Robotics GmbH
// Franka Robot API - Implementation

#include "franka_robot_api.h"

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

void FrankaRobotContext::setOutputPointers(double* q_ptr, double* dq_ptr, double* dt_sec_ptr) {
    q_out_ = q_ptr;
    dq_out_ = dq_ptr;
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
    // Step 1: Copy robot state to Simulink outputs
    // ========================================================================
    if (q_out_) {
        std::copy(state.q.begin(), state.q.end(), q_out_);
    }
    if (dq_out_) {
        std::copy(state.dq.begin(), state.dq.end(), dq_out_);
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
    
    return franka::Torques(tau_cmd);
}
