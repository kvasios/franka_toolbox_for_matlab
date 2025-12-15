// Copyright (c) 2025 Franka Robotics GmbH
// Franka Torque Control API - Implementation

#include "franka_torque_control_api.h"

#include <iostream>

// ============================================================================
// Constructor / Destructor
// ============================================================================

FrankaTorqueControlContext::FrankaTorqueControlContext() = default;

FrankaTorqueControlContext::~FrankaTorqueControlContext() {
    shutdown();
}

// ============================================================================
// Lifecycle Methods
// ============================================================================

void FrankaTorqueControlContext::initialize(const std::string& robot_ip) {
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

void FrankaTorqueControlContext::setControllerCallback(ControllerCallback callback) {
    controller_callback_ = callback;
}

void FrankaTorqueControlContext::setOutputPointers(double* q_ptr, double* dq_ptr) {
    q_out_ = q_ptr;
    dq_out_ = dq_ptr;
}

void FrankaTorqueControlContext::setInputPointers(const double* tau_J_d_ptr) {
    tau_J_d_in_ = tau_J_d_ptr;
}

void FrankaTorqueControlContext::shutdown() {
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

void FrankaTorqueControlContext::startControl() {
    if (running_) {
        return;
    }
    
    stop_requested_ = false;
    first_step_ = true;
    running_ = true;
    
    control_thread_ = std::thread(&FrankaTorqueControlContext::controlThreadFunc, this);
}

void FrankaTorqueControlContext::requestStop() {
    stop_requested_ = true;
}

bool FrankaTorqueControlContext::isControlRunning() const {
    return running_;
}

// ============================================================================
// Control Thread
// ============================================================================

void FrankaTorqueControlContext::controlThreadFunc() {
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

franka::Torques FrankaTorqueControlContext::controlCallback(
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
    
    // Skip controller on first step
    if (first_step_) {
        first_step_ = false;
        return franka::Torques({0, 0, 0, 0, 0, 0, 0});
    }
    
    // ========================================================================
    // Step 2: Execute the controller (function-call subsystem)
    // ========================================================================
    if (controller_callback_) {
        controller_callback_();
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
