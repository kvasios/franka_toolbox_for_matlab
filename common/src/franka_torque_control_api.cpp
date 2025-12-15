// Copyright (c) 2025 Franka Robotics GmbH
// Franka Torque Control API - Implementation

#include "franka_torque_control_api.h"

#include <chrono>
#include <iostream>

// ============================================================================
// Constructor / Destructor
// ============================================================================

FrankaTorqueControlContext::FrankaTorqueControlContext() {
    // Initialize torques to zero
    tau_J_d_.fill(0.0);
}

FrankaTorqueControlContext::~FrankaTorqueControlContext() {
    shutdown();
}

// ============================================================================
// Lifecycle Methods
// ============================================================================

void FrankaTorqueControlContext::initialize(const std::string& robot_ip) {
    robot_ip_ = robot_ip;
    
    try {
        // Connect to robot
        robot_ = std::make_unique<franka::Robot>(robot_ip_);
        
        // Attempt automatic error recovery
        try {
            robot_->automaticErrorRecovery();
        } catch (const franka::Exception& e) {
            // Robot might already be in good state, continue
        }
        
        // Load model
        model_ = std::make_unique<franka::Model>(robot_->loadModel());
        
        state_ = FrankaControlState::Idle;
        
    } catch (const franka::Exception& e) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        error_message_ = std::string("Failed to connect to robot: ") + e.what();
        state_ = FrankaControlState::Error;
    }
}

void FrankaTorqueControlContext::shutdown() {
    // Request stop if running
    if (state_ == FrankaControlState::Running) {
        stopControl();
    }
    
    // Wait for control thread to finish
    if (control_thread_.joinable()) {
        // Signal to unblock any waiting
        {
            std::lock_guard<std::mutex> lock(sync_mutex_);
            command_ready_ = true;
        }
        cv_command_ready_.notify_one();
        
        control_thread_.join();
    }
    
    // Release robot connection
    model_.reset();
    robot_.reset();
}

// ============================================================================
// Control Methods
// ============================================================================

void FrankaTorqueControlContext::startControl() {
    if (state_ != FrankaControlState::Idle) {
        return;  // Already running or in error state
    }
    
    if (!robot_) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        error_message_ = "Robot not initialized";
        state_ = FrankaControlState::Error;
        return;
    }
    
    // Reset state
    stop_requested_ = false;
    first_control_step_ = true;
    state_ready_ = false;
    command_ready_ = false;
    tau_J_d_.fill(0.0);
    
    // Clear any previous error
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        error_message_.clear();
    }
    
    state_ = FrankaControlState::Running;
    
    // Spawn control thread
    control_thread_ = std::thread(&FrankaTorqueControlContext::controlThreadFunc, this);
}

void FrankaTorqueControlContext::stopControl() {
    if (state_ != FrankaControlState::Running) {
        return;
    }
    
    state_ = FrankaControlState::Stopping;
    stop_requested_ = true;
    
    // Unblock control thread if waiting for command
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        command_ready_ = true;
    }
    cv_command_ready_.notify_one();
    
    // Wait for control thread to finish
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    
    state_ = FrankaControlState::Idle;
}

bool FrankaTorqueControlContext::isControlRunning() const {
    return state_ == FrankaControlState::Running;
}

bool FrankaTorqueControlContext::hasError() const {
    return state_ == FrankaControlState::Error;
}

std::string FrankaTorqueControlContext::getErrorMessage() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return error_message_;
}

// ============================================================================
// Synchronization Methods
// ============================================================================

void FrankaTorqueControlContext::waitForControlStep() {
    std::unique_lock<std::mutex> lock(sync_mutex_);
    cv_state_ready_.wait(lock, [this] { return state_ready_ || stop_requested_; });
    state_ready_ = false;
}

void FrankaTorqueControlContext::signalControlContinue() {
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        command_ready_ = true;
    }
    cv_command_ready_.notify_one();
}

// ============================================================================
// Data Access Methods
// ============================================================================

void FrankaTorqueControlContext::getJointPositions(double* q) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::copy(robot_state_.q.begin(), robot_state_.q.end(), q);
}

void FrankaTorqueControlContext::getJointVelocities(double* dq) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::copy(robot_state_.dq.begin(), robot_state_.dq.end(), dq);
}

void FrankaTorqueControlContext::setJointTorques(const double* tau_J_d) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    std::copy(tau_J_d, tau_J_d + 7, tau_J_d_.begin());
}

// ============================================================================
// Control Thread Implementation
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
        std::lock_guard<std::mutex> lock(error_mutex_);
        error_message_ = e.what();
        state_ = FrankaControlState::Error;
        
        // Unblock Simulink thread if waiting
        {
            std::lock_guard<std::mutex> sync_lock(sync_mutex_);
            state_ready_ = true;
        }
        cv_state_ready_.notify_one();
    }
    
    // If we exit normally (not error), set to idle
    if (state_ != FrankaControlState::Error) {
        state_ = FrankaControlState::Idle;
    }
}

franka::Torques FrankaTorqueControlContext::controlCallback(
    const franka::RobotState& state, 
    franka::Duration period) {
    
    // Check for stop request
    if (stop_requested_) {
        // Return zero torques and signal motion finished
        return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
    }
    
    // Update robot state (thread-safe)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        robot_state_ = state;
    }
    
    // Skip synchronization on first step (no torques computed yet)
    if (first_control_step_) {
        first_control_step_ = false;
        // Return zero torques for first step
        return franka::Torques({0, 0, 0, 0, 0, 0, 0});
    }
    
    // Signal Simulink that new state is ready
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        state_ready_ = true;
    }
    cv_state_ready_.notify_one();
    
    // Wait for Simulink to compute and set torques
    {
        std::unique_lock<std::mutex> lock(sync_mutex_);
        cv_command_ready_.wait(lock, [this] { return command_ready_ || stop_requested_; });
        command_ready_ = false;
    }
    
    // Check again for stop request
    if (stop_requested_) {
        return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
    }
    
    // Get commanded torques (thread-safe)
    std::array<double, 7> tau_cmd;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        tau_cmd = tau_J_d_;
    }
    
    return franka::Torques(tau_cmd);
}

