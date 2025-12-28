// Copyright (c) 2025 Franka Robotics GmbH
// Gripper Control - Async Implementation
//
// Provides async command handling for the Franka gripper via a dedicated
// worker thread. Move and Grasp commands are executed asynchronously while
// Stop and ReadState can be called at any time.

#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/gripper.h>
#include <memory>
#include <chrono>
#include <iostream>

// ============================================================================
// Destructor - cleanup worker thread
// ============================================================================

FrankaRobotRPCServiceImpl::~FrankaRobotRPCServiceImpl() {
    stopGripperWorkerThread();
}

// ============================================================================
// Worker Thread Management
// ============================================================================

void FrankaRobotRPCServiceImpl::startGripperWorkerThread() {
    if (gripper_worker_thread_.joinable()) {
        return;  // Already running
    }
    
    gripper_shutdown_requested_.store(false);
    gripper_worker_thread_ = std::thread(&FrankaRobotRPCServiceImpl::gripperWorkerLoop, this);
    KJ_LOG(INFO, "Gripper worker thread started");
}

void FrankaRobotRPCServiceImpl::stopGripperWorkerThread() {
    gripper_shutdown_requested_.store(true);
    
    {
        std::lock_guard<std::mutex> lock(gripper_mutex_);
        gripper_cv_.notify_all();
    }
    
    if (gripper_worker_thread_.joinable()) {
        gripper_worker_thread_.join();
    }
    
    KJ_LOG(INFO, "Gripper worker thread stopped");
}

void FrankaRobotRPCServiceImpl::gripperWorkerLoop() {
    while (!gripper_shutdown_requested_.load()) {
        GripperCommand cmd;
        double width, speed, force, epsilon_inner, epsilon_outer, timeout;
        
        // Wait for command
        {
            std::unique_lock<std::mutex> lock(gripper_mutex_);
            gripper_cv_.wait(lock, [this] {
                return has_pending_gripper_command_ || gripper_shutdown_requested_.load();
            });
            
            if (gripper_shutdown_requested_.load()) {
                break;
            }
            
            if (!has_pending_gripper_command_) {
                continue;
            }
            
            // Copy command parameters
            cmd = pending_gripper_command_;
            width = gripper_cmd_width_;
            speed = gripper_cmd_speed_;
            force = gripper_cmd_force_;
            epsilon_inner = gripper_cmd_epsilon_inner_;
            epsilon_outer = gripper_cmd_epsilon_outer_;
            timeout = gripper_cmd_timeout_;
            has_pending_gripper_command_ = false;
        }
        
        // Clear stop flag before starting new command
        gripper_stop_requested_.store(false);
        
        // Execute command
        gripper_command_status_.store(GripperCommandStatus::BUSY);
        
        bool success = false;
        std::string error_msg;
        
        try {
            auto start_time = std::chrono::steady_clock::now();
            
            // Execute the command with timeout monitoring
            // Note: libfranka gripper commands block until completion or error
            // The timeout is handled by calling stop() from another thread if needed
            
            switch (cmd) {
                case GripperCommand::Move:
                    KJ_LOG(INFO, "Gripper async: Executing move", width, speed);
                    success = gripper_->move(width, speed);
                    break;
                    
                case GripperCommand::Grasp:
                    KJ_LOG(INFO, "Gripper async: Executing grasp", width, speed, force);
                    success = gripper_->grasp(width, speed, force, epsilon_inner, epsilon_outer);
                    break;
                    
                default:
                    error_msg = "Unknown command";
                    break;
            }
            
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double>(end_time - start_time).count();
            
            // Check if we exceeded timeout (command may have completed just in time)
            if (elapsed > timeout && timeout > 0) {
                success = false;
                error_msg = "Command timed out after " + std::to_string(elapsed) + " seconds";
                gripper_command_status_.store(GripperCommandStatus::TIMEOUT);
            } else if (success || cmd == GripperCommand::Grasp) {
                // IMPORTANT SEMANTICS:
                // - For Grasp, libfranka returns true if an object was grasped (is_grasped),
                //   and false if no object was grasped. A "false" does NOT mean the command
                //   failed to execute. The command is considered successful unless it threw
                //   an exception / timed out / was stopped.
                // - For Move, libfranka's boolean reflects execution success.
                if (cmd == GripperCommand::Grasp) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    try {
                        auto gs = gripper_->readOnce();
                        if (!gs.is_grasped) {
                            // Command executed, but no object is currently held.
                            // Keep command_status as SUCCESS and communicate outcome via is_grasped + message.
                            error_msg = "No object grasped (is_grasped=false)";
                            gripper_command_status_.store(GripperCommandStatus::SUCCESS);
                            KJ_LOG(INFO, "Gripper async: Grasp completed; no object grasped");
                        } else {
                            gripper_command_status_.store(GripperCommandStatus::SUCCESS);
                            KJ_LOG(INFO, "Gripper async: Grasp completed; object is held");
                        }
                    } catch (const franka::Exception& e) {
                        // Failed to read state - assume grasp succeeded since gripper.grasp() returned true
                        gripper_command_status_.store(GripperCommandStatus::SUCCESS);
                        KJ_LOG(WARNING, "Gripper async: Could not verify grasp state", e.what());
                    }
                } else {
                    // Move command - just mark as success
                    gripper_command_status_.store(GripperCommandStatus::SUCCESS);
                    KJ_LOG(INFO, "Gripper async: Command succeeded");
                }
            } else {
                gripper_command_status_.store(GripperCommandStatus::FAILED);
                error_msg = "Command returned false";
                KJ_LOG(WARNING, "Gripper async: Command returned false");
            }
            
        } catch (const franka::CommandException& e) {
            // Check if this was caused by intentional stop()
            if (gripper_stop_requested_.load()) {
                error_msg = "Command interrupted by stop";
                KJ_LOG(INFO, "Gripper async: Command stopped by user");
                gripper_command_status_.store(GripperCommandStatus::STOPPED);
            } else {
                error_msg = std::string("CommandException: ") + e.what();
                KJ_LOG(ERROR, "Gripper async: CommandException", e.what());
                gripper_command_status_.store(GripperCommandStatus::FAILED);
            }
        } catch (const franka::NetworkException& e) {
            error_msg = std::string("NetworkException: ") + e.what();
            KJ_LOG(ERROR, "Gripper async: NetworkException", e.what());
            gripper_command_status_.store(GripperCommandStatus::FAILED);
        } catch (const franka::Exception& e) {
            // Check if this was caused by intentional stop()
            if (gripper_stop_requested_.load()) {
                error_msg = "Command interrupted by stop";
                KJ_LOG(INFO, "Gripper async: Command stopped by user");
                gripper_command_status_.store(GripperCommandStatus::STOPPED);
            } else {
                error_msg = std::string("Exception: ") + e.what();
                KJ_LOG(ERROR, "Gripper async: Exception", e.what());
                gripper_command_status_.store(GripperCommandStatus::FAILED);
            }
        }
        
        // Update error message
        {
            std::lock_guard<std::mutex> lock(gripper_status_mutex_);
            gripper_error_message_ = error_msg;
        }
        
        // Notify waiting clients
        gripper_done_cv_.notify_all();
    }
}

// ============================================================================
// Helper to fill GripperAsyncStatus
// ============================================================================

void FrankaRobotRPCServiceImpl::fillGripperAsyncStatus(GripperAsyncStatus::Builder& status) {
    // Get current gripper state
    if (gripper_) {
        try {
            franka::GripperState gs = gripper_->readOnce();
            auto state = status.initState();
            state.setWidth(gs.width);
            state.setMaxWidth(gs.max_width);
            state.setIsGrasped(gs.is_grasped);
            state.setTemperature(gs.temperature);
            state.setTimeStamp(gs.time.toSec());
        } catch (const franka::Exception& e) {
            KJ_LOG(WARNING, "Failed to read gripper state", e.what());
        }
    }
    
    // Set command status
    status.setCommandStatus(gripper_command_status_.load());
    
    // Set string fields (protected by mutex)
    {
        std::lock_guard<std::mutex> lock(gripper_status_mutex_);
        status.setLastCommand(gripper_last_command_name_);
        status.setErrorMessage(gripper_error_message_);
    }
}

// ============================================================================
// Synchronous Gripper Methods (unchanged behavior)
// ============================================================================

kj::Promise<void> FrankaRobotRPCServiceImpl::getGripperState(
    capnp::CallContext<GetGripperStateParams, GetGripperStateResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    try {
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }

        franka::GripperState state = gripper_->readOnce();

        auto results = context.getResults();
        auto gripper_state = results.initState();
        
        gripper_state.setWidth(state.width);
        gripper_state.setMaxWidth(state.max_width);
        gripper_state.setIsGrasped(state.is_grasped);
        gripper_state.setTemperature(state.temperature);
        gripper_state.setTimeStamp(state.time.toSec());

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to get gripper state", e.what());
        throw;
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::gripperGrasp(
    capnp::CallContext<GripperGraspParams, GripperGraspResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    double width = params.getWidth();
    double speed = params.getSpeed();
    double force = params.getForce();
    double epsilon_inner = params.getEpsilonInner();
    double epsilon_outer = params.getEpsilonOuter();

    try {
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }

        // Keep async status consistent even for synchronous calls.
        // MATLAB's Gripper.status() queries getGripperAsyncStatus(), so if the user
        // uses synchronous commands (default), we must still update these fields.
        {
            std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
            gripper_last_command_name_ = "grasp";
            gripper_error_message_.clear();
        }

        // Prevent synchronous gripper commands from racing with the async worker.
        {
            std::lock_guard<std::mutex> lock(gripper_mutex_);
            if (has_pending_gripper_command_ ||
                gripper_command_status_.load() == GripperCommandStatus::BUSY) {
                gripper_command_status_.store(GripperCommandStatus::FAILED);
                {
                    std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
                    gripper_error_message_ = "Command rejected: async gripper command in progress";
                }
                auto results = context.getResults();
                results.setSuccess(false);
                return kj::READY_NOW;
            }
        }

        gripper_command_status_.store(GripperCommandStatus::BUSY);
        bool success = gripper_->grasp(width, speed, force, epsilon_inner, epsilon_outer);
        
        auto results = context.getResults();
        results.setSuccess(success);

        // Semantics: for grasp(), a "false" means no object was grasped, not that the command failed.
        // Reserve FAILED for exceptions / stop / timeout / transport issues.
        gripper_command_status_.store(GripperCommandStatus::SUCCESS);
        if (!success) {
            std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
            gripper_error_message_ = "No object grasped (is_grasped=false)";
        }

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Gripper grasp failed", e.what());
        gripper_command_status_.store(GripperCommandStatus::FAILED);
        {
            std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
            gripper_last_command_name_ = "grasp";
            gripper_error_message_ = e.what();
        }
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::gripperHoming(
    capnp::CallContext<GripperHomingParams, GripperHomingResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    try {
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }

        {
            std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
            gripper_last_command_name_ = "homing";
            gripper_error_message_.clear();
        }

        {
            std::lock_guard<std::mutex> lock(gripper_mutex_);
            if (has_pending_gripper_command_ ||
                gripper_command_status_.load() == GripperCommandStatus::BUSY) {
                gripper_command_status_.store(GripperCommandStatus::FAILED);
                {
                    std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
                    gripper_error_message_ = "Command rejected: async gripper command in progress";
                }
                auto results = context.getResults();
                results.setSuccess(false);
                return kj::READY_NOW;
            }
        }

        gripper_command_status_.store(GripperCommandStatus::BUSY);
        bool success = gripper_->homing();
        
        auto results = context.getResults();
        results.setSuccess(success);

        if (success) {
            gripper_command_status_.store(GripperCommandStatus::SUCCESS);
        } else {
            gripper_command_status_.store(GripperCommandStatus::FAILED);
            std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
            gripper_error_message_ = "Command returned false";
        }

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Gripper homing failed", e.what());
        gripper_command_status_.store(GripperCommandStatus::FAILED);
        {
            std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
            gripper_last_command_name_ = "homing";
            gripper_error_message_ = e.what();
        }
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::gripperMove(
    capnp::CallContext<GripperMoveParams, GripperMoveResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    double width = params.getWidth();
    double speed = params.getSpeed();

    try {
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }

        {
            std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
            gripper_last_command_name_ = "move";
            gripper_error_message_.clear();
        }

        {
            std::lock_guard<std::mutex> lock(gripper_mutex_);
            if (has_pending_gripper_command_ ||
                gripper_command_status_.load() == GripperCommandStatus::BUSY) {
                gripper_command_status_.store(GripperCommandStatus::FAILED);
                {
                    std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
                    gripper_error_message_ = "Command rejected: async gripper command in progress";
                }
                auto results = context.getResults();
                results.setSuccess(false);
                return kj::READY_NOW;
            }
        }

        gripper_command_status_.store(GripperCommandStatus::BUSY);
        bool success = gripper_->move(width, speed);
        
        auto results = context.getResults();
        results.setSuccess(success);

        if (success) {
            gripper_command_status_.store(GripperCommandStatus::SUCCESS);
        } else {
            gripper_command_status_.store(GripperCommandStatus::FAILED);
            std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
            gripper_error_message_ = "Command returned false";
        }

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Gripper move failed", e.what());
        gripper_command_status_.store(GripperCommandStatus::FAILED);
        {
            std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
            gripper_last_command_name_ = "move";
            gripper_error_message_ = e.what();
        }
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::gripperStop(
    capnp::CallContext<GripperStopParams, GripperStopResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    try {
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }

        {
            std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
            gripper_last_command_name_ = "stop";
            gripper_error_message_.clear();
        }

        // Mark that stop was requested - worker thread will check this
        // when it catches the "Command aborted" exception
        gripper_stop_requested_.store(true);

        // CRITICAL: Clear any pending commands from the queue AND in-progress commands
        // This prevents the next queued command (e.g., grasp after move) from executing
        // and ensures status is properly updated when a command is interrupted
        bool had_pending = false;
        {
            std::lock_guard<std::mutex> lock(gripper_mutex_);
            
            // Clear pending queue if command was waiting
            if (has_pending_gripper_command_) {
                KJ_LOG(INFO, "Gripper stop: Flushing pending command from queue");
                has_pending_gripper_command_ = false;
                pending_gripper_command_ = GripperCommand::None;
                had_pending = true;
                
                // Mark status as stopped for the cancelled pending command
                gripper_command_status_.store(GripperCommandStatus::STOPPED);
                {
                    std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
                    gripper_error_message_ = "Pending command cancelled by stop";
                }
            }
        }

        // stop() is thread-safe and will interrupt any running move/grasp command
        // This is the key feature: stop can be called while move/grasp is in progress
        bool success = gripper_->stop();
        
        // CRITICAL: After stop(), the gripper may be in an error state where subsequent
        // commands throw "Command aborted" exceptions. We need to clear this by reading
        // the gripper state (which implicitly acknowledges the stop).
        try {
            gripper_->readOnce();
            KJ_LOG(INFO, "Gripper stop: Cleared gripper state after stop");
        } catch (const franka::Exception& e) {
            // Ignore errors during readOnce - we're just clearing state
            KJ_LOG(WARNING, "Gripper stop: readOnce after stop failed (non-fatal)", e.what());
        }
        
        // Wait for the worker thread to actually stop and update status
        // This prevents race conditions where a new command is queued immediately
        // after stop() but before the worker thread has updated the status
        if (!had_pending && gripper_command_status_.load() == GripperCommandStatus::BUSY) {
            KJ_LOG(INFO, "Gripper stop: Waiting for worker thread to acknowledge stop");
            
            // Wait up to 500ms for worker to update status
            std::unique_lock<std::mutex> lock(gripper_mutex_);
            gripper_done_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
                return gripper_command_status_.load() != GripperCommandStatus::BUSY;
            });
            
            // Update status if worker hasn't done it yet (shouldn't happen, but defensive)
            if (gripper_command_status_.load() == GripperCommandStatus::BUSY) {
                KJ_LOG(WARNING, "Gripper stop: Worker didn't update status in time, forcing STOPPED");
                gripper_command_status_.store(GripperCommandStatus::STOPPED);
                {
                    std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
                    gripper_error_message_ = "Command interrupted by stop";
                }
            }
        }
        
        auto results = context.getResults();
        results.setSuccess(success);
        
        KJ_LOG(INFO, "Gripper stop executed", success);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Gripper stop failed", e.what());
        gripper_command_status_.store(GripperCommandStatus::FAILED);
        {
            std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
            gripper_last_command_name_ = "stop";
            gripper_error_message_ = e.what();
        }
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

// ============================================================================
// Asynchronous Gripper Methods
// ============================================================================

kj::Promise<void> FrankaRobotRPCServiceImpl::gripperMoveAsync(
    capnp::CallContext<GripperMoveAsyncParams, GripperMoveAsyncResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    double width = params.getWidth();
    double speed = params.getSpeed();
    double timeout = params.getTimeout();
    
    // Default timeout of 15 seconds if not specified or invalid
    if (timeout <= 0) {
        timeout = 15.0;
    }

    try {
        // Initialize gripper if needed
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }
        
        // Ensure worker thread is running
        startGripperWorkerThread();
        
        // Check and queue atomically under mutex to prevent race conditions
        {
            std::lock_guard<std::mutex> lock(gripper_mutex_);
            
            // Check if a command is already in progress or pending
            // Only block if status is BUSY (actively executing) or command is queued
            // Allow queueing new commands after previous command finished (SUCCESS, FAILED, TIMEOUT, STOPPED)
            if (has_pending_gripper_command_ || 
                gripper_command_status_.load() == GripperCommandStatus::BUSY) {
                KJ_LOG(WARNING, "Gripper async: Command already in progress or pending");
                auto results = context.getResults();
                results.setStarted(false);
                return kj::READY_NOW;
            }
            
            // Queue the command
            pending_gripper_command_ = GripperCommand::Move;
            gripper_cmd_width_ = width;
            gripper_cmd_speed_ = speed;
            gripper_cmd_timeout_ = timeout;
            has_pending_gripper_command_ = true;
            
            // Mark as BUSY immediately when queueing. This prevents wait() from returning
            // early while the command is still pending (before the worker thread picks it up).
            gripper_command_status_.store(GripperCommandStatus::BUSY);
            
            {
                std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
                gripper_last_command_name_ = "move";
                gripper_error_message_.clear();
            }
        }
        
        // Notify worker thread
        gripper_cv_.notify_one();
        
        KJ_LOG(INFO, "Gripper async: Move command queued", width, speed, timeout);
        
        auto results = context.getResults();
        results.setStarted(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Gripper async move failed to start", e.what());
        auto results = context.getResults();
        results.setStarted(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::gripperGraspAsync(
    capnp::CallContext<GripperGraspAsyncParams, GripperGraspAsyncResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    double width = params.getWidth();
    double speed = params.getSpeed();
    double force = params.getForce();
    double epsilon_inner = params.getEpsilonInner();
    double epsilon_outer = params.getEpsilonOuter();
    double timeout = params.getTimeout();
    
    // Default timeout of 15 seconds if not specified or invalid
    if (timeout <= 0) {
        timeout = 15.0;
    }

    try {
        // Initialize gripper if needed
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }
        
        // Ensure worker thread is running
        startGripperWorkerThread();
        
        // Check and queue atomically under mutex to prevent race conditions
        {
            std::lock_guard<std::mutex> lock(gripper_mutex_);
            
            // Check if a command is already in progress or pending
            // Only block if status is BUSY (actively executing) or command is queued
            // Allow queueing new commands after previous command finished (SUCCESS, FAILED, TIMEOUT, STOPPED)
            auto current_status = gripper_command_status_.load();
            KJ_LOG(INFO, "Gripper async grasp: Checking queue state",
                   has_pending_gripper_command_, (int)current_status);
            
            if (has_pending_gripper_command_ || current_status == GripperCommandStatus::BUSY) {
                KJ_LOG(WARNING, "Gripper async: Command rejected - already in progress or pending",
                       has_pending_gripper_command_, (int)current_status);
                auto results = context.getResults();
                results.setStarted(false);
                return kj::READY_NOW;
            }
            
            // Queue the command
            KJ_LOG(INFO, "Gripper async: Queueing grasp command NOW");
            pending_gripper_command_ = GripperCommand::Grasp;
            gripper_cmd_width_ = width;
            gripper_cmd_speed_ = speed;
            gripper_cmd_force_ = force;
            gripper_cmd_epsilon_inner_ = epsilon_inner;
            gripper_cmd_epsilon_outer_ = epsilon_outer;
            gripper_cmd_timeout_ = timeout;
            has_pending_gripper_command_ = true;
            
            // Mark as BUSY immediately when queueing. This prevents wait() from returning
            // early while the command is still pending (before the worker thread picks it up).
            gripper_command_status_.store(GripperCommandStatus::BUSY);
            
            {
                std::lock_guard<std::mutex> status_lock(gripper_status_mutex_);
                gripper_last_command_name_ = "grasp";
                gripper_error_message_.clear();
                KJ_LOG(INFO, "Gripper async: Updated last_command to 'grasp'");
            }
        }
        
        // Notify worker thread
        gripper_cv_.notify_one();
        
        KJ_LOG(INFO, "Gripper async: Grasp command queued", width, speed, force, timeout);
        
        auto results = context.getResults();
        results.setStarted(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Gripper async grasp failed to start", e.what());
        auto results = context.getResults();
        results.setStarted(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::getGripperAsyncStatus(
    capnp::CallContext<GetGripperAsyncStatusParams, GetGripperAsyncStatusResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    try {
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }

        auto results = context.getResults();
        auto status = results.initStatus();
        fillGripperAsyncStatus(status);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to get gripper async status", e.what());
        throw;
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::gripperWaitForCommand(
    capnp::CallContext<GripperWaitForCommandParams, GripperWaitForCommandResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    double timeout = params.getTimeout();
    
    // Default timeout if not specified
    if (timeout <= 0) {
        timeout = 30.0;  // 30 second default wait timeout
    }

    try {
        if (!gripper_) {
            gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
        }

        // Wait for command completion or timeout
        {
            std::unique_lock<std::mutex> lock(gripper_mutex_);
            auto deadline = std::chrono::steady_clock::now() + 
                           std::chrono::duration<double>(timeout);
            
            bool completed = gripper_done_cv_.wait_until(lock, deadline, [this] {
                auto status = gripper_command_status_.load();
                // Consider both queued (has_pending_gripper_command_) and executing (BUSY).
                return !has_pending_gripper_command_ && status != GripperCommandStatus::BUSY;
            });
            
            if (!completed) {
                KJ_LOG(WARNING, "Gripper wait: Timed out waiting for command");
            }
        }

        auto results = context.getResults();
        auto status = results.initStatus();
        fillGripperAsyncStatus(status);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed during gripper wait", e.what());
        throw;
    }

    return kj::READY_NOW;
}
