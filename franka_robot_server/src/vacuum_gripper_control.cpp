// Copyright (c) 2025 Franka Robotics GmbH
// Vacuum Gripper Control - Async Implementation
//
// Provides async command handling for the Franka vacuum gripper via a dedicated
// worker thread. Vacuum and DropOff commands are executed asynchronously while
// Stop and ReadState can be called at any time.

#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/vacuum_gripper.h>
#include <memory>
#include <chrono>
#include <iostream>

// ============================================================================
// Worker Thread Management
// ============================================================================

void FrankaRobotRPCServiceImpl::startVacuumGripperWorkerThread() {
    if (vacuum_gripper_worker_thread_.joinable()) {
        return;  // Already running
    }
    
    vacuum_gripper_shutdown_requested_.store(false);
    vacuum_gripper_worker_thread_ = std::thread(&FrankaRobotRPCServiceImpl::vacuumGripperWorkerLoop, this);
    KJ_LOG(INFO, "Vacuum gripper worker thread started");
}

void FrankaRobotRPCServiceImpl::stopVacuumGripperWorkerThread() {
    vacuum_gripper_shutdown_requested_.store(true);
    
    {
        std::lock_guard<std::mutex> lock(vacuum_gripper_mutex_);
        vacuum_gripper_cv_.notify_all();
    }
    
    if (vacuum_gripper_worker_thread_.joinable()) {
        vacuum_gripper_worker_thread_.join();
    }
    
    KJ_LOG(INFO, "Vacuum gripper worker thread stopped");
}

void FrankaRobotRPCServiceImpl::vacuumGripperWorkerLoop() {
    while (!vacuum_gripper_shutdown_requested_.load()) {
        VacuumGripperCommand cmd;
        uint8_t control_point;
        uint32_t timeout;
        uint8_t profile;
        double command_timeout;
        
        // Wait for command
        {
            std::unique_lock<std::mutex> lock(vacuum_gripper_mutex_);
            vacuum_gripper_cv_.wait(lock, [this] {
                return has_pending_vacuum_gripper_command_ || vacuum_gripper_shutdown_requested_.load();
            });
            
            if (vacuum_gripper_shutdown_requested_.load()) {
                break;
            }
            
            if (!has_pending_vacuum_gripper_command_) {
                continue;
            }
            
            // Copy command parameters
            cmd = pending_vacuum_gripper_command_;
            control_point = vacuum_gripper_cmd_control_point_;
            timeout = vacuum_gripper_cmd_timeout_;
            profile = vacuum_gripper_cmd_profile_;
            command_timeout = vacuum_gripper_cmd_command_timeout_;
            has_pending_vacuum_gripper_command_ = false;
        }
        
        // Clear stop flag before starting new command
        vacuum_gripper_stop_requested_.store(false);
        
        // Execute command
        vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::BUSY);
        
        bool success = false;
        std::string error_msg;
        
        try {
            auto start_time = std::chrono::steady_clock::now();
            
            // Execute the command with timeout monitoring
            switch (cmd) {
                case VacuumGripperCommand::Vacuum:
                    KJ_LOG(INFO, "Vacuum gripper async: Executing vacuum", control_point, timeout, profile);
                    success = vacuum_gripper_->vacuum(
                        control_point,
                        std::chrono::milliseconds(timeout),
                        static_cast<franka::VacuumGripper::ProductionSetupProfile>(profile)
                    );
                    break;
                    
                case VacuumGripperCommand::DropOff:
                    KJ_LOG(INFO, "Vacuum gripper async: Executing drop off", timeout);
                    success = vacuum_gripper_->dropOff(std::chrono::milliseconds(timeout));
                    break;
                    
                default:
                    error_msg = "Unknown command";
                    break;
            }
            
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double>(end_time - start_time).count();
            
            // Check if we exceeded timeout (command may have completed just in time)
            if (elapsed > command_timeout && command_timeout > 0) {
                success = false;
                error_msg = "Command timed out after " + std::to_string(elapsed) + " seconds";
                vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::TIMEOUT);
            } else if (success) {
                vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::SUCCESS);
                KJ_LOG(INFO, "Vacuum gripper async: Command succeeded");
            } else {
                vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::FAILED);
                error_msg = "Command returned false";
                KJ_LOG(WARNING, "Vacuum gripper async: Command returned false");
            }
            
        } catch (const franka::CommandException& e) {
            // Check if this was caused by intentional stop()
            if (vacuum_gripper_stop_requested_.load()) {
                error_msg = "Command interrupted by stop";
                KJ_LOG(INFO, "Vacuum gripper async: Command stopped by user");
                vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::STOPPED);
            } else {
                error_msg = std::string("CommandException: ") + e.what();
                KJ_LOG(ERROR, "Vacuum gripper async: CommandException", e.what());
                vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::FAILED);
            }
        } catch (const franka::NetworkException& e) {
            error_msg = std::string("NetworkException: ") + e.what();
            KJ_LOG(ERROR, "Vacuum gripper async: NetworkException", e.what());
            vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::FAILED);
        } catch (const franka::Exception& e) {
            // Check if this was caused by intentional stop()
            if (vacuum_gripper_stop_requested_.load()) {
                error_msg = "Command interrupted by stop";
                KJ_LOG(INFO, "Vacuum gripper async: Command stopped by user");
                vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::STOPPED);
            } else {
                error_msg = std::string("Exception: ") + e.what();
                KJ_LOG(ERROR, "Vacuum gripper async: Exception", e.what());
                vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::FAILED);
            }
        }
        
        // Update error message
        {
            std::lock_guard<std::mutex> lock(vacuum_gripper_status_mutex_);
            vacuum_gripper_error_message_ = error_msg;
        }
        
        // Notify waiting clients
        vacuum_gripper_done_cv_.notify_all();
    }
}

// ============================================================================
// Helper to fill VacuumGripperAsyncStatus
// ============================================================================

void FrankaRobotRPCServiceImpl::fillVacuumGripperAsyncStatus(VacuumGripperAsyncStatus::Builder& status) {
    // Get current vacuum gripper state
    if (vacuum_gripper_) {
        try {
            franka::VacuumGripperState gs = vacuum_gripper_->readOnce();
            auto state = status.initState();
            state.setInControlRange(gs.in_control_range);
            state.setPartDetached(gs.part_detached);
            state.setPartPresent(gs.part_present);
            state.setDeviceStatus(static_cast<uint8_t>(gs.device_status));
            state.setActualPower(static_cast<double>(gs.actual_power));
            state.setVacuum(static_cast<double>(gs.vacuum));
            state.setTime(static_cast<double>(gs.time.toMSec()));
        } catch (const franka::Exception& e) {
            KJ_LOG(WARNING, "Failed to read vacuum gripper state", e.what());
        }
    }
    
    // Set command status
    status.setCommandStatus(vacuum_gripper_command_status_.load());
    
    // Set string fields (protected by mutex)
    {
        std::lock_guard<std::mutex> lock(vacuum_gripper_status_mutex_);
        status.setLastCommand(vacuum_gripper_last_command_name_);
        status.setErrorMessage(vacuum_gripper_error_message_);
    }
}

// ============================================================================
// Synchronous Vacuum Gripper Methods
// ============================================================================

kj::Promise<void> FrankaRobotRPCServiceImpl::getVacuumGripperState(
    capnp::CallContext<GetVacuumGripperStateParams, GetVacuumGripperStateResults> context) {
    try {
        auto gripper_state = vacuum_gripper_->readOnce();
        auto state = context.getResults().getState();
        
        state.setInControlRange(gripper_state.in_control_range);
        state.setPartDetached(gripper_state.part_detached);
        state.setPartPresent(gripper_state.part_present);
        state.setDeviceStatus(static_cast<uint8_t>(gripper_state.device_status));
        state.setActualPower(static_cast<double>(gripper_state.actual_power));
        state.setVacuum(static_cast<double>(gripper_state.vacuum));
        state.setTime(static_cast<double>(gripper_state.time.toMSec()));
        
        return kj::READY_NOW;
    } catch (const franka::Exception& e) {
        return kj::Promise<void>(kj::Exception(kj::Exception::Type::FAILED, __FILE__, __LINE__,
                                             kj::str("Failed to get vacuum gripper state: ", e.what())));
    }
}

kj::Promise<void> FrankaRobotRPCServiceImpl::vacuumGripperVacuum(
    capnp::CallContext<VacuumGripperVacuumParams, VacuumGripperVacuumResults> context) {
    try {
        auto params = context.getParams();
        
        // Keep async status consistent even for synchronous calls
        {
            std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
            vacuum_gripper_last_command_name_ = "vacuum";
            vacuum_gripper_error_message_.clear();
        }

        // Prevent synchronous commands from racing with the async worker
        {
            std::lock_guard<std::mutex> lock(vacuum_gripper_mutex_);
            if (has_pending_vacuum_gripper_command_ ||
                vacuum_gripper_command_status_.load() == VacuumGripperCommandStatus::BUSY) {
                vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::FAILED);
                {
                    std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
                    vacuum_gripper_error_message_ = "Command rejected: async vacuum gripper command in progress";
                }
                context.getResults().setSuccess(false);
                return kj::READY_NOW;
            }
        }

        vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::BUSY);
        bool success = vacuum_gripper_->vacuum(
            params.getControlPoint(),
            std::chrono::milliseconds(params.getTimeout()),
            static_cast<franka::VacuumGripper::ProductionSetupProfile>(params.getProfile())
        );
        context.getResults().setSuccess(success);

        if (success) {
            vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::SUCCESS);
        } else {
            vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::FAILED);
            std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
            vacuum_gripper_error_message_ = "Command returned false";
        }

        return kj::READY_NOW;
    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Vacuum gripper vacuum failed", e.what());
        vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::FAILED);
        {
            std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
            vacuum_gripper_error_message_ = e.what();
        }
        context.getResults().setSuccess(false);
        return kj::READY_NOW;
    }
}

kj::Promise<void> FrankaRobotRPCServiceImpl::vacuumGripperDropOff(
    capnp::CallContext<VacuumGripperDropOffParams, VacuumGripperDropOffResults> context) {
    try {
        auto params = context.getParams();
        
        // Keep async status consistent even for synchronous calls
        {
            std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
            vacuum_gripper_last_command_name_ = "dropOff";
            vacuum_gripper_error_message_.clear();
        }

        // Prevent synchronous commands from racing with the async worker
        {
            std::lock_guard<std::mutex> lock(vacuum_gripper_mutex_);
            if (has_pending_vacuum_gripper_command_ ||
                vacuum_gripper_command_status_.load() == VacuumGripperCommandStatus::BUSY) {
                vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::FAILED);
                {
                    std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
                    vacuum_gripper_error_message_ = "Command rejected: async vacuum gripper command in progress";
                }
                context.getResults().setSuccess(false);
                return kj::READY_NOW;
            }
        }

        vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::BUSY);
        bool success = vacuum_gripper_->dropOff(std::chrono::milliseconds(params.getTimeout()));
        context.getResults().setSuccess(success);

        if (success) {
            vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::SUCCESS);
        } else {
            vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::FAILED);
            std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
            vacuum_gripper_error_message_ = "Command returned false";
        }

        return kj::READY_NOW;
    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Vacuum gripper drop off failed", e.what());
        vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::FAILED);
        {
            std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
            vacuum_gripper_error_message_ = e.what();
        }
        context.getResults().setSuccess(false);
        return kj::READY_NOW;
    }
}

kj::Promise<void> FrankaRobotRPCServiceImpl::vacuumGripperStop(
    capnp::CallContext<VacuumGripperStopParams, VacuumGripperStopResults> context) {
    try {
        {
            std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
            vacuum_gripper_last_command_name_ = "stop";
            vacuum_gripper_error_message_.clear();
        }

        // Mark that stop was requested - worker thread will check this
        vacuum_gripper_stop_requested_.store(true);

        // Clear any pending commands from the queue
        bool had_pending = false;
        {
            std::lock_guard<std::mutex> lock(vacuum_gripper_mutex_);
            
            if (has_pending_vacuum_gripper_command_) {
                KJ_LOG(INFO, "Vacuum gripper stop: Flushing pending command from queue");
                has_pending_vacuum_gripper_command_ = false;
                pending_vacuum_gripper_command_ = VacuumGripperCommand::None;
                had_pending = true;
                
                vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::STOPPED);
                {
                    std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
                    vacuum_gripper_error_message_ = "Pending command cancelled by stop";
                }
            }
        }

        // stop() is thread-safe and will interrupt any running command
        bool success = vacuum_gripper_->stop();
        
        // Clear state after stop
        try {
            vacuum_gripper_->readOnce();
            KJ_LOG(INFO, "Vacuum gripper stop: Cleared state after stop");
        } catch (const franka::Exception& e) {
            KJ_LOG(WARNING, "Vacuum gripper stop: readOnce after stop failed (non-fatal)", e.what());
        }
        
        // Wait for the worker thread to acknowledge stop
        if (!had_pending && vacuum_gripper_command_status_.load() == VacuumGripperCommandStatus::BUSY) {
            KJ_LOG(INFO, "Vacuum gripper stop: Waiting for worker thread to acknowledge stop");
            
            std::unique_lock<std::mutex> lock(vacuum_gripper_mutex_);
            vacuum_gripper_done_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
                return vacuum_gripper_command_status_.load() != VacuumGripperCommandStatus::BUSY;
            });
            
            if (vacuum_gripper_command_status_.load() == VacuumGripperCommandStatus::BUSY) {
                KJ_LOG(WARNING, "Vacuum gripper stop: Worker didn't update status in time, forcing STOPPED");
                vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::STOPPED);
                {
                    std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
                    vacuum_gripper_error_message_ = "Command interrupted by stop";
                }
            }
        }
        
        context.getResults().setSuccess(success);
        KJ_LOG(INFO, "Vacuum gripper stop executed", success);

        return kj::READY_NOW;
    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Vacuum gripper stop failed", e.what());
        vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::FAILED);
        {
            std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
            vacuum_gripper_error_message_ = e.what();
        }
        context.getResults().setSuccess(false);
        return kj::READY_NOW;
    }
}

// ============================================================================
// Asynchronous Vacuum Gripper Methods
// ============================================================================

kj::Promise<void> FrankaRobotRPCServiceImpl::vacuumGripperVacuumAsync(
    capnp::CallContext<VacuumGripperVacuumAsyncParams, VacuumGripperVacuumAsyncResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    uint8_t control_point = params.getControlPoint();
    uint32_t timeout = params.getTimeout();
    uint8_t profile = params.getProfile();
    double command_timeout = params.getCommandTimeout();
    
    // Default timeout of 15 seconds if not specified or invalid
    if (command_timeout <= 0) {
        command_timeout = 15.0;
    }

    try {
        // Initialize vacuum gripper if needed
        if (!vacuum_gripper_) {
            vacuum_gripper_ = std::make_unique<franka::VacuumGripper>(robot_ip_);
        }
        
        // Ensure worker thread is running
        startVacuumGripperWorkerThread();
        
        // Check and queue atomically under mutex
        {
            std::lock_guard<std::mutex> lock(vacuum_gripper_mutex_);
            
            if (has_pending_vacuum_gripper_command_ || 
                vacuum_gripper_command_status_.load() == VacuumGripperCommandStatus::BUSY) {
                KJ_LOG(WARNING, "Vacuum gripper async: Command already in progress or pending");
                context.getResults().setStarted(false);
                return kj::READY_NOW;
            }
            
            // Queue the command
            pending_vacuum_gripper_command_ = VacuumGripperCommand::Vacuum;
            vacuum_gripper_cmd_control_point_ = control_point;
            vacuum_gripper_cmd_timeout_ = timeout;
            vacuum_gripper_cmd_profile_ = profile;
            vacuum_gripper_cmd_command_timeout_ = command_timeout;
            has_pending_vacuum_gripper_command_ = true;
            
            // Mark as BUSY immediately when queueing
            vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::BUSY);
            
            {
                std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
                vacuum_gripper_last_command_name_ = "vacuum";
                vacuum_gripper_error_message_.clear();
            }
        }
        
        // Notify worker thread
        vacuum_gripper_cv_.notify_one();
        
        KJ_LOG(INFO, "Vacuum gripper async: Vacuum command queued", control_point, timeout, profile, command_timeout);
        
        context.getResults().setStarted(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Vacuum gripper async vacuum failed to start", e.what());
        context.getResults().setStarted(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::vacuumGripperDropOffAsync(
    capnp::CallContext<VacuumGripperDropOffAsyncParams, VacuumGripperDropOffAsyncResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    uint32_t timeout = params.getTimeout();
    double command_timeout = params.getCommandTimeout();
    
    // Default timeout of 15 seconds if not specified or invalid
    if (command_timeout <= 0) {
        command_timeout = 15.0;
    }

    try {
        // Initialize vacuum gripper if needed
        if (!vacuum_gripper_) {
            vacuum_gripper_ = std::make_unique<franka::VacuumGripper>(robot_ip_);
        }
        
        // Ensure worker thread is running
        startVacuumGripperWorkerThread();
        
        // Check and queue atomically under mutex
        {
            std::lock_guard<std::mutex> lock(vacuum_gripper_mutex_);
            
            if (has_pending_vacuum_gripper_command_ || 
                vacuum_gripper_command_status_.load() == VacuumGripperCommandStatus::BUSY) {
                KJ_LOG(WARNING, "Vacuum gripper async: Command already in progress or pending");
                context.getResults().setStarted(false);
                return kj::READY_NOW;
            }
            
            // Queue the command
            pending_vacuum_gripper_command_ = VacuumGripperCommand::DropOff;
            vacuum_gripper_cmd_timeout_ = timeout;
            vacuum_gripper_cmd_command_timeout_ = command_timeout;
            has_pending_vacuum_gripper_command_ = true;
            
            // Mark as BUSY immediately when queueing
            vacuum_gripper_command_status_.store(VacuumGripperCommandStatus::BUSY);
            
            {
                std::lock_guard<std::mutex> status_lock(vacuum_gripper_status_mutex_);
                vacuum_gripper_last_command_name_ = "dropOff";
                vacuum_gripper_error_message_.clear();
            }
        }
        
        // Notify worker thread
        vacuum_gripper_cv_.notify_one();
        
        KJ_LOG(INFO, "Vacuum gripper async: DropOff command queued", timeout, command_timeout);
        
        context.getResults().setStarted(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Vacuum gripper async drop off failed to start", e.what());
        context.getResults().setStarted(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::getVacuumGripperAsyncStatus(
    capnp::CallContext<GetVacuumGripperAsyncStatusParams, GetVacuumGripperAsyncStatusResults> context) {
    
    if (robot_ip_.empty()) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    try {
        if (!vacuum_gripper_) {
            vacuum_gripper_ = std::make_unique<franka::VacuumGripper>(robot_ip_);
        }

        auto results = context.getResults();
        auto status = results.initStatus();
        fillVacuumGripperAsyncStatus(status);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to get vacuum gripper async status", e.what());
        throw;
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::vacuumGripperWaitForCommand(
    capnp::CallContext<VacuumGripperWaitForCommandParams, VacuumGripperWaitForCommandResults> context) {
    
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
        if (!vacuum_gripper_) {
            vacuum_gripper_ = std::make_unique<franka::VacuumGripper>(robot_ip_);
        }

        // Wait for command completion or timeout
        {
            std::unique_lock<std::mutex> lock(vacuum_gripper_mutex_);
            auto deadline = std::chrono::steady_clock::now() + 
                           std::chrono::duration<double>(timeout);
            
            bool completed = vacuum_gripper_done_cv_.wait_until(lock, deadline, [this] {
                auto status = vacuum_gripper_command_status_.load();
                return !has_pending_vacuum_gripper_command_ && status != VacuumGripperCommandStatus::BUSY;
            });
            
            if (!completed) {
                KJ_LOG(WARNING, "Vacuum gripper wait: Timed out waiting for command");
            }
        }

        auto results = context.getResults();
        auto status = results.initStatus();
        fillVacuumGripperAsyncStatus(status);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed during vacuum gripper wait", e.what());
        throw;
    }

    return kj::READY_NOW;
}
