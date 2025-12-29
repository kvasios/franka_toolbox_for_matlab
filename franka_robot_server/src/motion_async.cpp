// Copyright (c) 2025 Franka Robotics GmbH
// Motion Async Control - Async Implementation
//
// Provides async command handling for robot motion commands via a dedicated
// worker thread. Point-to-point and trajectory motions are executed asynchronously.

#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include "franka_robot_server/motion_generator.hpp"
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/control_types.h>
#include <memory>
#include <chrono>
#include <iostream>
#include <cmath>

namespace {
    static constexpr double kDeltaQMotionFinished = 1e-2;

    bool isInitialPositionValid(const std::array<double, 7>& current_position, 
                              const std::array<double, 7>& target_position) {
        for (size_t i = 0; i < 7; ++i) {
            if (std::abs(current_position[i] - target_position[i]) > kDeltaQMotionFinished) {
                return false;
            }
        }
        return true;
    }
}

// ============================================================================
// Worker Thread Management
// ============================================================================

void FrankaRobotRPCServiceImpl::startMotionWorkerThread() {
    if (motion_worker_thread_.joinable()) {
        return;  // Already running
    }
    
    motion_shutdown_requested_.store(false);
    motion_worker_thread_ = std::thread(&FrankaRobotRPCServiceImpl::motionWorkerLoop, this);
    KJ_LOG(INFO, "Motion worker thread started");
}

void FrankaRobotRPCServiceImpl::stopMotionWorkerThread() {
    motion_shutdown_requested_.store(true);
    
    {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        motion_cv_.notify_all();
    }
    
    if (motion_worker_thread_.joinable()) {
        motion_worker_thread_.join();
    }
    
    KJ_LOG(INFO, "Motion worker thread stopped");
}

void FrankaRobotRPCServiceImpl::motionWorkerLoop() {
    while (!motion_shutdown_requested_.load()) {
        MotionCommand cmd;
        std::array<double, 7> target_config;
        double speed_factor;
        std::vector<std::array<double, 7>> trajectory;
        double timeout;
        
        // Wait for command
        {
            std::unique_lock<std::mutex> lock(motion_mutex_);
            motion_cv_.wait(lock, [this] {
                return has_pending_motion_command_ || motion_shutdown_requested_.load();
            });
            
            if (motion_shutdown_requested_.load()) {
                break;
            }
            
            if (!has_pending_motion_command_) {
                continue;
            }
            
            // Copy command parameters
            cmd = pending_motion_command_;
            target_config = motion_cmd_target_config_;
            speed_factor = motion_cmd_speed_factor_;
            trajectory = motion_cmd_trajectory_;
            timeout = motion_cmd_timeout_;
            has_pending_motion_command_ = false;
        }
        
        // Clear stop flag before starting new command
        motion_stop_requested_.store(false);
        motion_progress_.store(0.0);
        
        // Execute command
        motion_command_status_.store(MotionCommandStatus::BUSY);
        
        bool success = false;
        std::string error_msg;
        
        try {
            auto start_time = std::chrono::steady_clock::now();
            
            switch (cmd) {
                case MotionCommand::PointToPoint: {
                    KJ_LOG(INFO, "Motion async: Executing point-to-point motion");
                    
                    MotionGenerator motion_generator(speed_factor, target_config);
                    
                    // Capture initial position and calculate total distance for progress tracking
                    auto initial_state = robot_->readOnce();
                    std::array<double, 7> q_start = initial_state.q;
                    double total_distance = 0.0;
                    for (size_t i = 0; i < 7; ++i) {
                        double diff = target_config[i] - q_start[i];
                        total_distance += diff * diff;
                    }
                    total_distance = std::sqrt(total_distance);
                    
                    // Wrap the motion generator to track progress and check for stop
                    auto control_callback = [this, &motion_generator, &start_time, timeout,
                                            &q_start, &target_config, total_distance](
                        const franka::RobotState& state,
                        franka::Duration period) -> franka::JointPositions {
                        
                        // Check for stop request
                        if (motion_stop_requested_.load()) {
                            throw franka::CommandException("Motion stopped by user");
                        }
                        
                        // Check timeout
                        auto elapsed = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - start_time).count();
                        if (timeout > 0 && elapsed > timeout) {
                            throw franka::CommandException("Motion timed out");
                        }
                        
                        // Calculate progress based on distance traveled
                        if (total_distance > 1e-6) {
                            double traveled = 0.0;
                            for (size_t i = 0; i < 7; ++i) {
                                double diff = state.q[i] - q_start[i];
                                traveled += diff * diff;
                            }
                            traveled = std::sqrt(traveled);
                            double progress = std::min(1.0, traveled / total_distance);
                            motion_progress_.store(progress);
                        }
                        
                        return motion_generator(state, period);
                    };
                    
                    robot_->control(control_callback);
                    success = true;
                    motion_progress_.store(1.0);
                    break;
                }
                    
                case MotionCommand::Trajectory: {
                    KJ_LOG(INFO, "Motion async: Executing trajectory motion", trajectory.size());
                    
                    // Check if initial position matches current robot position
                    auto current_state = robot_->readOnce();
                    if (!isInitialPositionValid(current_state.q, trajectory[0])) {
                        error_msg = "Initial trajectory point doesn't match current robot position";
                        motion_command_status_.store(MotionCommandStatus::FAILED);
                        break;
                    }
                    
                    uint64_t time_ms = 0;
                    size_t total_points = trajectory.size();
                    
                    auto control_callback = [this, &trajectory, &time_ms, total_points, &start_time, timeout](
                        const franka::RobotState& /*state*/,
                        franka::Duration period) -> franka::JointPositions {
                        
                        // Check for stop request
                        if (motion_stop_requested_.load()) {
                            throw franka::CommandException("Motion stopped by user");
                        }
                        
                        // Check timeout
                        auto elapsed = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - start_time).count();
                        if (timeout > 0 && elapsed > timeout) {
                            throw franka::CommandException("Motion timed out");
                        }
                        
                        time_ms += period.toMSec();
                        size_t current_point = std::min(static_cast<size_t>(time_ms), trajectory.size() - 1);
                        
                        // Update progress (avoid division by zero for single-point trajectories)
                        if (total_points > 1) {
                            motion_progress_.store(static_cast<double>(current_point) / static_cast<double>(total_points - 1));
                        } else {
                            motion_progress_.store(1.0);
                        }
                        
                        franka::JointPositions output(trajectory[current_point]);

                        if (current_point >= trajectory.size() - 1) {
                            return franka::MotionFinished(output);
                        }
                        
                        return output;
                    };

                    robot_->control(control_callback, franka::ControllerMode::kJointImpedance, true);
                    success = true;
                    motion_progress_.store(1.0);
                    break;
                }
                    
                default:
                    error_msg = "Unknown command";
                    break;
            }
            
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double>(end_time - start_time).count();
            
            if (success) {
                motion_command_status_.store(MotionCommandStatus::SUCCESS);
                KJ_LOG(INFO, "Motion async: Command succeeded in", elapsed, "seconds");
            }
            
        } catch (const franka::CommandException& e) {
            std::string what = e.what();
            if (motion_stop_requested_.load() || what.find("stopped") != std::string::npos) {
                error_msg = "Motion interrupted by stop";
                KJ_LOG(INFO, "Motion async: Command stopped by user");
                motion_command_status_.store(MotionCommandStatus::STOPPED);
            } else if (what.find("timed out") != std::string::npos) {
                error_msg = what;
                KJ_LOG(WARNING, "Motion async: Command timed out");
                motion_command_status_.store(MotionCommandStatus::TIMEOUT);
            } else {
                error_msg = std::string("CommandException: ") + what;
                KJ_LOG(ERROR, "Motion async: CommandException", what);
                motion_command_status_.store(MotionCommandStatus::FAILED);
            }
        } catch (const franka::NetworkException& e) {
            error_msg = std::string("NetworkException: ") + e.what();
            KJ_LOG(ERROR, "Motion async: NetworkException", e.what());
            motion_command_status_.store(MotionCommandStatus::FAILED);
        } catch (const franka::Exception& e) {
            if (motion_stop_requested_.load()) {
                error_msg = "Motion interrupted by stop";
                KJ_LOG(INFO, "Motion async: Command stopped by user");
                motion_command_status_.store(MotionCommandStatus::STOPPED);
            } else {
                error_msg = std::string("Exception: ") + e.what();
                KJ_LOG(ERROR, "Motion async: Exception", e.what());
                motion_command_status_.store(MotionCommandStatus::FAILED);
            }
        }
        
        // Update error message
        {
            std::lock_guard<std::mutex> lock(motion_status_mutex_);
            motion_error_message_ = error_msg;
        }
        
        // Notify waiting clients
        motion_done_cv_.notify_all();
    }
}

// ============================================================================
// Helper to fill MotionAsyncStatus
// ============================================================================

void FrankaRobotRPCServiceImpl::fillMotionAsyncStatus(MotionAsyncStatus::Builder& status) {
    // Get current robot state
    if (robot_) {
        try {
            franka::RobotState rs = robot_->readOnce();
            auto q = status.initQ(7);
            auto dq = status.initDq(7);
            for (size_t i = 0; i < 7; ++i) {
                q.set(i, rs.q[i]);
                dq.set(i, rs.dq[i]);
            }
        } catch (const franka::Exception& e) {
            KJ_LOG(WARNING, "Failed to read robot state for motion status", e.what());
            // Initialize with zeros
            auto q = status.initQ(7);
            auto dq = status.initDq(7);
            for (size_t i = 0; i < 7; ++i) {
                q.set(i, 0.0);
                dq.set(i, 0.0);
            }
        }
    } else {
        auto q = status.initQ(7);
        auto dq = status.initDq(7);
        for (size_t i = 0; i < 7; ++i) {
            q.set(i, 0.0);
            dq.set(i, 0.0);
        }
    }
    
    // Set command status
    status.setCommandStatus(motion_command_status_.load());
    status.setProgress(motion_progress_.load());
    
    // Set string fields (protected by mutex)
    {
        std::lock_guard<std::mutex> lock(motion_status_mutex_);
        status.setLastCommand(motion_last_command_name_);
        status.setErrorMessage(motion_error_message_);
    }
}

// ============================================================================
// Asynchronous Motion Methods
// ============================================================================

kj::Promise<void> FrankaRobotRPCServiceImpl::jointPointToPointMotionAsync(
    capnp::CallContext<JointPointToPointMotionAsyncParams, JointPointToPointMotionAsyncResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    auto target_config = params.getTargetConfiguration();
    double speed_factor = params.getSpeedFactor();
    double timeout = params.getTimeout();
    
    if (target_config.size() != 7) {
        KJ_FAIL_REQUIRE("Target configuration must have exactly 7 joint angles");
    }

    if (speed_factor <= 0.0 || speed_factor > 1.0) {
        KJ_FAIL_REQUIRE("Speed factor must be in range (0, 1]");
    }
    
    // Default timeout of 60 seconds if not specified or invalid
    if (timeout <= 0) {
        timeout = 60.0;
    }

    try {
        // Ensure worker thread is running
        startMotionWorkerThread();
        
        // Check and queue atomically under mutex
        {
            std::lock_guard<std::mutex> lock(motion_mutex_);
            
            if (has_pending_motion_command_ || 
                motion_command_status_.load() == MotionCommandStatus::BUSY) {
                KJ_LOG(WARNING, "Motion async: Command already in progress or pending");
                context.getResults().setStarted(false);
                return kj::READY_NOW;
            }
            
            // Queue the command
            pending_motion_command_ = MotionCommand::PointToPoint;
            for (size_t i = 0; i < 7; i++) {
                motion_cmd_target_config_[i] = target_config[i];
            }
            motion_cmd_speed_factor_ = speed_factor;
            motion_cmd_timeout_ = timeout;
            has_pending_motion_command_ = true;
            
            // Mark as BUSY immediately when queueing
            motion_command_status_.store(MotionCommandStatus::BUSY);
            motion_progress_.store(0.0);
            
            {
                std::lock_guard<std::mutex> status_lock(motion_status_mutex_);
                motion_last_command_name_ = "joint_point_to_point_motion";
                motion_error_message_.clear();
            }
        }
        
        // Notify worker thread
        motion_cv_.notify_one();
        
        KJ_LOG(INFO, "Motion async: Point-to-point command queued", speed_factor, timeout);
        
        context.getResults().setStarted(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Motion async point-to-point failed to start", e.what());
        context.getResults().setStarted(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::jointTrajectoryMotionAsync(
    capnp::CallContext<JointTrajectoryMotionAsyncParams, JointTrajectoryMotionAsyncResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    auto trajectory = params.getTrajectory();
    double timeout = params.getTimeout();
    
    if (trajectory.size() == 0) {
        KJ_FAIL_REQUIRE("Trajectory cannot be empty");
    }
    
    // Default timeout based on trajectory length if not specified
    if (timeout <= 0) {
        timeout = static_cast<double>(trajectory.size()) / 1000.0 + 10.0;  // trajectory length in seconds + 10s buffer
    }

    try {
        // Ensure worker thread is running
        startMotionWorkerThread();
        
        // Convert and validate trajectory
        std::vector<std::array<double, 7>> positions;
        for (auto point : trajectory) {
            if (point.getPositions().size() != 7) {
                KJ_FAIL_REQUIRE("Each trajectory point must have 7 position values");
            }
            std::array<double, 7> pos{};
            auto pos_list = point.getPositions();
            for (size_t i = 0; i < 7; ++i) {
                pos[i] = pos_list[i];
            }
            positions.push_back(pos);
        }
        
        // Check and queue atomically under mutex
        {
            std::lock_guard<std::mutex> lock(motion_mutex_);
            
            if (has_pending_motion_command_ || 
                motion_command_status_.load() == MotionCommandStatus::BUSY) {
                KJ_LOG(WARNING, "Motion async: Command already in progress or pending");
                context.getResults().setStarted(false);
                return kj::READY_NOW;
            }
            
            // Queue the command
            pending_motion_command_ = MotionCommand::Trajectory;
            motion_cmd_trajectory_ = std::move(positions);
            motion_cmd_timeout_ = timeout;
            has_pending_motion_command_ = true;
            
            // Mark as BUSY immediately when queueing
            motion_command_status_.store(MotionCommandStatus::BUSY);
            motion_progress_.store(0.0);
            
            {
                std::lock_guard<std::mutex> status_lock(motion_status_mutex_);
                motion_last_command_name_ = "joint_trajectory_motion";
                motion_error_message_.clear();
            }
        }
        
        // Notify worker thread
        motion_cv_.notify_one();
        
        KJ_LOG(INFO, "Motion async: Trajectory command queued", trajectory.size(), timeout);
        
        context.getResults().setStarted(true);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Motion async trajectory failed to start", e.what());
        context.getResults().setStarted(false);
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::getMotionAsyncStatus(
    capnp::CallContext<GetMotionAsyncStatusParams, GetMotionAsyncStatusResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    try {
        auto results = context.getResults();
        auto status = results.initStatus();
        fillMotionAsyncStatus(status);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to get motion async status", e.what());
        throw;
    }

    return kj::READY_NOW;
}

kj::Promise<void> FrankaRobotRPCServiceImpl::motionWaitForCommand(
    capnp::CallContext<MotionWaitForCommandParams, MotionWaitForCommandResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    auto params = context.getParams();
    double timeout = params.getTimeout();
    
    // Default timeout if not specified
    if (timeout <= 0) {
        timeout = 120.0;  // 2 minute default wait timeout for motions
    }

    try {
        // Wait for command completion or timeout
        {
            std::unique_lock<std::mutex> lock(motion_mutex_);
            auto deadline = std::chrono::steady_clock::now() + 
                           std::chrono::duration<double>(timeout);
            
            bool completed = motion_done_cv_.wait_until(lock, deadline, [this] {
                auto status = motion_command_status_.load();
                return !has_pending_motion_command_ && status != MotionCommandStatus::BUSY;
            });
            
            if (!completed) {
                KJ_LOG(WARNING, "Motion wait: Timed out waiting for command");
            }
        }

        auto results = context.getResults();
        auto status = results.initStatus();
        fillMotionAsyncStatus(status);

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed during motion wait", e.what());
        throw;
    }

    return kj::READY_NOW;
}

