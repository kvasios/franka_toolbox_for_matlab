#include "franka_robot_server/franka_robot_rpc_service.hpp"
#include <franka/exception.h>
#include <franka/robot.h>

kj::Promise<void> FrankaRobotRPCServiceImpl::stopRobot(
    capnp::CallContext<StopRobotParams, StopRobotResults> context) {
    
    if (!robot_) {
        KJ_FAIL_REQUIRE("Robot not initialized");
    }

    try {
        // Signal async motion to stop (will be checked in worker thread)
        motion_stop_requested_.store(true);
        
        // Clear any pending motion commands
        {
            std::lock_guard<std::mutex> lock(motion_mutex_);
            if (has_pending_motion_command_) {
                KJ_LOG(INFO, "Stop robot: Flushing pending motion command");
                has_pending_motion_command_ = false;
                pending_motion_command_ = MotionCommand::None;
                motion_command_status_.store(MotionCommandStatus::STOPPED);
                {
                    std::lock_guard<std::mutex> status_lock(motion_status_mutex_);
                    motion_error_message_ = "Pending command cancelled by stop";
                }
            }
        }
        
        // Stop all currently running motions (this will cause the control callback to throw)
        robot_->stop();
        
        // Wait briefly for the worker thread to update status
        if (motion_command_status_.load() == MotionCommandStatus::BUSY) {
            std::unique_lock<std::mutex> lock(motion_mutex_);
            motion_done_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
                return motion_command_status_.load() != MotionCommandStatus::BUSY;
            });
            
            // Force status update if worker didn't respond
            if (motion_command_status_.load() == MotionCommandStatus::BUSY) {
                motion_command_status_.store(MotionCommandStatus::STOPPED);
                {
                    std::lock_guard<std::mutex> status_lock(motion_status_mutex_);
                    motion_error_message_ = "Motion interrupted by stop";
                }
            }
        }

        auto results = context.getResults();
        results.setSuccess(true);
        KJ_LOG(INFO, "Robot stopped");

    } catch (const franka::Exception& e) {
        KJ_LOG(ERROR, "Failed to stop robot", e.what());
        auto results = context.getResults();
        results.setSuccess(false);
    }

    return kj::READY_NOW;
}

