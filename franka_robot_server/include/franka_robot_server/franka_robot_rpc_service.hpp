#pragma once

#include <capnp/ez-rpc.h>
#include "rpc.capnp.h"
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/gripper.h>
#include <franka/vacuum_gripper.h>

class FrankaRobotRPCServiceImpl final : public RPCService::Server {
public:
    FrankaRobotRPCServiceImpl() = default;
    explicit FrankaRobotRPCServiceImpl(uint16_t port) : server_port_(port) {}
    ~FrankaRobotRPCServiceImpl();

    kj::Promise<void> automaticErrorRecovery(
        capnp::CallContext<AutomaticErrorRecoveryParams, AutomaticErrorRecoveryResults> context) override;

    kj::Promise<void> initializeRobot(
        capnp::CallContext<InitializeRobotParams, InitializeRobotResults> context) override;

    kj::Promise<void> initializeGripper(
        capnp::CallContext<InitializeGripperParams, InitializeGripperResults> context) override;

    kj::Promise<void> initializeVacuumGripper(
        capnp::CallContext<InitializeVacuumGripperParams, InitializeVacuumGripperResults> context) override;

    kj::Promise<void> getRobotState(
        capnp::CallContext<GetRobotStateParams, GetRobotStateResults> context) override;

    kj::Promise<void> getJointPoses(
        capnp::CallContext<GetJointPosesParams, GetJointPosesResults> context) override;

    kj::Promise<void> jointPointToPointMotion(
        capnp::CallContext<JointPointToPointMotionParams, JointPointToPointMotionResults> context) override;

    kj::Promise<void> jointTrajectoryMotion(
        capnp::CallContext<JointTrajectoryMotionParams, JointTrajectoryMotionResults> context) override;

    kj::Promise<void> getGripperState(
        capnp::CallContext<GetGripperStateParams, GetGripperStateResults> context) override;

    kj::Promise<void> gripperGrasp(
        capnp::CallContext<GripperGraspParams, GripperGraspResults> context) override;

    kj::Promise<void> gripperHoming(
        capnp::CallContext<GripperHomingParams, GripperHomingResults> context) override;

    kj::Promise<void> gripperMove(
        capnp::CallContext<GripperMoveParams, GripperMoveResults> context) override;

    kj::Promise<void> gripperStop(
        capnp::CallContext<GripperStopParams, GripperStopResults> context) override;

    // Async gripper methods
    kj::Promise<void> gripperMoveAsync(
        capnp::CallContext<GripperMoveAsyncParams, GripperMoveAsyncResults> context) override;

    kj::Promise<void> gripperGraspAsync(
        capnp::CallContext<GripperGraspAsyncParams, GripperGraspAsyncResults> context) override;

    kj::Promise<void> getGripperAsyncStatus(
        capnp::CallContext<GetGripperAsyncStatusParams, GetGripperAsyncStatusResults> context) override;

    kj::Promise<void> gripperWaitForCommand(
        capnp::CallContext<GripperWaitForCommandParams, GripperWaitForCommandResults> context) override;

    kj::Promise<void> setCollisionBehavior(
        capnp::CallContext<SetCollisionBehaviorParams, SetCollisionBehaviorResults> context) override;

    kj::Promise<void> setLoadInertia(
        capnp::CallContext<SetLoadInertiaParams, SetLoadInertiaResults> context) override;

    // Vacuum Gripper methods
    kj::Promise<void> getVacuumGripperState(
        capnp::CallContext<GetVacuumGripperStateParams, GetVacuumGripperStateResults> context) override;

    kj::Promise<void> vacuumGripperVacuum(
        capnp::CallContext<VacuumGripperVacuumParams, VacuumGripperVacuumResults> context) override;

    kj::Promise<void> vacuumGripperDropOff(
        capnp::CallContext<VacuumGripperDropOffParams, VacuumGripperDropOffResults> context) override;

    kj::Promise<void> vacuumGripperStop(
        capnp::CallContext<VacuumGripperStopParams, VacuumGripperStopResults> context) override;

    // Impedance control
    kj::Promise<void> setJointImpedance(
        capnp::CallContext<SetJointImpedanceParams, SetJointImpedanceResults> context) override;

    kj::Promise<void> setCartesianImpedance(
        capnp::CallContext<SetCartesianImpedanceParams, SetCartesianImpedanceResults> context) override;

    // Guiding mode
    kj::Promise<void> setGuidingMode(
        capnp::CallContext<SetGuidingModeParams, SetGuidingModeResults> context) override;

    // Frame transformations
    kj::Promise<void> setK(
        capnp::CallContext<SetKParams, SetKResults> context) override;

    kj::Promise<void> setEE(
        capnp::CallContext<SetEEParams, SetEEResults> context) override;

    // Robot control
    kj::Promise<void> stopRobot(
        capnp::CallContext<StopRobotParams, StopRobotResults> context) override;

    // Health check
    kj::Promise<void> ping(
        capnp::CallContext<PingParams, PingResults> context) override;

private:
    uint16_t server_port_ = 0;  // Stored for ping response
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<franka::Model> model_;
    std::unique_ptr<franka::Gripper> gripper_;
    std::unique_ptr<franka::VacuumGripper> vacuum_gripper_;
    std::string robot_ip_;

    // Async gripper command infrastructure
    void startGripperWorkerThread();
    void stopGripperWorkerThread();
    void gripperWorkerLoop();
    void fillGripperAsyncStatus(GripperAsyncStatus::Builder& status);
    
    std::thread gripper_worker_thread_;
    std::mutex gripper_mutex_;
    std::condition_variable gripper_cv_;
    std::condition_variable gripper_done_cv_;  // Notified when command completes
    std::atomic<bool> gripper_shutdown_requested_{false};
    
    // Async command state
    enum class GripperCommand { None, Move, Grasp };
    GripperCommand pending_gripper_command_{GripperCommand::None};
    bool has_pending_gripper_command_{false};
    
    // Command parameters
    double gripper_cmd_width_{0.0};
    double gripper_cmd_speed_{0.0};
    double gripper_cmd_force_{0.0};
    double gripper_cmd_epsilon_inner_{0.0};
    double gripper_cmd_epsilon_outer_{0.0};
    double gripper_cmd_timeout_{15.0};
    
    // Command status (atomic for thread-safe reads)
    std::atomic<GripperCommandStatus> gripper_command_status_{GripperCommandStatus::IDLE};
    std::string gripper_last_command_name_;
    std::string gripper_error_message_;
    mutable std::mutex gripper_status_mutex_;  // Protects string members
}; 