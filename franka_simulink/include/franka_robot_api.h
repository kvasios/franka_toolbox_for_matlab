// Copyright (c) 2025 Franka Robotics GmbH
// Franka Robot API - Header
//
// This provides the interface between Simulink code generation and libfranka
// with support for multiple control modes and shared robot connections.
//
// Architecture:
//   FrankaRobotManager    - Global singleton registry (IP → Instance)
//   FrankaRobotInstance   - Shared robot connection (one per physical robot)
//   FrankaRobotContext    - Per-block lightweight context (I/O pointers, callbacks)
//
// Key Features:
//   - Multiple blocks can target the same robot IP (sequential control handoff)
//   - Auxiliary blocks (stop, errorRecovery, readOnce) work by IP lookup
//   - Single connection per robot, shared across all blocks
//   - Thread-safe control ownership with atomic flags

#ifndef FRANKA_ROBOT_API_H
#define FRANKA_ROBOT_API_H

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "franka_simulink_types.h"

// Forward declarations
class FrankaRobotInstance;
class FrankaRobotContext;

/**
 * @brief Control mode enumeration
 * 
 * Matches the block mask dropdown order for mode selection.
 * 
 * Single-callback modes (0-4) use the robot's internal controller for non-torque modes.
 * Dual-callback modes (5-8) allow user to provide both torque AND motion generator callbacks.
 */
enum class FrankaControlMode {
    /* Single-callback modes */
    Torques = 0,                    ///< Direct torque control
    JointPositions = 1,             ///< Joint position with internal impedance
    JointVelocities = 2,            ///< Joint velocity with internal impedance
    CartesianPose = 3,              ///< Cartesian pose with internal impedance
    CartesianVelocities = 4,        ///< Cartesian velocity with internal impedance
    /* Dual-callback modes (torque + motion generator) */
    TorquesJointPositions = 5,      ///< External torque + joint position motion gen
    TorquesJointVelocities = 6,     ///< External torque + joint velocity motion gen
    TorquesCartesianPose = 7,       ///< External torque + Cartesian pose motion gen
    TorquesCartesianVelocities = 8  ///< External torque + Cartesian velocity motion gen
};

/**
 * @brief Type for the controller callback function
 * 
 * This is a pointer to the Simulink-generated function that executes
 * the function-call subsystem (user's controller).
 */
using ControllerCallback = void (*)(void* user_data, double dt_sec);

// ============================================================================
// FrankaRobotInstance - Shared Robot Connection (one per physical robot)
// ============================================================================

/**
 * @brief Shared robot instance representing a single physical robot connection.
 * 
 * This class owns the franka::Robot and franka::Model objects and manages
 * control ownership among multiple FrankaRobotContext instances that may
 * target the same robot (same IP).
 * 
 * Lifecycle:
 *   - Created by FrankaRobotManager::getOrCreate(ip)
 *   - Shared by all blocks targeting the same IP
 *   - Destroyed when FrankaRobotManager shuts down
 * 
 * Control Ownership:
 *   - Only one FrankaRobotContext can have control at a time
 *   - claimControl() / releaseControl() manage ownership
 *   - Auxiliary operations (stop, errorRecovery) work via atomic flags
 */
class FrankaRobotInstance {
public:
    /**
     * @brief Construct and connect to robot at given IP
     * @param ip Robot IP address
     * @throws franka::Exception if connection fails
     */
    explicit FrankaRobotInstance(const std::string& ip);
    
    ~FrankaRobotInstance();
    
    // Non-copyable, non-movable
    FrankaRobotInstance(const FrankaRobotInstance&) = delete;
    FrankaRobotInstance& operator=(const FrankaRobotInstance&) = delete;
    FrankaRobotInstance(FrankaRobotInstance&&) = delete;
    FrankaRobotInstance& operator=(FrankaRobotInstance&&) = delete;
    
    // ========================================================================
    // Accessors
    // ========================================================================
    
    const std::string& ip() const { return ip_; }
    franka::Robot& robot() { return *robot_; }
    franka::Model& model() { return *model_; }
    
    // ========================================================================
    // Control Ownership
    // ========================================================================
    
    /**
     * @brief Attempt to claim control of this robot
     * @param ctx The context requesting control
     * @return true if control was granted, false if another context has control
     * 
     * Thread-safe. Only one context can have control at a time.
     */
    bool claimControl(FrankaRobotContext* ctx);
    
    /**
     * @brief Release control of this robot
     * @param ctx The context releasing control (must be current owner)
     */
    void releaseControl(FrankaRobotContext* ctx);
    
    /**
     * @brief Check if any context currently has control
     */
    bool isControlActive() const { return active_controller_.load() != nullptr; }
    
    /**
     * @brief Get the context currently in control (may be nullptr)
     */
    FrankaRobotContext* activeController() const { return active_controller_.load(); }
    
    // ========================================================================
    // Universal Operations (callable by any block, any time)
    // ========================================================================
    
    /**
     * @brief Request stop of current control (thread-safe, atomic)
     * 
     * This sets a flag that the active controller's callback will check.
     * Safe to call from any thread, any time.
     */
    void requestStop();
    
    /**
     * @brief Check if stop has been requested
     */
    bool isStopRequested() const { return stop_requested_.load(); }
    
    /**
     * @brief Clear the stop request flag (called when starting new control)
     */
    void clearStopRequest() { stop_requested_.store(false); }
    
    /**
     * @brief Attempt automatic error recovery
     * @return true if recovery succeeded, false if failed or control is active
     * 
     * Can only be called when no control is active.
     */
    bool automaticErrorRecovery();
    
    /**
     * @brief Read robot state once (outside of control loop)
     * @param state_out Output for robot state (can be nullptr)
     * @param model_out Output for model data (can be nullptr)
     * @return true if read succeeded, false if control is active
     * 
     * Can only be called when no control is active.
     */
    bool readOnce(FrankaRobotStateBus* state_out, FrankaModelDataBus* model_out = nullptr);
    
    // ========================================================================
    // Settings Application
    // ========================================================================
    
    /**
     * @brief Apply robot settings
     * @param settings Settings to apply
     * 
     * Should be called before starting control.
     */
    void applySettings(const FrankaRobotSettingsBus* settings);
    
private:
    std::string ip_;
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<franka::Model> model_;
    
    std::atomic<FrankaRobotContext*> active_controller_{nullptr};
    std::atomic<bool> stop_requested_{false};
    
    // Helper functions
    void copyRobotState(const franka::RobotState& src, FrankaRobotStateBus* dst);
    void computeModelData(const franka::RobotState& state, FrankaModelDataBus* dst);
};

// ============================================================================
// FrankaRobotManager - Global Registry (IP → Instance)
// ============================================================================

/**
 * @brief Global manager for robot instances, indexed by IP.
 * 
 * This singleton-like class maintains a registry of all robot connections.
 * All operations can be performed by IP address, making auxiliary blocks
 * trivial to implement (no handle wiring needed).
 * 
 * Thread Safety:
 *   - All static methods are thread-safe
 *   - Uses mutex for instance map access
 *   - Individual instance operations use atomic flags
 */
class FrankaRobotManager {
public:
    // ========================================================================
    // Instance Management
    // ========================================================================
    
    /**
     * @brief Get or create a robot instance for the given IP
     * @param ip Robot IP address
     * @return Pointer to the instance (never null if no exception)
     * @throws franka::Exception if connection fails (for new instance)
     */
    static FrankaRobotInstance* getOrCreate(const std::string& ip);
    
    /**
     * @brief Get existing instance for IP (no creation)
     * @param ip Robot IP address
     * @return Pointer to instance, or nullptr if not connected
     */
    static FrankaRobotInstance* get(const std::string& ip);
    
    /**
     * @brief Check if a robot is connected
     * @param ip Robot IP address
     */
    static bool isConnected(const std::string& ip);
    
    /**
     * @brief Shutdown and remove all instances
     * 
     * Called during model terminate to clean up all connections.
     */
    static void shutdownAll();
    
    // ========================================================================
    // Operations by IP (for auxiliary blocks)
    // ========================================================================
    
    /**
     * @brief Request stop for robot at IP
     * @param ip Robot IP address
     * 
     * If no robot is connected at this IP, does nothing.
     */
    static void stop(const std::string& ip);
    
    /**
     * @brief Attempt automatic error recovery for robot at IP
     * @param ip Robot IP address
     * @return true if recovery succeeded
     */
    static bool automaticErrorRecovery(const std::string& ip);
    
    /**
     * @brief Read robot state once for robot at IP
     * @param ip Robot IP address
     * @param state_out Output for robot state
     * @param model_out Output for model data (optional)
     * @return true if read succeeded
     */
    static bool readOnce(const std::string& ip, 
                         FrankaRobotStateBus* state_out,
                         FrankaModelDataBus* model_out = nullptr);
    
    /**
     * @brief Check if control is currently running for robot at IP
     * @param ip Robot IP address
     */
    static bool isControlRunning(const std::string& ip);
    
    // Sanitize IP string (trim whitespace, handle embedded nulls)
    // Public so contexts can compare IPs consistently
    static std::string sanitizeIP(const std::string& ip);
    
private:
    static std::unordered_map<std::string, std::unique_ptr<FrankaRobotInstance>> instances_;
    static std::mutex mutex_;
};

// ============================================================================
// FrankaRobotContext - Per-Block Lightweight Context
// ============================================================================

/**
 * @brief Lightweight context for a single Simulink block instance.
 * 
 * This class manages:
 *   - I/O pointer wiring (Simulink signals ↔ robot data)
 *   - Controller callback registration
 *   - Control thread lifecycle
 * 
 * It does NOT own the robot connection - it borrows from FrankaRobotInstance
 * via FrankaRobotManager.
 * 
 * Execution Model:
 *   1. initialize(ip) - Get/create shared instance from manager
 *   2. setControllerCallback(...) - Wire up Simulink controller
 *   3. setXxxPointer(...) - Wire up I/O signals
 *   4. startControl() - Claim control and spawn control thread
 *   5. [control runs at 1kHz in callback]
 *   6. requestStop() or falling enable edge - Control ends
 *   7. Control thread exits, releases control
 */
class FrankaRobotContext {
public:
    FrankaRobotContext();
    ~FrankaRobotContext();
    
    // Non-copyable, non-movable
    FrankaRobotContext(const FrankaRobotContext&) = delete;
    FrankaRobotContext& operator=(const FrankaRobotContext&) = delete;
    FrankaRobotContext(FrankaRobotContext&&) = delete;
    FrankaRobotContext& operator=(FrankaRobotContext&&) = delete;
    
    // ========================================================================
    // Lifecycle
    // ========================================================================
    
    /**
     * @brief Initialize by connecting to robot at IP
     * @param robot_ip IP address of the Franka robot
     * 
     * Gets or creates a shared FrankaRobotInstance from the manager.
     */
    void initialize(const std::string& robot_ip);
    
    /**
     * @brief Check if initialized (connected to a robot instance)
     */
    bool isInitialized() const { return instance_ != nullptr; }
    
    /**
     * @brief Set the controller callback function
     * @param callback Function pointer to the controller callback
     * @param user_data Opaque pointer passed to callback on each invocation
     */
    void setControllerCallback(ControllerCallback callback, void* user_data);
    
    /**
     * @brief Set the control mode
     * @param mode Control mode
     */
    void setControlMode(FrankaControlMode mode);
    
    /**
     * @brief Configure real-time thread settings (PREEMPT_RT)
     * @param priority Thread priority for SCHED_FIFO (0=disabled, 1-99)
     * @param cpu_affinity CPU core to pin thread to (-1=no pinning, 0+=specific core)
     * @param lock_memory Whether to lock memory with mlockall
     */
    void setRealtimeConfig(int priority, int cpu_affinity, bool lock_memory);
    
    /**
     * @brief Shutdown and cleanup
     */
    void shutdown();
    
    // ========================================================================
    // I/O Pointer Setup
    // ========================================================================
    
    void setStateOutputPointer(FrankaRobotStateBus* state_ptr);
    void setModelOutputPointer(FrankaModelDataBus* model_ptr);
    void setDtOutputPointer(double* dt_sec_ptr);
    void setSettingsInputPointer(const FrankaRobotSettingsBus* settings_ptr);
    
    // Single-callback mode input setters
    void setTorqueInputPointer(const double* tau_J_d_ptr);
    void setJointPositionInputPointer(const double* q_d_ptr);
    void setJointVelocityInputPointer(const double* dq_d_ptr);
    void setCartesianPoseInputPointer(const double* O_T_EE_d_ptr, const double* elbow_d_ptr);
    void setCartesianVelocityInputPointer(const double* O_dP_EE_d_ptr, const double* elbow_d_ptr);
    
    // Dual-callback mode input setters
    void setTorquesJointPositionInputPointers(const double* tau_J_d_ptr, const double* q_d_ptr);
    void setTorquesJointVelocityInputPointers(const double* tau_J_d_ptr, const double* dq_d_ptr);
    void setTorquesCartesianPoseInputPointers(const double* tau_J_d_ptr, 
                                               const double* O_T_EE_d_ptr, 
                                               const double* elbow_d_ptr);
    void setTorquesCartesianVelocityInputPointers(const double* tau_J_d_ptr,
                                                   const double* O_dP_EE_d_ptr,
                                                   const double* elbow_d_ptr);
    
    // Legacy interface (deprecated)
    void setInputPointers(const double* tau_J_d_ptr);
    
    // ========================================================================
    // Control
    // ========================================================================
    
    /**
     * @brief Start the control loop
     * 
     * Claims control of the robot instance and spawns the control thread.
     */
    void startControl();
    
    /**
     * @brief Request graceful stop
     */
    void requestStop();
    
    /**
     * @brief Check if control is currently running (this context)
     */
    bool isControlRunning() const { return running_.load(); }
    
private:
    void controlThreadFunc();
    void publishStateOnce(double dt_sec_override);
    
    // Mode-specific control callbacks
    franka::Torques torqueCallback(const franka::RobotState& state, franka::Duration period);
    franka::JointPositions jointPositionCallback(const franka::RobotState& state, franka::Duration period);
    franka::JointVelocities jointVelocityCallback(const franka::RobotState& state, franka::Duration period);
    franka::CartesianPose cartesianPoseCallback(const franka::RobotState& state, franka::Duration period);
    franka::CartesianVelocities cartesianVelocityCallback(const franka::RobotState& state, franka::Duration period);
    
    // Dual-callback mode callbacks
    franka::Torques dualTorqueCallback(const franka::RobotState& state, franka::Duration period);
    franka::JointPositions dualJointPositionMotionCallback(const franka::RobotState& state, franka::Duration period);
    franka::JointVelocities dualJointVelocityMotionCallback(const franka::RobotState& state, franka::Duration period);
    franka::CartesianPose dualCartesianPoseMotionCallback(const franka::RobotState& state, franka::Duration period);
    franka::CartesianVelocities dualCartesianVelocityMotionCallback(const franka::RobotState& state, franka::Duration period);
    
    // Common pre-callback logic
    bool executePreCallback(const franka::RobotState& state, franka::Duration period);
    
    // Helper functions
    void copyRobotState(const franka::RobotState& src, FrankaRobotStateBus* dst);
    void computeModelData(const franka::RobotState& state, FrankaModelDataBus* dst);
    
    // Shared robot instance (borrowed from manager, not owned)
    FrankaRobotInstance* instance_{nullptr};
    
    // Control mode
    FrankaControlMode control_mode_{FrankaControlMode::Torques};
    
    // Real-time thread configuration (PREEMPT_RT)
    int rt_priority_{98};        ///< SCHED_FIFO priority (0=disabled, 1-99)
    int rt_cpu_affinity_{-1};    ///< CPU core (-1=no pinning)
    bool rt_lock_memory_{true};  ///< Lock memory with mlockall
    
    // Control state
    std::atomic<bool> running_{false};
    std::thread control_thread_;
    
    // Controller callback (points to Simulink-generated function)
    ControllerCallback controller_callback_{nullptr};
    void* controller_user_data_{nullptr};
    
    // Pointers to Simulink output signals
    FrankaRobotStateBus* state_out_{nullptr};
    FrankaModelDataBus* model_out_{nullptr};
    double* dt_sec_out_{nullptr};
    
    // Pointer to Simulink input settings bus
    const FrankaRobotSettingsBus* settings_in_{nullptr};
    
    // Pointers to Simulink input signals (mode-specific)
    const double* tau_J_d_in_{nullptr};
    const double* q_d_in_{nullptr};
    const double* dq_d_in_{nullptr};
    const double* O_T_EE_d_in_{nullptr};
    const double* O_dP_EE_d_in_{nullptr};
    const double* elbow_d_in_{nullptr};
};

#endif // FRANKA_ROBOT_API_H
