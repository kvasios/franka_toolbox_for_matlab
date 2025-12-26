// Copyright (c) 2025 Franka Robotics GmbH
// Franka Vacuum Gripper API - Header
//
// This provides the interface between Simulink code generation and libfranka
// vacuum gripper with async command handling via a dedicated worker thread.
//
// Architecture:
//   FrankaVacuumGripperManager    - Global singleton registry (IP → Instance)
//   FrankaVacuumGripperInstance   - Shared vacuum gripper connection (one per physical gripper)
//   FrankaVacuumGripperContext    - Per-block lightweight context (I/O pointers, state polling)
//
// Key Features:
//   - Async command execution (vacuum, dropOff, stop) via worker thread
//   - Commands triggered by rising edge signals
//   - Thread-safe command queue and status reporting
//   - Periodic state polling for continuous gripper state output

#ifndef FRANKA_VACUUM_GRIPPER_API_H
#define FRANKA_VACUUM_GRIPPER_API_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <franka/exception.h>
#include <franka/vacuum_gripper.h>

#include "franka_simulink_types.h"

// Forward declarations
class FrankaVacuumGripperInstance;
class FrankaVacuumGripperContext;

/**
 * @brief Command request structure for the async command queue
 */
struct VacuumGripperCommandRequest {
    FrankaVacuumGripperCommand command;
    
    // Vacuum parameters
    uint8_t vacuum_setpoint;    // [10*mbar]
    std::chrono::milliseconds vacuum_timeout;
    franka::VacuumGripper::ProductionSetupProfile vacuum_profile;
    
    // Drop off parameters
    std::chrono::milliseconds dropoff_timeout;
};

// ============================================================================
// FrankaVacuumGripperInstance - Shared Vacuum Gripper Connection
// ============================================================================

/**
 * @brief Shared vacuum gripper instance representing a single physical vacuum gripper connection.
 * 
 * This class owns the franka::VacuumGripper object and manages the async command
 * worker thread that executes blocking gripper operations.
 */
class FrankaVacuumGripperInstance {
public:
    /**
     * @brief Construct and connect to vacuum gripper at given IP
     * @param ip Robot IP address (vacuum gripper is connected to robot)
     * @throws franka::Exception if connection fails
     */
    explicit FrankaVacuumGripperInstance(const std::string& ip);
    
    ~FrankaVacuumGripperInstance();
    
    // Non-copyable, non-movable
    FrankaVacuumGripperInstance(const FrankaVacuumGripperInstance&) = delete;
    FrankaVacuumGripperInstance& operator=(const FrankaVacuumGripperInstance&) = delete;
    FrankaVacuumGripperInstance(FrankaVacuumGripperInstance&&) = delete;
    FrankaVacuumGripperInstance& operator=(FrankaVacuumGripperInstance&&) = delete;
    
    // ========================================================================
    // Accessors
    // ========================================================================
    
    const std::string& ip() const { return ip_; }
    franka::VacuumGripper& gripper() { return *gripper_; }
    
    // ========================================================================
    // Command Execution (Async)
    // ========================================================================
    
    /**
     * @brief Queue a command for async execution
     * @param request Command request with all parameters
     */
    void queueCommand(const VacuumGripperCommandRequest& request);
    
    /**
     * @brief Stop immediately - interrupts any running command
     * @return true if stop succeeded
     * 
     * This calls gripper_->stop() directly (NOT through the queue) to
     * interrupt any currently running vacuum/dropOff command.
     * 
     * Thread-safe: franka::VacuumGripper members are threadsafe per libfranka docs.
     */
    bool stopImmediate();
    
    /**
     * @brief Check if a command is currently executing
     */
    bool isCommandInProgress() const { return command_in_progress_.load(); }
    
    /**
     * @brief Get current command status
     */
    FrankaVacuumGripperStatus getStatus() const { return static_cast<FrankaVacuumGripperStatus>(status_.load()); }
    
    /**
     * @brief Get last executed command
     */
    FrankaVacuumGripperCommand getLastCommand() const { return static_cast<FrankaVacuumGripperCommand>(last_command_.load()); }
    
    /**
     * @brief Get last command result (true = success)
     */
    bool getLastCommandSuccess() const { return last_command_success_.load(); }
    
    /**
     * @brief Get error code from last failed command
     */
    int getErrorCode() const { return error_code_.load(); }
    
    // ========================================================================
    // State Reading
    // ========================================================================
    
    /**
     * @brief Read vacuum gripper state (blocking call)
     * @param state_out Output for gripper state
     * @return true if read succeeded
     */
    bool readState(FrankaVacuumGripperStateBus* state_out);
    
    /**
     * @brief Get cached state with current status (thread-safe)
     * @param state_out Output for state
     */
    void getCachedState(FrankaVacuumGripperStateBus* state_out) const;
    
    // ========================================================================
    // Lifecycle
    // ========================================================================
    
    /**
     * @brief Shutdown the command worker thread
     */
    void shutdown();

private:
    void commandWorkerThread();
    void executeCommand(const VacuumGripperCommandRequest& request);
    void updateCachedState();
    void copyVacuumGripperState(const franka::VacuumGripperState& src, FrankaVacuumGripperStateBus* dst);
    
    std::string ip_;
    std::unique_ptr<franka::VacuumGripper> gripper_;
    
    // Command worker thread
    std::thread command_thread_;
    std::atomic<bool> shutdown_requested_{false};
    
    // Command queue (single slot - latest command wins)
    std::mutex command_mutex_;
    std::condition_variable command_cv_;
    bool has_pending_command_{false};
    VacuumGripperCommandRequest pending_command_;
    
    // Command execution state
    std::atomic<bool> command_in_progress_{false};
    std::atomic<int> status_{FRANKA_VACUUM_STATUS_IDLE};
    std::atomic<int> last_command_{FRANKA_VACUUM_CMD_NONE};
    std::atomic<bool> last_command_success_{false};
    std::atomic<int> error_code_{0};
    
    // Cached state
    mutable std::mutex state_mutex_;
    FrankaVacuumGripperStateBus cached_state_{};
};

// ============================================================================
// FrankaVacuumGripperManager - Global Registry (IP → Instance)
// ============================================================================

/**
 * @brief Global manager for vacuum gripper instances, indexed by IP.
 */
class FrankaVacuumGripperManager {
public:
    // ========================================================================
    // Instance Management
    // ========================================================================
    
    static FrankaVacuumGripperInstance* getOrCreate(const std::string& ip);
    static FrankaVacuumGripperInstance* get(const std::string& ip);
    static bool isConnected(const std::string& ip);
    static void shutdownAll();
    
    // ========================================================================
    // Operations by IP
    // ========================================================================
    
    static void vacuum(const std::string& ip, uint8_t setpoint, 
                       std::chrono::milliseconds timeout,
                       franka::VacuumGripper::ProductionSetupProfile profile);
    static void dropOff(const std::string& ip, std::chrono::milliseconds timeout);
    static void stop(const std::string& ip);
    static bool readState(const std::string& ip, FrankaVacuumGripperStateBus* state_out);
    
    static std::string sanitizeIP(const std::string& ip);
    
private:
    static std::unordered_map<std::string, std::unique_ptr<FrankaVacuumGripperInstance>> instances_;
    static std::mutex mutex_;
};

// ============================================================================
// FrankaVacuumGripperContext - Per-Block Lightweight Context
// ============================================================================

/**
 * @brief Lightweight context for a single Simulink vacuum gripper block instance.
 */
class FrankaVacuumGripperContext {
public:
    FrankaVacuumGripperContext();
    ~FrankaVacuumGripperContext();
    
    // Non-copyable, non-movable
    FrankaVacuumGripperContext(const FrankaVacuumGripperContext&) = delete;
    FrankaVacuumGripperContext& operator=(const FrankaVacuumGripperContext&) = delete;
    FrankaVacuumGripperContext(FrankaVacuumGripperContext&&) = delete;
    FrankaVacuumGripperContext& operator=(FrankaVacuumGripperContext&&) = delete;
    
    // ========================================================================
    // Lifecycle
    // ========================================================================
    
    /**
     * @brief Initialize by connecting to vacuum gripper at IP (blocking)
     */
    void initialize(const std::string& robot_ip);
    
    /**
     * @brief Initialize by connecting to vacuum gripper at IP (non-blocking)
     * 
     * Spawns a background thread to perform the connection.
     * Check isConnecting() and isInitialized() to track progress.
     */
    void initializeAsync(const std::string& robot_ip);
    
    bool isInitialized() const { return instance_ != nullptr; }
    
    /**
     * @brief Check if async connection is in progress
     */
    bool isConnecting() const { return connection_in_progress_.load(); }
    
    void shutdown();
    
    // ========================================================================
    // I/O Pointer Setup
    // ========================================================================
    
    void setStateOutputPointer(FrankaVacuumGripperStateBus* state_ptr);
    void setCommandInputPointer(const FrankaVacuumGripperCommandBus* command_ptr);
    
    // ========================================================================
    // Step Execution
    // ========================================================================
    
    /**
     * @brief Process command triggers and update state output
     * @param vacuum_rising 1.0 if vacuum rising edge detected, 0.0 otherwise
     * @param dropoff_rising 1.0 if drop_off rising edge detected, 0.0 otherwise
     * @param stop_rising 1.0 if stop rising edge detected, 0.0 otherwise
     * @param read_state_rising 1.0 if read_state rising edge detected, 0.0 otherwise
     */
    void step(double vacuum_rising, double dropoff_rising, 
              double stop_rising, double read_state_rising);
    
    /**
     * @brief Force a state read
     */
    void readStateOnce();

private:
    FrankaVacuumGripperInstance* instance_{nullptr};
    
    // Async connection state
    std::thread connection_thread_;
    std::atomic<bool> connection_in_progress_{false};
    std::string pending_robot_ip_;
    
    FrankaVacuumGripperStateBus* state_out_{nullptr};
    const FrankaVacuumGripperCommandBus* command_in_{nullptr};
};

#endif // FRANKA_VACUUM_GRIPPER_API_H
