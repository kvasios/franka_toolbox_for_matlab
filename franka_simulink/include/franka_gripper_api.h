// Copyright (c) 2025 Franka Robotics GmbH
// Franka Gripper API - Header
//
// This provides the interface between Simulink code generation and libfranka
// gripper with async command handling via a dedicated worker thread.
//
// Architecture:
//   FrankaGripperManager    - Global singleton registry (IP → Instance)
//   FrankaGripperInstance   - Shared gripper connection (one per physical gripper)
//   FrankaGripperContext    - Per-block lightweight context (I/O pointers, state polling)
//
// Key Features:
//   - Async command execution (homing, grasp, move, stop) via worker thread
//   - Commands triggered by rising edge signals (like robot S-function triggers)
//   - Thread-safe command queue and status reporting
//   - Periodic state polling for continuous gripper state output

#ifndef FRANKA_GRIPPER_API_H
#define FRANKA_GRIPPER_API_H

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>

#include <franka/exception.h>
#include <franka/gripper.h>

#include "franka_simulink_types.h"

// Forward declarations
class FrankaGripperInstance;
class FrankaGripperContext;

/**
 * @brief Command request structure for the async command queue
 */
struct GripperCommandRequest {
    FrankaGripperCommand command;
    
    // Grasp parameters
    double grasp_width;
    double grasp_speed;
    double grasp_force;
    double grasp_epsilon_inner;
    double grasp_epsilon_outer;
    
    // Move parameters
    double move_width;
    double move_speed;
};

// ============================================================================
// FrankaGripperInstance - Shared Gripper Connection (one per physical gripper)
// ============================================================================

/**
 * @brief Shared gripper instance representing a single physical gripper connection.
 * 
 * This class owns the franka::Gripper object and manages the async command
 * worker thread that executes blocking gripper operations.
 * 
 * Lifecycle:
 *   - Created by FrankaGripperManager::getOrCreate(ip)
 *   - Shared by all blocks targeting the same IP
 *   - Destroyed when FrankaGripperManager shuts down
 * 
 * Thread Model:
 *   - Command thread: Executes blocking gripper commands (homing, grasp, move, stop)
 *   - State polling: Done from Simulink step via readOnce() when idle
 */
class FrankaGripperInstance {
public:
    /**
     * @brief Construct and connect to gripper at given IP
     * @param ip Robot IP address (gripper is connected to robot)
     * @throws franka::Exception if connection fails
     */
    explicit FrankaGripperInstance(const std::string& ip);
    
    ~FrankaGripperInstance();
    
    // Non-copyable, non-movable
    FrankaGripperInstance(const FrankaGripperInstance&) = delete;
    FrankaGripperInstance& operator=(const FrankaGripperInstance&) = delete;
    FrankaGripperInstance(FrankaGripperInstance&&) = delete;
    FrankaGripperInstance& operator=(FrankaGripperInstance&&) = delete;
    
    // ========================================================================
    // Accessors
    // ========================================================================
    
    const std::string& ip() const { return ip_; }
    franka::Gripper& gripper() { return *gripper_; }
    
    // ========================================================================
    // Command Execution (Async)
    // ========================================================================
    
    /**
     * @brief Queue a command for async execution
     * @param request Command request with all parameters
     * 
     * Thread-safe. If a command is already in progress, this replaces
     * the pending command (only one pending command at a time).
     */
    void queueCommand(const GripperCommandRequest& request);
    
    /**
     * @brief Check if a command is currently executing
     */
    bool isCommandInProgress() const { return command_in_progress_.load(); }
    
    /**
     * @brief Get current command status
     */
    FrankaGripperStatus getStatus() const { return static_cast<FrankaGripperStatus>(status_.load()); }
    
    /**
     * @brief Get last executed command
     */
    FrankaGripperCommand getLastCommand() const { return static_cast<FrankaGripperCommand>(last_command_.load()); }
    
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
     * @brief Read gripper state (blocking call)
     * @param state_out Output for gripper state
     * @return true if read succeeded
     * 
     * Can be called while command is in progress (uses separate UDP channel).
     */
    bool readState(FrankaGripperStateBus* state_out);
    
    /**
     * @brief Get cached state with current status (thread-safe)
     * @param state_out Output for state
     */
    void getCachedState(FrankaGripperStateBus* state_out) const;
    
    // ========================================================================
    // Lifecycle
    // ========================================================================
    
    /**
     * @brief Shutdown the command worker thread
     */
    void shutdown();

private:
    void commandWorkerThread();
    void executeCommand(const GripperCommandRequest& request);
    void updateCachedState();
    void copyGripperState(const franka::GripperState& src, FrankaGripperStateBus* dst);
    
    std::string ip_;
    std::unique_ptr<franka::Gripper> gripper_;
    
    // Command worker thread
    std::thread command_thread_;
    std::atomic<bool> shutdown_requested_{false};
    
    // Command queue (single slot - latest command wins)
    std::mutex command_mutex_;
    std::condition_variable command_cv_;
    bool has_pending_command_{false};
    GripperCommandRequest pending_command_;
    
    // Command execution state
    std::atomic<bool> command_in_progress_{false};
    std::atomic<int> status_{FRANKA_GRIPPER_STATUS_IDLE};
    std::atomic<int> last_command_{FRANKA_GRIPPER_CMD_NONE};
    std::atomic<bool> last_command_success_{false};
    std::atomic<int> error_code_{0};
    
    // Cached state
    mutable std::mutex state_mutex_;
    FrankaGripperStateBus cached_state_{};
};

// ============================================================================
// FrankaGripperManager - Global Registry (IP → Instance)
// ============================================================================

/**
 * @brief Global manager for gripper instances, indexed by IP.
 * 
 * This singleton-like class maintains a registry of all gripper connections.
 * Thread-safe for concurrent access from multiple blocks.
 */
class FrankaGripperManager {
public:
    // ========================================================================
    // Instance Management
    // ========================================================================
    
    /**
     * @brief Get or create a gripper instance for the given IP
     * @param ip Robot IP address (gripper connects via robot)
     * @return Pointer to the instance (never null if no exception)
     * @throws franka::Exception if connection fails (for new instance)
     */
    static FrankaGripperInstance* getOrCreate(const std::string& ip);
    
    /**
     * @brief Get existing instance for IP (no creation)
     * @param ip Robot IP address
     * @return Pointer to instance, or nullptr if not connected
     */
    static FrankaGripperInstance* get(const std::string& ip);
    
    /**
     * @brief Check if a gripper is connected
     * @param ip Robot IP address
     */
    static bool isConnected(const std::string& ip);
    
    /**
     * @brief Shutdown and remove all instances
     */
    static void shutdownAll();
    
    // ========================================================================
    // Operations by IP
    // ========================================================================
    
    /**
     * @brief Queue a homing command
     */
    static void homing(const std::string& ip);
    
    /**
     * @brief Queue a grasp command
     */
    static void grasp(const std::string& ip, double width, double speed, double force,
                      double epsilon_inner = 0.005, double epsilon_outer = 0.005);
    
    /**
     * @brief Queue a move command
     */
    static void move(const std::string& ip, double width, double speed);
    
    /**
     * @brief Queue a stop command
     */
    static void stop(const std::string& ip);
    
    /**
     * @brief Read gripper state
     */
    static bool readState(const std::string& ip, FrankaGripperStateBus* state_out);
    
    // Sanitize IP string (trim whitespace, handle embedded nulls)
    static std::string sanitizeIP(const std::string& ip);
    
private:
    static std::unordered_map<std::string, std::unique_ptr<FrankaGripperInstance>> instances_;
    static std::mutex mutex_;
};

// ============================================================================
// FrankaGripperContext - Per-Block Lightweight Context
// ============================================================================

/**
 * @brief Lightweight context for a single Simulink gripper block instance.
 * 
 * This class manages:
 *   - I/O pointer wiring (Simulink signals ↔ gripper data)
 *   - Command trigger edge detection
 *   - Periodic state polling
 * 
 * It does NOT own the gripper connection - it borrows from FrankaGripperInstance
 * via FrankaGripperManager.
 */
class FrankaGripperContext {
public:
    FrankaGripperContext();
    ~FrankaGripperContext();
    
    // Non-copyable, non-movable
    FrankaGripperContext(const FrankaGripperContext&) = delete;
    FrankaGripperContext& operator=(const FrankaGripperContext&) = delete;
    FrankaGripperContext(FrankaGripperContext&&) = delete;
    FrankaGripperContext& operator=(FrankaGripperContext&&) = delete;
    
    // ========================================================================
    // Lifecycle
    // ========================================================================
    
    /**
     * @brief Initialize by connecting to gripper at IP (blocking)
     * @param robot_ip IP address of the Franka robot
     */
    void initialize(const std::string& robot_ip);
    
    /**
     * @brief Initialize by connecting to gripper at IP (non-blocking)
     * @param robot_ip IP address of the Franka robot
     * 
     * Spawns a background thread to perform the connection.
     * Check isConnecting() and isInitialized() to track progress.
     */
    void initializeAsync(const std::string& robot_ip);
    
    /**
     * @brief Check if initialized (connected)
     */
    bool isInitialized() const { return instance_ != nullptr; }
    
    /**
     * @brief Check if async connection is in progress
     */
    bool isConnecting() const { return connection_in_progress_.load(); }
    
    /**
     * @brief Shutdown and cleanup
     */
    void shutdown();
    
    // ========================================================================
    // I/O Pointer Setup
    // ========================================================================
    
    void setStateOutputPointer(FrankaGripperStateBus* state_ptr);
    void setCommandInputPointer(const FrankaGripperCommandBus* command_ptr);
    
    // ========================================================================
    // Step Execution
    // ========================================================================
    
    /**
     * @brief Process command triggers and update state output
     * @param homing_rising 1.0 if homing rising edge detected, 0.0 otherwise
     * @param grasp_rising 1.0 if grasp rising edge detected, 0.0 otherwise
     * @param move_rising 1.0 if move rising edge detected, 0.0 otherwise
     * @param stop_rising 1.0 if stop rising edge detected, 0.0 otherwise
     * @param read_state_rising 1.0 if read_state rising edge detected, 0.0 otherwise
     * 
     * Edge detection is done by the caller (TLC-generated code using DWork).
     * This method queues commands on edges and updates the state output.
     */
    void step(double homing_rising, double grasp_rising, double move_rising,
              double stop_rising, double read_state_rising);
    
    /**
     * @brief Force a state read (for read_once functionality)
     */
    void readStateOnce();

private:
    // Shared gripper instance (borrowed from manager, not owned)
    FrankaGripperInstance* instance_{nullptr};
    
    // Async connection state
    std::thread connection_thread_;
    std::atomic<bool> connection_in_progress_{false};
    std::string pending_robot_ip_;
    
    // Pointers to Simulink signals
    FrankaGripperStateBus* state_out_{nullptr};
    const FrankaGripperCommandBus* command_in_{nullptr};
};

#endif // FRANKA_GRIPPER_API_H
