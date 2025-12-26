// Copyright (c) 2025 Franka Robotics GmbH
// Franka Vacuum Gripper API - Implementation
//
// Provides async command handling for the Franka vacuum gripper via a dedicated
// worker thread. Commands are queued and executed sequentially while
// state can be polled independently.

#include "franka_vacuum_gripper_api.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <iostream>

// ============================================================================
// FrankaVacuumGripperInstance Implementation
// ============================================================================

FrankaVacuumGripperInstance::FrankaVacuumGripperInstance(const std::string& ip)
    : ip_(ip)
{
    // Connect to vacuum gripper (uses robot's IP)
    gripper_ = std::make_unique<franka::VacuumGripper>(ip);
    
    // Initialize cached state
    std::memset(&cached_state_, 0, sizeof(cached_state_));
    
    // Start command worker thread
    command_thread_ = std::thread(&FrankaVacuumGripperInstance::commandWorkerThread, this);
    
    std::cout << "FrankaVacuumGripperInstance: Connected to vacuum gripper at " << ip << std::endl;
}

FrankaVacuumGripperInstance::~FrankaVacuumGripperInstance()
{
    shutdown();
}

void FrankaVacuumGripperInstance::shutdown()
{
    // Signal shutdown
    shutdown_requested_.store(true);
    
    // Wake up command thread
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_cv_.notify_all();
    }
    
    // Wait for thread to finish
    if (command_thread_.joinable()) {
        command_thread_.join();
    }
    
    std::cout << "FrankaVacuumGripperInstance: Shutdown complete for " << ip_ << std::endl;
}

void FrankaVacuumGripperInstance::queueCommand(const VacuumGripperCommandRequest& request)
{
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_ = request;
        has_pending_command_ = true;
    }
    command_cv_.notify_one();
}

bool FrankaVacuumGripperInstance::stopImmediate()
{
    // Call stop() directly - this is threadsafe per libfranka docs and will
    // interrupt any currently running vacuum/dropOff command.
    // The blocked command in the worker thread will return false or throw.
    try {
        std::cout << "FrankaVacuumGripper: Executing IMMEDIATE stop (interrupting current command)..." << std::endl;
        bool result = gripper_->stop();
        
        // Update status atomically
        last_command_.store(static_cast<int>(FRANKA_VACUUM_CMD_STOP));
        last_command_success_.store(result);
        
        if (result) {
            status_.store(FRANKA_VACUUM_STATUS_SUCCESS);
            error_code_.store(0);
            std::cout << "FrankaVacuumGripper: Immediate stop succeeded" << std::endl;
        } else {
            status_.store(FRANKA_VACUUM_STATUS_FAILED);
            error_code_.store(1);
            std::cout << "FrankaVacuumGripper: Immediate stop returned false" << std::endl;
        }
        
        return result;
        
    } catch (const franka::CommandException& e) {
        std::cerr << "FrankaVacuumGripper: stopImmediate CommandException: " << e.what() << std::endl;
        last_command_.store(static_cast<int>(FRANKA_VACUUM_CMD_STOP));
        last_command_success_.store(false);
        status_.store(FRANKA_VACUUM_STATUS_ERROR);
        error_code_.store(2);
        return false;
    } catch (const franka::NetworkException& e) {
        std::cerr << "FrankaVacuumGripper: stopImmediate NetworkException: " << e.what() << std::endl;
        last_command_.store(static_cast<int>(FRANKA_VACUUM_CMD_STOP));
        last_command_success_.store(false);
        status_.store(FRANKA_VACUUM_STATUS_ERROR);
        error_code_.store(3);
        return false;
    } catch (const franka::Exception& e) {
        std::cerr << "FrankaVacuumGripper: stopImmediate Exception: " << e.what() << std::endl;
        last_command_.store(static_cast<int>(FRANKA_VACUUM_CMD_STOP));
        last_command_success_.store(false);
        status_.store(FRANKA_VACUUM_STATUS_ERROR);
        error_code_.store(4);
        return false;
    }
}

bool FrankaVacuumGripperInstance::readState(FrankaVacuumGripperStateBus* state_out)
{
    try {
        franka::VacuumGripperState state = gripper_->readOnce();
        
        // Update cached state
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            copyVacuumGripperState(state, &cached_state_);
            
            // Also update status fields
            cached_state_.command_status = status_.load();
            cached_state_.last_command = last_command_.load();
            cached_state_.command_success = last_command_success_.load() ? 1 : 0;
            cached_state_.error_code = error_code_.load();
        }
        
        // Copy to output if provided
        if (state_out) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            *state_out = cached_state_;
        }
        
        return true;
    } catch (const franka::Exception& e) {
        std::cerr << "FrankaVacuumGripperInstance::readState: " << e.what() << std::endl;
        return false;
    }
}

void FrankaVacuumGripperInstance::getCachedState(FrankaVacuumGripperStateBus* state_out) const
{
    if (!state_out) return;
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    *state_out = cached_state_;
    
    // Update status fields (these change without explicit readState)
    state_out->command_status = status_.load();
    state_out->last_command = last_command_.load();
    state_out->command_success = last_command_success_.load() ? 1 : 0;
    state_out->error_code = error_code_.load();
}

void FrankaVacuumGripperInstance::commandWorkerThread()
{
    while (!shutdown_requested_.load()) {
        VacuumGripperCommandRequest request;
        
        // Wait for command
        {
            std::unique_lock<std::mutex> lock(command_mutex_);
            command_cv_.wait(lock, [this] {
                return has_pending_command_ || shutdown_requested_.load();
            });
            
            if (shutdown_requested_.load()) {
                break;
            }
            
            if (!has_pending_command_) {
                continue;
            }
            
            request = pending_command_;
            has_pending_command_ = false;
        }
        
        // Execute command
        command_in_progress_.store(true);
        status_.store(FRANKA_VACUUM_STATUS_BUSY);
        
        executeCommand(request);
        
        command_in_progress_.store(false);
        
        // Update state after command completes
        updateCachedState();
    }
}

void FrankaVacuumGripperInstance::executeCommand(const VacuumGripperCommandRequest& request)
{
    last_command_.store(static_cast<int>(request.command));
    
    try {
        bool result = false;
        
        switch (request.command) {
            case FRANKA_VACUUM_CMD_VACUUM:
                std::cout << "FrankaVacuumGripper: Executing vacuum (setpoint=" 
                          << static_cast<int>(request.vacuum_setpoint)
                          << " [10*mbar], timeout=" << request.vacuum_timeout.count() 
                          << "ms, profile=P" << static_cast<int>(request.vacuum_profile) 
                          << ")..." << std::endl;
                result = gripper_->vacuum(
                    request.vacuum_setpoint,
                    request.vacuum_timeout,
                    request.vacuum_profile
                );
                break;
                
            case FRANKA_VACUUM_CMD_DROP_OFF:
                std::cout << "FrankaVacuumGripper: Executing dropOff (timeout=" 
                          << request.dropoff_timeout.count() << "ms)..." << std::endl;
                result = gripper_->dropOff(request.dropoff_timeout);
                break;
                
            case FRANKA_VACUUM_CMD_STOP:
                std::cout << "FrankaVacuumGripper: Executing stop..." << std::endl;
                result = gripper_->stop();
                break;
                
            default:
                std::cerr << "FrankaVacuumGripper: Unknown command: " 
                          << static_cast<int>(request.command) << std::endl;
                status_.store(FRANKA_VACUUM_STATUS_ERROR);
                error_code_.store(-1);
                return;
        }
        
        last_command_success_.store(result);
        
        if (result) {
            status_.store(FRANKA_VACUUM_STATUS_SUCCESS);
            error_code_.store(0);
            std::cout << "FrankaVacuumGripper: Command succeeded" << std::endl;
        } else {
            status_.store(FRANKA_VACUUM_STATUS_FAILED);
            error_code_.store(1);
            std::cout << "FrankaVacuumGripper: Command returned false (unsuccessful)" << std::endl;
        }
        
    } catch (const franka::CommandException& e) {
        std::cerr << "FrankaVacuumGripper: CommandException: " << e.what() << std::endl;
        last_command_success_.store(false);
        status_.store(FRANKA_VACUUM_STATUS_ERROR);
        error_code_.store(2);
    } catch (const franka::NetworkException& e) {
        std::cerr << "FrankaVacuumGripper: NetworkException: " << e.what() << std::endl;
        last_command_success_.store(false);
        status_.store(FRANKA_VACUUM_STATUS_ERROR);
        error_code_.store(3);
    } catch (const franka::Exception& e) {
        std::cerr << "FrankaVacuumGripper: Exception: " << e.what() << std::endl;
        last_command_success_.store(false);
        status_.store(FRANKA_VACUUM_STATUS_ERROR);
        error_code_.store(4);
    } catch (const std::exception& e) {
        std::cerr << "FrankaVacuumGripper: std::exception: " << e.what() << std::endl;
        last_command_success_.store(false);
        status_.store(FRANKA_VACUUM_STATUS_ERROR);
        error_code_.store(5);
    }
}

void FrankaVacuumGripperInstance::updateCachedState()
{
    try {
        franka::VacuumGripperState state = gripper_->readOnce();
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        copyVacuumGripperState(state, &cached_state_);
        
        // Update status fields
        cached_state_.command_status = status_.load();
        cached_state_.last_command = last_command_.load();
        cached_state_.command_success = last_command_success_.load() ? 1 : 0;
        cached_state_.error_code = error_code_.load();
        
    } catch (const franka::Exception& e) {
        std::cerr << "FrankaVacuumGripperInstance::updateCachedState: " << e.what() << std::endl;
    }
}

void FrankaVacuumGripperInstance::copyVacuumGripperState(
    const franka::VacuumGripperState& src, FrankaVacuumGripperStateBus* dst)
{
    dst->in_control_range = src.in_control_range ? 1.0 : 0.0;
    dst->part_detached = src.part_detached ? 1.0 : 0.0;
    dst->part_present = src.part_present ? 1.0 : 0.0;
    
    // Convert device status enum
    switch (src.device_status) {
        case franka::VacuumGripperDeviceStatus::kGreen:
            dst->device_status = FRANKA_VACUUM_DEVICE_GREEN;
            break;
        case franka::VacuumGripperDeviceStatus::kYellow:
            dst->device_status = FRANKA_VACUUM_DEVICE_YELLOW;
            break;
        case franka::VacuumGripperDeviceStatus::kOrange:
            dst->device_status = FRANKA_VACUUM_DEVICE_ORANGE;
            break;
        case franka::VacuumGripperDeviceStatus::kRed:
            dst->device_status = FRANKA_VACUUM_DEVICE_RED;
            break;
    }
    
    dst->actual_power = static_cast<double>(src.actual_power);
    dst->vacuum = static_cast<double>(src.vacuum);
    dst->time = static_cast<double>(src.time.toMSec()) / 1000.0;  // Convert to seconds
}

// ============================================================================
// FrankaVacuumGripperManager Implementation
// ============================================================================

std::unordered_map<std::string, std::unique_ptr<FrankaVacuumGripperInstance>> FrankaVacuumGripperManager::instances_;
std::mutex FrankaVacuumGripperManager::mutex_;

std::string FrankaVacuumGripperManager::sanitizeIP(const std::string& ip)
{
    // Find null terminator if embedded
    size_t len = ip.size();
    for (size_t i = 0; i < ip.size(); ++i) {
        if (ip[i] == '\0') {
            len = i;
            break;
        }
    }
    
    std::string result = ip.substr(0, len);
    
    // Trim leading whitespace
    auto start = std::find_if_not(result.begin(), result.end(), [](unsigned char c) {
        return std::isspace(c);
    });
    
    // Trim trailing whitespace
    auto end = std::find_if_not(result.rbegin(), result.rend(), [](unsigned char c) {
        return std::isspace(c);
    }).base();
    
    if (start >= end) {
        return "";
    }
    
    return std::string(start, end);
}

FrankaVacuumGripperInstance* FrankaVacuumGripperManager::getOrCreate(const std::string& ip)
{
    std::string clean_ip = sanitizeIP(ip);
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = instances_.find(clean_ip);
    if (it != instances_.end()) {
        return it->second.get();
    }
    
    // Create new instance
    auto instance = std::make_unique<FrankaVacuumGripperInstance>(clean_ip);
    FrankaVacuumGripperInstance* ptr = instance.get();
    instances_[clean_ip] = std::move(instance);
    
    return ptr;
}

FrankaVacuumGripperInstance* FrankaVacuumGripperManager::get(const std::string& ip)
{
    std::string clean_ip = sanitizeIP(ip);
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = instances_.find(clean_ip);
    if (it != instances_.end()) {
        return it->second.get();
    }
    
    return nullptr;
}

bool FrankaVacuumGripperManager::isConnected(const std::string& ip)
{
    return get(ip) != nullptr;
}

void FrankaVacuumGripperManager::shutdownAll()
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (auto& pair : instances_) {
        pair.second->shutdown();
    }
    
    instances_.clear();
    
    std::cout << "FrankaVacuumGripperManager: All instances shutdown" << std::endl;
}

void FrankaVacuumGripperManager::vacuum(const std::string& ip, uint8_t setpoint,
                                         std::chrono::milliseconds timeout,
                                         franka::VacuumGripper::ProductionSetupProfile profile)
{
    auto* instance = get(ip);
    if (instance) {
        VacuumGripperCommandRequest request{};
        request.command = FRANKA_VACUUM_CMD_VACUUM;
        request.vacuum_setpoint = setpoint;
        request.vacuum_timeout = timeout;
        request.vacuum_profile = profile;
        instance->queueCommand(request);
    }
}

void FrankaVacuumGripperManager::dropOff(const std::string& ip, std::chrono::milliseconds timeout)
{
    auto* instance = get(ip);
    if (instance) {
        VacuumGripperCommandRequest request{};
        request.command = FRANKA_VACUUM_CMD_DROP_OFF;
        request.dropoff_timeout = timeout;
        instance->queueCommand(request);
    }
}

void FrankaVacuumGripperManager::stop(const std::string& ip)
{
    auto* instance = get(ip);
    if (instance) {
        // Use immediate stop to interrupt any running command
        // This is threadsafe per libfranka docs
        instance->stopImmediate();
    }
}

bool FrankaVacuumGripperManager::readState(const std::string& ip, FrankaVacuumGripperStateBus* state_out)
{
    auto* instance = get(ip);
    if (instance) {
        return instance->readState(state_out);
    }
    return false;
}

// ============================================================================
// FrankaVacuumGripperContext Implementation
// ============================================================================

FrankaVacuumGripperContext::FrankaVacuumGripperContext()
{
}

FrankaVacuumGripperContext::~FrankaVacuumGripperContext()
{
    shutdown();
}

void FrankaVacuumGripperContext::initialize(const std::string& robot_ip)
{
    if (instance_) {
        return;  // Already initialized
    }
    
    instance_ = FrankaVacuumGripperManager::getOrCreate(robot_ip);
}

void FrankaVacuumGripperContext::initializeAsync(const std::string& robot_ip)
{
    // Already connected?
    if (instance_) {
        return;
    }
    
    // Already connecting?
    if (connection_in_progress_.load()) {
        // Check if it's the same IP
        if (FrankaVacuumGripperManager::sanitizeIP(robot_ip) == 
            FrankaVacuumGripperManager::sanitizeIP(pending_robot_ip_)) {
            return;  // Same connection already in progress
        }
        return;  // Different IP requested while connecting - wait
    }
    
    // Store the IP and mark as connecting
    pending_robot_ip_ = robot_ip;
    connection_in_progress_.store(true);
    
    // Join any previous connection thread
    if (connection_thread_.joinable()) {
        connection_thread_.join();
    }
    
    // Spawn connection thread
    connection_thread_ = std::thread([this, ip = robot_ip]() {
        try {
            FrankaVacuumGripperInstance* inst = FrankaVacuumGripperManager::getOrCreate(ip);
            instance_ = inst;
            std::cout << "FrankaVacuumGripperContext: Async connection to " << ip << " succeeded" << std::endl;
        } catch (const franka::Exception& e) {
            std::cerr << "FrankaVacuumGripperContext: Async connection failed: " << e.what() << std::endl;
        }
        connection_in_progress_.store(false);
    });
}

void FrankaVacuumGripperContext::shutdown()
{
    // Wait for any pending async connection
    if (connection_thread_.joinable()) {
        connection_thread_.join();
    }
    connection_in_progress_.store(false);
    pending_robot_ip_.clear();
    
    // Context doesn't own the instance, so just clear the pointer
    instance_ = nullptr;
}

void FrankaVacuumGripperContext::setStateOutputPointer(FrankaVacuumGripperStateBus* state_ptr)
{
    state_out_ = state_ptr;
}

void FrankaVacuumGripperContext::setCommandInputPointer(const FrankaVacuumGripperCommandBus* command_ptr)
{
    command_in_ = command_ptr;
}

void FrankaVacuumGripperContext::step(double vacuum_rising, double dropoff_rising,
                                       double stop_rising, double read_state_rising)
{
    if (!instance_) {
        return;
    }
    
    // Edge detection is done by the caller (TLC code using DWork)
    bool do_vacuum = vacuum_rising > 0.5;
    bool do_dropoff = dropoff_rising > 0.5;
    bool do_stop = stop_rising > 0.5;
    bool do_read_state = read_state_rising > 0.5;
    
    // Process commands on rising edges
    // STOP is special: it calls stopImmediate() directly to interrupt running commands
    // Other commands are queued for the worker thread
    if (do_stop) {
        // IMMEDIATE STOP - interrupts any running command!
        // This is the key fix: stop() is called directly (not queued) because
        // franka::VacuumGripper is threadsafe. This allows stopping vacuum/dropOff in progress.
        instance_->stopImmediate();
    } else if (do_vacuum && command_in_) {
        VacuumGripperCommandRequest request{};
        request.command = FRANKA_VACUUM_CMD_VACUUM;
        request.vacuum_setpoint = static_cast<uint8_t>(command_in_->vacuum_setpoint);
        request.vacuum_timeout = std::chrono::milliseconds(
            static_cast<int64_t>(command_in_->vacuum_timeout));
        
        // Convert profile enum
        switch (command_in_->vacuum_profile) {
            case FRANKA_VACUUM_PROFILE_P0:
                request.vacuum_profile = franka::VacuumGripper::ProductionSetupProfile::kP0;
                break;
            case FRANKA_VACUUM_PROFILE_P1:
                request.vacuum_profile = franka::VacuumGripper::ProductionSetupProfile::kP1;
                break;
            case FRANKA_VACUUM_PROFILE_P2:
                request.vacuum_profile = franka::VacuumGripper::ProductionSetupProfile::kP2;
                break;
            case FRANKA_VACUUM_PROFILE_P3:
                request.vacuum_profile = franka::VacuumGripper::ProductionSetupProfile::kP3;
                break;
            default:
                request.vacuum_profile = franka::VacuumGripper::ProductionSetupProfile::kP0;
                break;
        }
        
        instance_->queueCommand(request);
    } else if (do_dropoff && command_in_) {
        VacuumGripperCommandRequest request{};
        request.command = FRANKA_VACUUM_CMD_DROP_OFF;
        request.dropoff_timeout = std::chrono::milliseconds(
            static_cast<int64_t>(command_in_->dropoff_timeout));
        instance_->queueCommand(request);
    }
    
    // Read state if triggered or update from cache
    if (do_read_state) {
        readStateOnce();
    } else if (state_out_) {
        // Copy cached state to output (thread-safe)
        instance_->getCachedState(state_out_);
    }
}

void FrankaVacuumGripperContext::readStateOnce()
{
    if (instance_ && state_out_) {
        instance_->readState(state_out_);
    }
}
