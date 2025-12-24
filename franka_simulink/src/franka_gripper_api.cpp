// Copyright (c) 2025 Franka Robotics GmbH
// Franka Gripper API - Implementation
//
// Provides async command handling for the Franka gripper via a dedicated
// worker thread. Commands are queued and executed sequentially while
// state can be polled independently.

#include "franka_gripper_api.h"

#include <cstring>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <cctype>

// ============================================================================
// FrankaGripperInstance Implementation
// ============================================================================

FrankaGripperInstance::FrankaGripperInstance(const std::string& ip)
    : ip_(ip)
{
    // Connect to gripper (uses robot's IP)
    gripper_ = std::make_unique<franka::Gripper>(ip);
    
    // Initialize cached state
    std::memset(&cached_state_, 0, sizeof(cached_state_));
    
    // Start command worker thread
    command_thread_ = std::thread(&FrankaGripperInstance::commandWorkerThread, this);
    
    std::cout << "FrankaGripperInstance: Connected to gripper at " << ip << std::endl;
}

FrankaGripperInstance::~FrankaGripperInstance()
{
    shutdown();
}

void FrankaGripperInstance::shutdown()
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
    
    std::cout << "FrankaGripperInstance: Shutdown complete for " << ip_ << std::endl;
}

void FrankaGripperInstance::queueCommand(const GripperCommandRequest& request)
{
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_ = request;
        has_pending_command_ = true;
    }
    command_cv_.notify_one();
}

bool FrankaGripperInstance::readState(FrankaGripperStateBus* state_out)
{
    try {
        franka::GripperState state = gripper_->readOnce();
        
        // Update cached state
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            copyGripperState(state, &cached_state_);
            
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
        std::cerr << "FrankaGripperInstance::readState: " << e.what() << std::endl;
        return false;
    }
}

void FrankaGripperInstance::commandWorkerThread()
{
    while (!shutdown_requested_.load()) {
        GripperCommandRequest request;
        
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
        status_.store(FRANKA_GRIPPER_STATUS_BUSY);
        
        executeCommand(request);
        
        command_in_progress_.store(false);
        
        // Update state after command completes
        updateCachedState();
    }
}

void FrankaGripperInstance::executeCommand(const GripperCommandRequest& request)
{
    last_command_.store(static_cast<int>(request.command));
    
    try {
        bool result = false;
        
        switch (request.command) {
            case FRANKA_GRIPPER_CMD_HOMING:
                std::cout << "FrankaGripper: Executing homing..." << std::endl;
                result = gripper_->homing();
                break;
                
            case FRANKA_GRIPPER_CMD_GRASP:
                std::cout << "FrankaGripper: Executing grasp (width=" << request.grasp_width
                          << ", speed=" << request.grasp_speed
                          << ", force=" << request.grasp_force << ")..." << std::endl;
                result = gripper_->grasp(
                    request.grasp_width,
                    request.grasp_speed,
                    request.grasp_force,
                    request.grasp_epsilon_inner,
                    request.grasp_epsilon_outer
                );
                break;
                
            case FRANKA_GRIPPER_CMD_MOVE:
                std::cout << "FrankaGripper: Executing move (width=" << request.move_width
                          << ", speed=" << request.move_speed << ")..." << std::endl;
                result = gripper_->move(request.move_width, request.move_speed);
                break;
                
            case FRANKA_GRIPPER_CMD_STOP:
                std::cout << "FrankaGripper: Executing stop..." << std::endl;
                result = gripper_->stop();
                break;
                
            default:
                std::cerr << "FrankaGripper: Unknown command: " << static_cast<int>(request.command) << std::endl;
                status_.store(FRANKA_GRIPPER_STATUS_ERROR);
                error_code_.store(-1);
                return;
        }
        
        last_command_success_.store(result);
        
        if (result) {
            status_.store(FRANKA_GRIPPER_STATUS_SUCCESS);
            error_code_.store(0);
            std::cout << "FrankaGripper: Command succeeded" << std::endl;
        } else {
            status_.store(FRANKA_GRIPPER_STATUS_FAILED);
            error_code_.store(1);  // Generic failure
            std::cout << "FrankaGripper: Command returned false (unsuccessful)" << std::endl;
        }
        
    } catch (const franka::CommandException& e) {
        std::cerr << "FrankaGripper: CommandException: " << e.what() << std::endl;
        last_command_success_.store(false);
        status_.store(FRANKA_GRIPPER_STATUS_ERROR);
        error_code_.store(2);  // Command exception
    } catch (const franka::NetworkException& e) {
        std::cerr << "FrankaGripper: NetworkException: " << e.what() << std::endl;
        last_command_success_.store(false);
        status_.store(FRANKA_GRIPPER_STATUS_ERROR);
        error_code_.store(3);  // Network exception
    } catch (const franka::Exception& e) {
        std::cerr << "FrankaGripper: Exception: " << e.what() << std::endl;
        last_command_success_.store(false);
        status_.store(FRANKA_GRIPPER_STATUS_ERROR);
        error_code_.store(4);  // Generic libfranka exception
    } catch (const std::exception& e) {
        std::cerr << "FrankaGripper: std::exception: " << e.what() << std::endl;
        last_command_success_.store(false);
        status_.store(FRANKA_GRIPPER_STATUS_ERROR);
        error_code_.store(5);  // Other exception
    }
}

void FrankaGripperInstance::updateCachedState()
{
    try {
        franka::GripperState state = gripper_->readOnce();
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        copyGripperState(state, &cached_state_);
        
        // Update status fields
        cached_state_.command_status = status_.load();
        cached_state_.last_command = last_command_.load();
        cached_state_.command_success = last_command_success_.load() ? 1 : 0;
        cached_state_.error_code = error_code_.load();
        
    } catch (const franka::Exception& e) {
        std::cerr << "FrankaGripperInstance::updateCachedState: " << e.what() << std::endl;
    }
}

void FrankaGripperInstance::copyGripperState(const franka::GripperState& src, FrankaGripperStateBus* dst)
{
    dst->width = src.width;
    dst->max_width = src.max_width;
    dst->is_grasped = src.is_grasped ? 1.0 : 0.0;
    dst->temperature = static_cast<double>(src.temperature);
    dst->time = static_cast<double>(src.time.toMSec()) / 1000.0;  // Convert to seconds
}

void FrankaGripperInstance::getCachedState(FrankaGripperStateBus* state_out) const
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

// ============================================================================
// FrankaGripperManager Implementation
// ============================================================================

std::unordered_map<std::string, std::unique_ptr<FrankaGripperInstance>> FrankaGripperManager::instances_;
std::mutex FrankaGripperManager::mutex_;

std::string FrankaGripperManager::sanitizeIP(const std::string& ip)
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

FrankaGripperInstance* FrankaGripperManager::getOrCreate(const std::string& ip)
{
    std::string clean_ip = sanitizeIP(ip);
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = instances_.find(clean_ip);
    if (it != instances_.end()) {
        return it->second.get();
    }
    
    // Create new instance
    auto instance = std::make_unique<FrankaGripperInstance>(clean_ip);
    FrankaGripperInstance* ptr = instance.get();
    instances_[clean_ip] = std::move(instance);
    
    return ptr;
}

FrankaGripperInstance* FrankaGripperManager::get(const std::string& ip)
{
    std::string clean_ip = sanitizeIP(ip);
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = instances_.find(clean_ip);
    if (it != instances_.end()) {
        return it->second.get();
    }
    
    return nullptr;
}

bool FrankaGripperManager::isConnected(const std::string& ip)
{
    return get(ip) != nullptr;
}

void FrankaGripperManager::shutdownAll()
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (auto& pair : instances_) {
        pair.second->shutdown();
    }
    
    instances_.clear();
    
    std::cout << "FrankaGripperManager: All instances shutdown" << std::endl;
}

void FrankaGripperManager::homing(const std::string& ip)
{
    auto* instance = get(ip);
    if (instance) {
        GripperCommandRequest request{};
        request.command = FRANKA_GRIPPER_CMD_HOMING;
        instance->queueCommand(request);
    }
}

void FrankaGripperManager::grasp(const std::string& ip, double width, double speed, double force,
                                  double epsilon_inner, double epsilon_outer)
{
    auto* instance = get(ip);
    if (instance) {
        GripperCommandRequest request{};
        request.command = FRANKA_GRIPPER_CMD_GRASP;
        request.grasp_width = width;
        request.grasp_speed = speed;
        request.grasp_force = force;
        request.grasp_epsilon_inner = epsilon_inner;
        request.grasp_epsilon_outer = epsilon_outer;
        instance->queueCommand(request);
    }
}

void FrankaGripperManager::move(const std::string& ip, double width, double speed)
{
    auto* instance = get(ip);
    if (instance) {
        GripperCommandRequest request{};
        request.command = FRANKA_GRIPPER_CMD_MOVE;
        request.move_width = width;
        request.move_speed = speed;
        instance->queueCommand(request);
    }
}

void FrankaGripperManager::stop(const std::string& ip)
{
    auto* instance = get(ip);
    if (instance) {
        GripperCommandRequest request{};
        request.command = FRANKA_GRIPPER_CMD_STOP;
        instance->queueCommand(request);
    }
}

bool FrankaGripperManager::readState(const std::string& ip, FrankaGripperStateBus* state_out)
{
    auto* instance = get(ip);
    if (instance) {
        return instance->readState(state_out);
    }
    return false;
}

// ============================================================================
// FrankaGripperContext Implementation
// ============================================================================

FrankaGripperContext::FrankaGripperContext()
{
}

FrankaGripperContext::~FrankaGripperContext()
{
    shutdown();
}

void FrankaGripperContext::initialize(const std::string& robot_ip)
{
    if (instance_) {
        return;  // Already initialized
    }
    
    instance_ = FrankaGripperManager::getOrCreate(robot_ip);
}

void FrankaGripperContext::initializeAsync(const std::string& robot_ip)
{
    // Already connected?
    if (instance_) {
        return;
    }
    
    // Already connecting?
    if (connection_in_progress_.load()) {
        // Check if it's the same IP
        if (FrankaGripperManager::sanitizeIP(robot_ip) == 
            FrankaGripperManager::sanitizeIP(pending_robot_ip_)) {
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
            FrankaGripperInstance* inst = FrankaGripperManager::getOrCreate(ip);
            instance_ = inst;
            std::cout << "FrankaGripperContext: Async connection to " << ip << " succeeded" << std::endl;
        } catch (const franka::Exception& e) {
            std::cerr << "FrankaGripperContext: Async connection failed: " << e.what() << std::endl;
        }
        connection_in_progress_.store(false);
    });
}

void FrankaGripperContext::shutdown()
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

void FrankaGripperContext::setStateOutputPointer(FrankaGripperStateBus* state_ptr)
{
    state_out_ = state_ptr;
}

void FrankaGripperContext::setCommandInputPointer(const FrankaGripperCommandBus* command_ptr)
{
    command_in_ = command_ptr;
}

void FrankaGripperContext::step(double homing_rising, double grasp_rising, double move_rising,
                                 double stop_rising, double read_state_rising)
{
    if (!instance_) {
        return;
    }
    
    // Edge detection is done by the caller (TLC code using DWork)
    // Values > 0.5 indicate a rising edge was detected
    bool do_stop = stop_rising > 0.5;
    bool do_homing = homing_rising > 0.5;
    bool do_grasp = grasp_rising > 0.5;
    bool do_move = move_rising > 0.5;
    bool do_read_state = read_state_rising > 0.5;
    
    // Queue commands on rising edges (priority: stop > homing > grasp > move)
    if (do_stop) {
        GripperCommandRequest request{};
        request.command = FRANKA_GRIPPER_CMD_STOP;
        instance_->queueCommand(request);
    } else if (do_homing) {
        GripperCommandRequest request{};
        request.command = FRANKA_GRIPPER_CMD_HOMING;
        instance_->queueCommand(request);
    } else if (do_grasp && command_in_) {
        GripperCommandRequest request{};
        request.command = FRANKA_GRIPPER_CMD_GRASP;
        request.grasp_width = command_in_->grasp_width;
        request.grasp_speed = command_in_->grasp_speed;
        request.grasp_force = command_in_->grasp_force;
        request.grasp_epsilon_inner = command_in_->grasp_epsilon_inner;
        request.grasp_epsilon_outer = command_in_->grasp_epsilon_outer;
        instance_->queueCommand(request);
    } else if (do_move && command_in_) {
        GripperCommandRequest request{};
        request.command = FRANKA_GRIPPER_CMD_MOVE;
        request.move_width = command_in_->move_width;
        request.move_speed = command_in_->move_speed;
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

void FrankaGripperContext::readStateOnce()
{
    if (instance_ && state_out_) {
        instance_->readState(state_out_);
    }
}
