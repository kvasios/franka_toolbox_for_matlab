// Copyright (c) 2025 Franka Robotics GmbH
// Franka Robot API - Implementation

#include "franka_robot_api.h"

#include <cerrno>
#include <cmath>
#include <iostream>
#include <stdexcept>

// Real-time thread configuration for PREEMPT_RT
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

// ============================================================================
// FrankaRobotManager - Static Members
// ============================================================================

std::unordered_map<std::string, std::unique_ptr<FrankaRobotInstance>> FrankaRobotManager::instances_;
std::mutex FrankaRobotManager::mutex_;

// ============================================================================
// Helper: Sanitize Array Values (guard against NaN/Inf)
// ============================================================================

template<size_t N>
static void sanitizeArray(std::array<double, N>& arr, const char* name) {
    bool ok = true;
    for (double v : arr) {
        if (!std::isfinite(v)) {
            ok = false;
            break;
        }
    }
    if (!ok) {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) {
            std::cerr << "FrankaRobotAPI: non-finite " << name << " detected; "
                         "zeroing values. Check Simulink signal wiring / controller execution."
                      << std::endl;
        }
        arr.fill(0.0);
    }
}

// ============================================================================
// Helper: Validate 4x4 Homogeneous Transformation Matrix
// ============================================================================
// 
// libfranka expects the transformation matrix to be:
//   - Column-major: [R11 R21 R31 0 | R12 R22 R32 0 | R13 R23 R33 0 | tx ty tz 1]
//   - Valid rotation matrix: orthonormal, determinant ≈ 1
//   - Homogeneous row: [0, 0, 0, 1]
//
// Returns true if the matrix is valid; false if it's invalid (zeros, NaN, etc.)
//
static bool isValidTransformationMatrix(const std::array<double, 16>& T) {
    // Check for non-finite values
    for (double v : T) {
        if (!std::isfinite(v)) {
            return false;
        }
    }
    
    // Check homogeneous row: indices 3, 7, 11 should be 0; index 15 should be 1
    // Column-major layout: T[col*4 + row]
    //   Row 3 (homogeneous): T[3], T[7], T[11], T[15]
    const double eps = 1e-6;
    if (std::abs(T[3]) > eps || std::abs(T[7]) > eps || std::abs(T[11]) > eps) {
        return false;
    }
    if (std::abs(T[15] - 1.0) > eps) {
        return false;
    }
    
    // Extract rotation matrix columns (column-major)
    // Column 0: T[0], T[1], T[2]
    // Column 1: T[4], T[5], T[6]
    // Column 2: T[8], T[9], T[10]
    double r00 = T[0], r10 = T[1], r20 = T[2];
    double r01 = T[4], r11 = T[5], r21 = T[6];
    double r02 = T[8], r12 = T[9], r22 = T[10];
    
    // Check if rotation columns have non-zero length (not all zeros)
    double col0_len_sq = r00*r00 + r10*r10 + r20*r20;
    double col1_len_sq = r01*r01 + r11*r11 + r21*r21;
    double col2_len_sq = r02*r02 + r12*r12 + r22*r22;
    
    if (col0_len_sq < 0.5 || col1_len_sq < 0.5 || col2_len_sq < 0.5) {
        // Columns are too short - likely zeros or invalid
        return false;
    }
    
    // Optional: Check orthonormality (det ≈ 1)
    // For performance, we just check column lengths are close to 1
    if (std::abs(col0_len_sq - 1.0) > 0.1 ||
        std::abs(col1_len_sq - 1.0) > 0.1 ||
        std::abs(col2_len_sq - 1.0) > 0.1) {
        return false;
    }
    
    return true;
}

// ============================================================================
// FrankaRobotInstance - Implementation
// ============================================================================

FrankaRobotInstance::FrankaRobotInstance(const std::string& ip) 
    : ip_(ip) {
    
    try {
        robot_ = std::make_unique<franka::Robot>(ip_);
        
        model_ = std::make_unique<franka::Model>(robot_->loadModel());
        
        std::cout << "FrankaRobotInstance: Connected to robot at " << ip_ << std::endl;
        
    } catch (const franka::Exception& e) {
        std::cerr << "FrankaRobotInstance: Failed to connect to " << ip_ 
                  << ": " << e.what() << std::endl;
        throw;
    }
}

FrankaRobotInstance::~FrankaRobotInstance() {
    // If control is still active, request stop and wait
    if (active_controller_.load() != nullptr) {
        requestStop();
        // Give it a moment to clean up (the context's thread should handle this)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "FrankaRobotInstance: Disconnected from " << ip_ << std::endl;
}

bool FrankaRobotInstance::claimControl(FrankaRobotContext* ctx) {
    FrankaRobotContext* expected = nullptr;
    if (active_controller_.compare_exchange_strong(expected, ctx)) {
        std::cout << "FrankaRobotInstance [" << ip_ << "]: Control claimed" << std::endl;
        return true;
    }
    
    if (expected == ctx) {
        // Already have control
        return true;
    }
    
    std::cerr << "FrankaRobotInstance [" << ip_ << "]: Cannot claim control - "
              << "another context is active" << std::endl;
    return false;
}

void FrankaRobotInstance::releaseControl(FrankaRobotContext* ctx) {
    FrankaRobotContext* expected = ctx;
    if (active_controller_.compare_exchange_strong(expected, nullptr)) {
        std::cout << "FrankaRobotInstance [" << ip_ << "]: Control released" << std::endl;
    }
    // If we weren't the owner, do nothing (already released or someone else has it)
}

void FrankaRobotInstance::requestStop() {
    stop_requested_.store(true);
}

bool FrankaRobotInstance::automaticErrorRecovery() {
    if (isControlActive()) {
        std::cerr << "FrankaRobotInstance [" << ip_ << "]: Cannot run error recovery "
                  << "while control is active" << std::endl;
        return false;
    }
    
    try {
        robot_->automaticErrorRecovery();
        std::cout << "FrankaRobotInstance [" << ip_ << "]: Error recovery succeeded" << std::endl;
        return true;
    } catch (const franka::Exception& e) {
        std::cerr << "FrankaRobotInstance [" << ip_ << "]: Error recovery failed: " 
                  << e.what() << std::endl;
        return false;
    }
}

bool FrankaRobotInstance::readOnce(FrankaRobotStateBus* state_out, FrankaModelDataBus* model_out) {
    if (isControlActive()) {
        std::cerr << "FrankaRobotInstance [" << ip_ << "]: Cannot readOnce while control is active"
                  << std::endl;
        return false;
    }
    
    try {
        franka::RobotState state = robot_->readOnce();
        
        if (state_out) {
            copyRobotState(state, state_out);
        }
        if (model_out && model_) {
            computeModelData(state, model_out);
        }
        return true;
    } catch (const franka::Exception& e) {
        std::cerr << "FrankaRobotInstance [" << ip_ << "]: readOnce failed: " 
                  << e.what() << std::endl;
        return false;
    }
}

void FrankaRobotInstance::applySettings(const FrankaRobotSettingsBus* settings) {
    if (!settings) {
        return;
    }
    
    try {
        // ====================================================================
        // Apply Collision Behavior
        // ====================================================================
        {
            std::array<double, 7> lower_torque_accel, upper_torque_accel;
            std::array<double, 7> lower_torque_nom, upper_torque_nom;
            std::array<double, 6> lower_force_accel, upper_force_accel;
            std::array<double, 6> lower_force_nom, upper_force_nom;
            
            std::copy(settings->lower_torque_thresholds_acceleration,
                      settings->lower_torque_thresholds_acceleration + 7,
                      lower_torque_accel.begin());
            std::copy(settings->upper_torque_thresholds_acceleration,
                      settings->upper_torque_thresholds_acceleration + 7,
                      upper_torque_accel.begin());
            std::copy(settings->lower_torque_thresholds_nominal,
                      settings->lower_torque_thresholds_nominal + 7,
                      lower_torque_nom.begin());
            std::copy(settings->upper_torque_thresholds_nominal,
                      settings->upper_torque_thresholds_nominal + 7,
                      upper_torque_nom.begin());
            
            std::copy(settings->lower_force_thresholds_acceleration,
                      settings->lower_force_thresholds_acceleration + 6,
                      lower_force_accel.begin());
            std::copy(settings->upper_force_thresholds_acceleration,
                      settings->upper_force_thresholds_acceleration + 6,
                      upper_force_accel.begin());
            std::copy(settings->lower_force_thresholds_nominal,
                      settings->lower_force_thresholds_nominal + 6,
                      lower_force_nom.begin());
            std::copy(settings->upper_force_thresholds_nominal,
                      settings->upper_force_thresholds_nominal + 6,
                      upper_force_nom.begin());
            
            robot_->setCollisionBehavior(
                lower_torque_accel, upper_torque_accel,
                lower_torque_nom, upper_torque_nom,
                lower_force_accel, upper_force_accel,
                lower_force_nom, upper_force_nom
            );
        }
        
        // ====================================================================
        // Apply Joint Impedance
        // ====================================================================
        {
            std::array<double, 7> K_theta;
            std::copy(settings->joint_impedance_stiffness,
                      settings->joint_impedance_stiffness + 7,
                      K_theta.begin());
            robot_->setJointImpedance(K_theta);
        }
        
        // ====================================================================
        // Apply Cartesian Impedance
        // ====================================================================
        {
            std::array<double, 6> K_x;
            std::copy(settings->cartesian_impedance_stiffness,
                      settings->cartesian_impedance_stiffness + 6,
                      K_x.begin());
            robot_->setCartesianImpedance(K_x);
        }
        
        // ====================================================================
        // Apply End Effector Frame (NE_T_EE)
        // ====================================================================
        {
            std::array<double, 16> NE_T_EE;
            std::copy(settings->NE_T_EE,
                      settings->NE_T_EE + 16,
                      NE_T_EE.begin());
            robot_->setEE(NE_T_EE);
        }
        
        // ====================================================================
        // Apply Stiffness Frame (EE_T_K)
        // ====================================================================
        {
            std::array<double, 16> EE_T_K;
            std::copy(settings->EE_T_K,
                      settings->EE_T_K + 16,
                      EE_T_K.begin());
            robot_->setK(EE_T_K);
        }
        
        // ====================================================================
        // Apply External Load
        // ====================================================================
        {
            double load_mass = settings->load_mass;
            std::array<double, 3> F_x_Cload;
            std::array<double, 9> load_inertia;
            
            std::copy(settings->load_center_of_mass,
                      settings->load_center_of_mass + 3,
                      F_x_Cload.begin());
            std::copy(settings->load_inertia_matrix,
                      settings->load_inertia_matrix + 9,
                      load_inertia.begin());
            
            robot_->setLoad(load_mass, F_x_Cload, load_inertia);
        }
        
        std::cout << "FrankaRobotInstance [" << ip_ << "]: Settings applied" << std::endl;
        
    } catch (const franka::Exception& e) {
        std::cerr << "FrankaRobotInstance [" << ip_ << "]: Failed to apply settings: " 
                  << e.what() << std::endl;
    }
}

void FrankaRobotInstance::copyRobotState(const franka::RobotState& src, 
                                          FrankaRobotStateBus* dst) {
    // Copy flat arrays directly (libfranka uses std::array which is contiguous)
    #define COPY_ARRAY(field) \
        std::copy(src.field.begin(), src.field.end(), dst->field)
    
    // Transformation Matrices (4x4 stored as flat 16-element arrays)
    COPY_ARRAY(O_T_EE);
    COPY_ARRAY(O_T_EE_d);
    COPY_ARRAY(F_T_EE);
    COPY_ARRAY(F_T_NE);
    COPY_ARRAY(NE_T_EE);
    COPY_ARRAY(EE_T_K);
    COPY_ARRAY(O_T_EE_c);
    
    // End Effector Inertial Parameters
    dst->m_ee = src.m_ee;
    COPY_ARRAY(I_ee);
    COPY_ARRAY(F_x_Cee);
    
    // External Load Inertial Parameters
    dst->m_load = src.m_load;
    COPY_ARRAY(I_load);
    COPY_ARRAY(F_x_Cload);
    
    // Total Inertial Parameters
    dst->m_total = src.m_total;
    COPY_ARRAY(I_total);
    COPY_ARRAY(F_x_Ctotal);
    
    // Elbow Configuration
    COPY_ARRAY(elbow);
    COPY_ARRAY(elbow_d);
    COPY_ARRAY(elbow_c);
    COPY_ARRAY(delbow_c);
    COPY_ARRAY(ddelbow_c);
    
    // Joint-Space Signals
    COPY_ARRAY(tau_J);
    COPY_ARRAY(tau_J_d);
    COPY_ARRAY(dtau_J);
    COPY_ARRAY(q);
    COPY_ARRAY(q_d);
    COPY_ARRAY(dq);
    COPY_ARRAY(dq_d);
    COPY_ARRAY(ddq_d);
    COPY_ARRAY(theta);
    COPY_ARRAY(dtheta);
    
    // Contact and Collision Detection
    COPY_ARRAY(joint_contact);
    COPY_ARRAY(cartesian_contact);
    COPY_ARRAY(joint_collision);
    COPY_ARRAY(cartesian_collision);
    
    // External Force Estimates
    COPY_ARRAY(tau_ext_hat_filtered);
    COPY_ARRAY(O_F_ext_hat_K);
    COPY_ARRAY(K_F_ext_hat_K);
    
    // Cartesian Motion Signals
    COPY_ARRAY(O_dP_EE_d);
    COPY_ARRAY(O_ddP_O);
    COPY_ARRAY(O_dP_EE_c);
    COPY_ARRAY(O_ddP_EE_c);
    
    // Status Signals
    dst->control_command_success_rate = src.control_command_success_rate;
    dst->robot_mode = static_cast<int32_t>(src.robot_mode);
    dst->time = src.time.toSec();
    
    #undef COPY_ARRAY
}

void FrankaRobotInstance::computeModelData(const franka::RobotState& state,
                                            FrankaModelDataBus* dst) {
    // Mass matrix M(q) - stored as flat 49-element array (7x7 col-major)
    {
        auto M = model_->mass(state);
        std::copy(M.begin(), M.end(), dst->mass);
    }
    
    // Coriolis force vector c(q,dq)
    {
        auto c = model_->coriolis(state);
        std::copy(c.begin(), c.end(), dst->coriolis);
    }
    
    // Gravity vector g(q)
    {
        auto g = model_->gravity(state);
        std::copy(g.begin(), g.end(), dst->gravity);
    }
    
    // End effector Jacobian in base frame - stored as flat 42-element array (6x7 col-major)
    {
        auto J = model_->zeroJacobian(franka::Frame::kEndEffector, state);
        std::copy(J.begin(), J.end(), dst->jacobian);
    }
    
    // End effector body Jacobian - stored as flat 42-element array (6x7 col-major)
    {
        auto J_body = model_->bodyJacobian(franka::Frame::kEndEffector, state);
        std::copy(J_body.begin(), J_body.end(), dst->jacobian_body);
    }
}

// ============================================================================
// FrankaRobotManager - Implementation
// ============================================================================

std::string FrankaRobotManager::sanitizeIP(const std::string& ip) {
    auto is_ws = [](unsigned char c) -> bool {
        return c == ' ' || c == '\t' || c == '\r' || c == '\n';
    };

    std::string sanitized = ip;
    
    // Handle embedded nulls
    const std::string::size_type nul_pos = sanitized.find('\0');
    if (nul_pos != std::string::npos) {
        sanitized.resize(nul_pos);
    }
    
    // Trim leading whitespace
    std::string::size_type start = 0;
    while (start < sanitized.size() && is_ws(static_cast<unsigned char>(sanitized[start]))) {
        ++start;
    }
    
    // Trim trailing whitespace
    std::string::size_type end = sanitized.size();
    while (end > start && is_ws(static_cast<unsigned char>(sanitized[end - 1]))) {
        --end;
    }
    
    return sanitized.substr(start, end - start);
}

FrankaRobotInstance* FrankaRobotManager::getOrCreate(const std::string& ip) {
    std::string sanitized = sanitizeIP(ip);
    
    if (sanitized.empty()) {
        throw std::invalid_argument("FrankaRobotManager: robot_ip is empty");
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = instances_.find(sanitized);
    if (it != instances_.end()) {
        return it->second.get();
    }
    
    // Create new instance (may throw)
    auto instance = std::make_unique<FrankaRobotInstance>(sanitized);
    FrankaRobotInstance* ptr = instance.get();
    instances_[sanitized] = std::move(instance);
    
    return ptr;
}

FrankaRobotInstance* FrankaRobotManager::get(const std::string& ip) {
    std::string sanitized = sanitizeIP(ip);
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = instances_.find(sanitized);
    if (it != instances_.end()) {
        return it->second.get();
    }
    
    return nullptr;
}

bool FrankaRobotManager::isConnected(const std::string& ip) {
    return get(ip) != nullptr;
}

void FrankaRobotManager::shutdownAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Request stop on all instances first
    for (auto& pair : instances_) {
        pair.second->requestStop();
    }
    
    // Brief delay to allow control threads to exit
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Clear all instances (destructors will clean up)
    instances_.clear();
    
    std::cout << "FrankaRobotManager: All instances shut down" << std::endl;
}

void FrankaRobotManager::stop(const std::string& ip) {
    FrankaRobotInstance* instance = get(ip);
    if (instance) {
        instance->requestStop();
    }
}

bool FrankaRobotManager::automaticErrorRecovery(const std::string& ip) {
    FrankaRobotInstance* instance = get(ip);
    if (instance) {
        return instance->automaticErrorRecovery();
    }
    return false;
}

bool FrankaRobotManager::readOnce(const std::string& ip, 
                                   FrankaRobotStateBus* state_out,
                                   FrankaModelDataBus* model_out) {
    FrankaRobotInstance* instance = get(ip);
    if (instance) {
        return instance->readOnce(state_out, model_out);
    }
    return false;
}

bool FrankaRobotManager::isControlRunning(const std::string& ip) {
    FrankaRobotInstance* instance = get(ip);
    if (instance) {
        return instance->isControlActive();
    }
    return false;
}

// ============================================================================
// FrankaRobotContext - Implementation
// ============================================================================

FrankaRobotContext::FrankaRobotContext() = default;

FrankaRobotContext::~FrankaRobotContext() {
    shutdown();
}

void FrankaRobotContext::initialize(const std::string& robot_ip) {
    if (instance_) {
        // Already initialized - check if same IP
        if (instance_->ip() == FrankaRobotManager::sanitizeIP(robot_ip)) {
            return;  // Same robot, nothing to do
        }
        // Different robot - release old and get new
        shutdown();
    }
    
    try {
        instance_ = FrankaRobotManager::getOrCreate(robot_ip);
    } catch (const franka::Exception& e) {
        std::cerr << "FrankaRobotContext: Failed to initialize: " << e.what() << std::endl;
        throw;
    }
}

void FrankaRobotContext::setControllerCallback(ControllerCallback callback, void* user_data) {
    controller_callback_ = callback;
    controller_user_data_ = user_data;
}

void FrankaRobotContext::setControlMode(FrankaControlMode mode) {
    control_mode_ = mode;
}

void FrankaRobotContext::setRealtimeConfig(int priority, int cpu_affinity, bool lock_memory) {
    rt_priority_ = priority;
    rt_cpu_affinity_ = cpu_affinity;
    rt_lock_memory_ = lock_memory;
}

void FrankaRobotContext::shutdown() {
    if (running_) {
        requestStop();
    }

    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    
    // Don't release instance_ here - the manager owns it
    // Other blocks may still be using it
}

// I/O Pointer Setters
void FrankaRobotContext::setStateOutputPointer(FrankaRobotStateBus* state_ptr) {
    state_out_ = state_ptr;
}

void FrankaRobotContext::setModelOutputPointer(FrankaModelDataBus* model_ptr) {
    model_out_ = model_ptr;
}

void FrankaRobotContext::setDtOutputPointer(double* dt_sec_ptr) {
    dt_sec_out_ = dt_sec_ptr;
}

void FrankaRobotContext::setSettingsInputPointer(const FrankaRobotSettingsBus* settings_ptr) {
    settings_in_ = settings_ptr;
}

void FrankaRobotContext::setInputPointers(const double* tau_J_d_ptr) {
    tau_J_d_in_ = tau_J_d_ptr;
}

void FrankaRobotContext::setTorqueInputPointer(const double* tau_J_d_ptr) {
    tau_J_d_in_ = tau_J_d_ptr;
}

void FrankaRobotContext::setJointPositionInputPointer(const double* q_d_ptr) {
    q_d_in_ = q_d_ptr;
}

void FrankaRobotContext::setJointVelocityInputPointer(const double* dq_d_ptr) {
    dq_d_in_ = dq_d_ptr;
}

void FrankaRobotContext::setCartesianPoseInputPointer(const double* O_T_EE_d_ptr, 
                                                       const double* elbow_d_ptr) {
    O_T_EE_d_in_ = O_T_EE_d_ptr;
    elbow_d_in_ = elbow_d_ptr;
}

void FrankaRobotContext::setCartesianVelocityInputPointer(const double* O_dP_EE_d_ptr,
                                                           const double* elbow_d_ptr) {
    O_dP_EE_d_in_ = O_dP_EE_d_ptr;
    elbow_d_in_ = elbow_d_ptr;
}

void FrankaRobotContext::setTorquesJointPositionInputPointers(const double* tau_J_d_ptr,
                                                               const double* q_d_ptr) {
    tau_J_d_in_ = tau_J_d_ptr;
    q_d_in_ = q_d_ptr;
}

void FrankaRobotContext::setTorquesJointVelocityInputPointers(const double* tau_J_d_ptr,
                                                               const double* dq_d_ptr) {
    tau_J_d_in_ = tau_J_d_ptr;
    dq_d_in_ = dq_d_ptr;
}

void FrankaRobotContext::setTorquesCartesianPoseInputPointers(const double* tau_J_d_ptr,
                                                               const double* O_T_EE_d_ptr,
                                                               const double* elbow_d_ptr) {
    tau_J_d_in_ = tau_J_d_ptr;
    O_T_EE_d_in_ = O_T_EE_d_ptr;
    elbow_d_in_ = elbow_d_ptr;
}

void FrankaRobotContext::setTorquesCartesianVelocityInputPointers(const double* tau_J_d_ptr,
                                                                   const double* O_dP_EE_d_ptr,
                                                                   const double* elbow_d_ptr) {
    tau_J_d_in_ = tau_J_d_ptr;
    O_dP_EE_d_in_ = O_dP_EE_d_ptr;
    elbow_d_in_ = elbow_d_ptr;
}

// ============================================================================
// Control Methods
// ============================================================================

void FrankaRobotContext::startControl() {
    if (running_) {
        return;
    }

    if (!instance_) {
        std::cerr << "FrankaRobotContext: startControl() called before initialize()"
                  << std::endl;
        return;
    }
    
    // Join any previous control thread before starting a new one
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    
    // Attempt to claim control from the shared instance
    if (!instance_->claimControl(this)) {
        std::cerr << "FrankaRobotContext: Cannot start control - robot is busy" << std::endl;
        return;
    }
    
    // Apply robot settings before starting control
    instance_->applySettings(settings_in_);
    
    // Clear any previous stop request
    instance_->clearStopRequest();

    // Publish state once on the boundary
    publishStateOnce(0.0);
    
    running_ = true;
    control_thread_ = std::thread(&FrankaRobotContext::controlThreadFunc, this);
}

void FrankaRobotContext::requestStop() {
    if (instance_) {
        instance_->requestStop();
    }
}

void FrankaRobotContext::publishStateOnce(double dt_sec_override) {
    if (!instance_) {
        return;
    }

    try {
        const franka::RobotState state = instance_->robot().readOnce();

        if (state_out_) {
            copyRobotState(state, state_out_);
        }
        if (model_out_) {
            computeModelData(state, model_out_);
        }
        if (dt_sec_out_) {
            *dt_sec_out_ = dt_sec_override;
        }
    } catch (const franka::Exception& e) {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) {
            std::cerr << "FrankaRobotContext: readOnce() failed: " << e.what() << std::endl;
        }
    }
}

// ============================================================================
// Real-Time Thread Configuration (PREEMPT_RT)
// ============================================================================

/**
 * @brief Configure current thread for real-time operation under PREEMPT_RT
 * 
 * This function:
 *   1. Optionally locks all memory pages (prevents page faults)
 *   2. Sets SCHED_FIFO scheduling policy with specified priority
 *   3. Optionally pins thread to a specific CPU core for cache locality
 * 
 * @param priority Thread priority for SCHED_FIFO (0=disabled, 1-99)
 * @param cpu_affinity CPU core to pin to (-1=no pinning, 0+=specific core)
 * @param lock_memory Whether to lock memory with mlockall
 * @note Requires CAP_SYS_NICE capability or root privileges for RT scheduling
 * @return true if all requested RT configuration succeeded, false otherwise
 */
static bool configureRealtimeThread(int priority, int cpu_affinity, bool lock_memory) {
    bool success = true;
    
    // 1. Lock all memory pages to prevent page faults during control
    //    MCL_CURRENT: Lock all pages currently mapped
    //    MCL_FUTURE:  Lock pages mapped in the future
    if (lock_memory) {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            std::cerr << "FrankaRobotContext: mlockall() failed (errno=" << errno 
                      << "). Memory may not be locked. Consider running with elevated privileges."
                      << std::endl;
            success = false;
        }
    }
    
    // 2. Set SCHED_FIFO scheduling policy with specified priority
    //    Priority 0 means disabled (stay with default SCHED_OTHER)
    if (priority > 0) {
        struct sched_param param;
        param.sched_priority = priority;
        
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            std::cerr << "FrankaRobotContext: pthread_setschedparam(SCHED_FIFO, " << priority 
                      << ") failed (errno=" << errno << "). Running without RT priority. "
                      << "Consider running with elevated privileges or setting CAP_SYS_NICE."
                      << std::endl;
            success = false;
        }
    }
    
    // 3. Set CPU affinity to pin thread to a specific core
    //    This improves cache locality and reduces migration overhead.
    //    -1 means no pinning (let scheduler decide)
    if (cpu_affinity >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_affinity, &cpuset);
        
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            // CPU affinity failure is less critical - just warn
            std::cerr << "FrankaRobotContext: pthread_setaffinity_np(core " << cpu_affinity 
                      << ") failed (errno=" << errno << "). Thread may migrate between cores." 
                      << std::endl;
            // Don't set success = false; affinity is a nice-to-have
        }
    }
    
    return success;
}

// ============================================================================
// Control Thread
// ============================================================================

void FrankaRobotContext::controlThreadFunc() {
    // Configure thread for real-time operation (PREEMPT_RT)
    // This must be done from within the thread itself
    configureRealtimeThread(rt_priority_, rt_cpu_affinity_, rt_lock_memory_);
    
    // Get rate limiter and cutoff frequency from settings
    bool limit_rate = true;
    double cutoff_frequency = 100.0;
    
    if (settings_in_) {
        limit_rate = (settings_in_->rate_limiter > 0.5);
        cutoff_frequency = settings_in_->cutoff_frequency;
    }
    
    // Publish boundary state BEFORE entering robot.control()
    // This gives the Simulink outputs an initial state (though O_T_EE_c may be stale
    // from a previous control session - the correct O_T_EE_c is only available inside
    // robot.control() callbacks where libfranka initializes it properly).
    publishStateOnce(0.0);
    
    // DUAL-CALLBACK MODE DESIGN:
    //
    // In libfranka's dual-callback mode, the motion callback runs FIRST, then the
    // torque callback. Both callbacks receive the same robot state.
    //
    // Our design: The Simulink controller runs in the MOTION callback (via executePreCallback).
    // This ensures the controller has computed all outputs (O_T_EE_d, tau_J_d, etc.)
    // BEFORE either callback reads from them.
    //
    // Flow per cycle:
    //   1. Motion callback: executePreCallback() → copies state, runs Simulink controller
    //                       → returns O_T_EE_d (or q_d, dq_d, etc.) from Simulink
    //   2. Torque callback: just reads tau_J_d from Simulink (controller already ran)
    //
    // On the FIRST callback (period == 0), libfranka initializes state.O_T_EE_c to match
    // the current measured pose. The Simulink controller sees this correct value and can
    // use it for "capture on first sample" logic (e.g., initial_pose = O_T_EE_c).

    try {
        switch (control_mode_) {
            // ================================================================
            // Single-callback modes (0-4)
            // ================================================================
            case FrankaControlMode::Torques:
                instance_->robot().control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::Torques {
                        return this->torqueCallback(state, period);
                    },
                    limit_rate,
                    cutoff_frequency
                );
                break;
                
            case FrankaControlMode::JointPositions:
                instance_->robot().control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::JointPositions {
                        return this->jointPositionCallback(state, period);
                    },
                    franka::ControllerMode::kJointImpedance,
                    limit_rate,
                    cutoff_frequency
                );
                break;
                
            case FrankaControlMode::JointVelocities:
                instance_->robot().control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::JointVelocities {
                        return this->jointVelocityCallback(state, period);
                    },
                    franka::ControllerMode::kJointImpedance,
                    limit_rate,
                    cutoff_frequency
                );
                break;
                
            case FrankaControlMode::CartesianPose:
                instance_->robot().control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::CartesianPose {
                        return this->cartesianPoseCallback(state, period);
                    },
                    franka::ControllerMode::kCartesianImpedance,
                    limit_rate,
                    cutoff_frequency
                );
                break;
                
            case FrankaControlMode::CartesianVelocities:
                instance_->robot().control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::CartesianVelocities {
                        return this->cartesianVelocityCallback(state, period);
                    },
                    franka::ControllerMode::kCartesianImpedance,
                    limit_rate,
                    cutoff_frequency
                );
                break;
                
            // ================================================================
            // Dual-callback modes (5-8): Torque + Motion Generator
            // ================================================================
            case FrankaControlMode::TorquesJointPositions:
                instance_->robot().control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::Torques {
                        return this->dualTorqueCallback(state, period);
                    },
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::JointPositions {
                        return this->dualJointPositionMotionCallback(state, period);
                    },
                    limit_rate,
                    cutoff_frequency
                );
                break;
                
            case FrankaControlMode::TorquesJointVelocities:
                instance_->robot().control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::Torques {
                        return this->dualTorqueCallback(state, period);
                    },
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::JointVelocities {
                        return this->dualJointVelocityMotionCallback(state, period);
                    },
                    limit_rate,
                    cutoff_frequency
                );
                break;
                
            case FrankaControlMode::TorquesCartesianPose:
                instance_->robot().control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::Torques {
                        return this->dualTorqueCallback(state, period);
                    },
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::CartesianPose {
                        return this->dualCartesianPoseMotionCallback(state, period);
                    },
                    limit_rate,
                    cutoff_frequency
                );
                break;
                
            case FrankaControlMode::TorquesCartesianVelocities:
                instance_->robot().control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::Torques {
                        return this->dualTorqueCallback(state, period);
                    },
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::CartesianVelocities {
                        return this->dualCartesianVelocityMotionCallback(state, period);
                    },
                    limit_rate,
                    cutoff_frequency
                );
                break;
        }
    } catch (const franka::Exception& e) {
        std::cerr << "FrankaRobotContext: Control exception: " << e.what() << std::endl;
    }

    // Publish boundary state AFTER robot.control() ends
    publishStateOnce(0.0);
    
    // Release control
    instance_->releaseControl(this);
    running_ = false;
}

// ============================================================================
// Common Pre-Callback Logic
// ============================================================================

bool FrankaRobotContext::executePreCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    // Check for stop request (from this context or via manager)
    if (instance_->isStopRequested()) {
        return false;
    }
    
    // Copy robot state to Simulink output bus
    if (state_out_) {
        copyRobotState(state, state_out_);
    }
    
    // Compute and copy model data
    if (model_out_) {
        computeModelData(state, model_out_);
    }

    // Compute dt_sec and execute controller
    const double dt_sec = period.toSec();
    if (dt_sec_out_) {
        *dt_sec_out_ = dt_sec;
    }
    
    if (controller_callback_) {
        controller_callback_(controller_user_data_, dt_sec);
    }
    
    return true;
}

// ============================================================================
// Mode-Specific Callbacks
// ============================================================================

franka::Torques FrankaRobotContext::torqueCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (!executePreCallback(state, period)) {
        return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
    }
    
    std::array<double, 7> tau_cmd{};
    if (tau_J_d_in_) {
        std::copy(tau_J_d_in_, tau_J_d_in_ + 7, tau_cmd.begin());
    }
    sanitizeArray(tau_cmd, "tau_J_d");
    
    return franka::Torques(tau_cmd);
}

franka::JointPositions FrankaRobotContext::jointPositionCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (!executePreCallback(state, period)) {
        return franka::MotionFinished(franka::JointPositions(state.q_d));
    }
    
    std::array<double, 7> q_cmd{};
    if (q_d_in_) {
        std::copy(q_d_in_, q_d_in_ + 7, q_cmd.begin());
    } else {
        q_cmd = state.q_d;
    }
    sanitizeArray(q_cmd, "q_d");
    
    return franka::JointPositions(q_cmd);
}

franka::JointVelocities FrankaRobotContext::jointVelocityCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (!executePreCallback(state, period)) {
        return franka::MotionFinished(franka::JointVelocities({0, 0, 0, 0, 0, 0, 0}));
    }
    
    std::array<double, 7> dq_cmd{};
    if (dq_d_in_) {
        std::copy(dq_d_in_, dq_d_in_ + 7, dq_cmd.begin());
    }
    sanitizeArray(dq_cmd, "dq_d");
    
    return franka::JointVelocities(dq_cmd);
}

franka::CartesianPose FrankaRobotContext::cartesianPoseCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (!executePreCallback(state, period)) {
        return franka::MotionFinished(franka::CartesianPose(state.O_T_EE_d, state.elbow_d));
    }
    
    std::array<double, 16> pose_cmd{};
    if (O_T_EE_d_in_) {
        std::copy(O_T_EE_d_in_, O_T_EE_d_in_ + 16, pose_cmd.begin());
    } else {
        pose_cmd = state.O_T_EE_d;
    }
    
    // Validate transformation matrix - if invalid, use robot's current desired pose
    if (!isValidTransformationMatrix(pose_cmd)) {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) {
            std::cerr << "FrankaRobotAPI: Invalid O_T_EE_d transformation matrix detected "
                         "(likely zero-initialized). Using robot's current O_T_EE_d."
                      << std::endl;
        }
        pose_cmd = state.O_T_EE_d;
    }
    
    std::array<double, 2> elbow_cmd{};
    if (elbow_d_in_) {
        std::copy(elbow_d_in_, elbow_d_in_ + 2, elbow_cmd.begin());
    } else {
        elbow_cmd = state.elbow_d;
    }
    sanitizeArray(elbow_cmd, "elbow_d");
    
    return franka::CartesianPose(pose_cmd, elbow_cmd);
}

franka::CartesianVelocities FrankaRobotContext::cartesianVelocityCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (!executePreCallback(state, period)) {
        return franka::MotionFinished(franka::CartesianVelocities({0, 0, 0, 0, 0, 0}, state.elbow_d));
    }
    
    std::array<double, 6> vel_cmd{};
    if (O_dP_EE_d_in_) {
        std::copy(O_dP_EE_d_in_, O_dP_EE_d_in_ + 6, vel_cmd.begin());
    }
    sanitizeArray(vel_cmd, "O_dP_EE_d");
    
    std::array<double, 2> elbow_cmd{};
    if (elbow_d_in_) {
        std::copy(elbow_d_in_, elbow_d_in_ + 2, elbow_cmd.begin());
    } else {
        elbow_cmd = state.elbow_d;
    }
    sanitizeArray(elbow_cmd, "elbow_d");
    
    return franka::CartesianVelocities(vel_cmd, elbow_cmd);
}

// ============================================================================
// Dual-Callback Mode Callbacks
// ============================================================================

franka::Torques FrankaRobotContext::dualTorqueCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    // In dual-callback mode, the motion callback runs FIRST and executes the
    // Simulink controller. The torque callback just reads the outputs.
    // We only check for stop request here.
    if (instance_->isStopRequested()) {
        return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
    }
    
    std::array<double, 7> tau_cmd{};
    if (tau_J_d_in_) {
        std::copy(tau_J_d_in_, tau_J_d_in_ + 7, tau_cmd.begin());
    }
    sanitizeArray(tau_cmd, "tau_J_d");
    
    return franka::Torques(tau_cmd);
}

franka::JointPositions FrankaRobotContext::dualJointPositionMotionCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    // In dual-callback mode, the motion callback runs FIRST.
    // We run the Simulink controller here so it can compute outputs for both callbacks.
    if (!executePreCallback(state, period)) {
        return franka::MotionFinished(franka::JointPositions(state.q_d));
    }
    
    std::array<double, 7> q_cmd{};
    if (q_d_in_) {
        std::copy(q_d_in_, q_d_in_ + 7, q_cmd.begin());
    } else {
        q_cmd = state.q_d;
    }
    sanitizeArray(q_cmd, "q_d");
    
    return franka::JointPositions(q_cmd);
}

franka::JointVelocities FrankaRobotContext::dualJointVelocityMotionCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    // In dual-callback mode, the motion callback runs FIRST.
    // We run the Simulink controller here so it can compute outputs for both callbacks.
    if (!executePreCallback(state, period)) {
        return franka::MotionFinished(franka::JointVelocities({0, 0, 0, 0, 0, 0, 0}));
    }
    
    std::array<double, 7> dq_cmd{};
    if (dq_d_in_) {
        std::copy(dq_d_in_, dq_d_in_ + 7, dq_cmd.begin());
    }
    sanitizeArray(dq_cmd, "dq_d");
    
    return franka::JointVelocities(dq_cmd);
}

franka::CartesianPose FrankaRobotContext::dualCartesianPoseMotionCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    // In dual-callback mode, the motion callback runs FIRST.
    // We run the Simulink controller here so it can compute outputs for both callbacks.
    if (!executePreCallback(state, period)) {
        return franka::MotionFinished(franka::CartesianPose(state.O_T_EE_d, state.elbow_d));
    }
    
    std::array<double, 16> pose_cmd{};
    if (O_T_EE_d_in_) {
        std::copy(O_T_EE_d_in_, O_T_EE_d_in_ + 16, pose_cmd.begin());
    } else {
        pose_cmd = state.O_T_EE_d;
    }
    
    // Validate transformation matrix - if invalid, use robot's current commanded pose
    if (!isValidTransformationMatrix(pose_cmd)) {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) {
            std::cerr << "FrankaRobotAPI: Invalid O_T_EE_d transformation matrix "
                         "(likely zero-initialized). Using robot's current O_T_EE_c."
                      << std::endl;
        }
        pose_cmd = state.O_T_EE_c;
    }
    
    std::array<double, 2> elbow_cmd{};
    if (elbow_d_in_) {
        std::copy(elbow_d_in_, elbow_d_in_ + 2, elbow_cmd.begin());
    } else {
        elbow_cmd = state.elbow_c;
    }
    sanitizeArray(elbow_cmd, "elbow_d");
    
    return franka::CartesianPose(pose_cmd, elbow_cmd);
}

franka::CartesianVelocities FrankaRobotContext::dualCartesianVelocityMotionCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    // In dual-callback mode, the motion callback runs FIRST.
    // We run the Simulink controller here so it can compute outputs for both callbacks.
    if (!executePreCallback(state, period)) {
        return franka::MotionFinished(franka::CartesianVelocities({0, 0, 0, 0, 0, 0}, state.elbow_d));
    }
    
    std::array<double, 6> vel_cmd{};
    if (O_dP_EE_d_in_) {
        std::copy(O_dP_EE_d_in_, O_dP_EE_d_in_ + 6, vel_cmd.begin());
    }
    sanitizeArray(vel_cmd, "O_dP_EE_d");
    
    std::array<double, 2> elbow_cmd{};
    if (elbow_d_in_) {
        std::copy(elbow_d_in_, elbow_d_in_ + 2, elbow_cmd.begin());
    } else {
        elbow_cmd = state.elbow_c;
    }
    sanitizeArray(elbow_cmd, "elbow_d");
    
    return franka::CartesianVelocities(vel_cmd, elbow_cmd);
}

// ============================================================================
// State Copying (duplicated from FrankaRobotInstance for context's local use)
// ============================================================================

void FrankaRobotContext::copyRobotState(const franka::RobotState& src, 
                                         FrankaRobotStateBus* dst) {
    // Copy flat arrays directly (libfranka uses std::array which is contiguous)
    #define COPY_ARRAY(field) \
        std::copy(src.field.begin(), src.field.end(), dst->field)
    
    // Transformation Matrices (4x4 stored as flat 16-element arrays)
    COPY_ARRAY(O_T_EE);
    COPY_ARRAY(O_T_EE_d);
    COPY_ARRAY(F_T_EE);
    COPY_ARRAY(F_T_NE);
    COPY_ARRAY(NE_T_EE);
    COPY_ARRAY(EE_T_K);
    COPY_ARRAY(O_T_EE_c);
    
    dst->m_ee = src.m_ee;
    COPY_ARRAY(I_ee);
    COPY_ARRAY(F_x_Cee);
    
    dst->m_load = src.m_load;
    COPY_ARRAY(I_load);
    COPY_ARRAY(F_x_Cload);
    
    dst->m_total = src.m_total;
    COPY_ARRAY(I_total);
    COPY_ARRAY(F_x_Ctotal);
    
    COPY_ARRAY(elbow);
    COPY_ARRAY(elbow_d);
    COPY_ARRAY(elbow_c);
    COPY_ARRAY(delbow_c);
    COPY_ARRAY(ddelbow_c);
    
    COPY_ARRAY(tau_J);
    COPY_ARRAY(tau_J_d);
    COPY_ARRAY(dtau_J);
    COPY_ARRAY(q);
    COPY_ARRAY(q_d);
    COPY_ARRAY(dq);
    COPY_ARRAY(dq_d);
    COPY_ARRAY(ddq_d);
    COPY_ARRAY(theta);
    COPY_ARRAY(dtheta);
    
    COPY_ARRAY(joint_contact);
    COPY_ARRAY(cartesian_contact);
    COPY_ARRAY(joint_collision);
    COPY_ARRAY(cartesian_collision);
    
    COPY_ARRAY(tau_ext_hat_filtered);
    COPY_ARRAY(O_F_ext_hat_K);
    COPY_ARRAY(K_F_ext_hat_K);
    
    COPY_ARRAY(O_dP_EE_d);
    COPY_ARRAY(O_ddP_O);
    COPY_ARRAY(O_dP_EE_c);
    COPY_ARRAY(O_ddP_EE_c);
    
    dst->control_command_success_rate = src.control_command_success_rate;
    dst->robot_mode = static_cast<int32_t>(src.robot_mode);
    dst->time = src.time.toSec();
    
    #undef COPY_ARRAY
}

void FrankaRobotContext::computeModelData(const franka::RobotState& state,
                                           FrankaModelDataBus* dst) {
    if (!instance_) return;
    
    auto& model = instance_->model();
    
    // Mass matrix M(q) - stored as flat 49-element array (7x7 col-major)
    {
        auto M = model.mass(state);
        std::copy(M.begin(), M.end(), dst->mass);
    }
    
    {
        auto c = model.coriolis(state);
        std::copy(c.begin(), c.end(), dst->coriolis);
    }
    
    {
        auto g = model.gravity(state);
        std::copy(g.begin(), g.end(), dst->gravity);
    }
    
    // Jacobians - stored as flat 42-element arrays (6x7 col-major)
    {
        auto J = model.zeroJacobian(franka::Frame::kEndEffector, state);
        std::copy(J.begin(), J.end(), dst->jacobian);
    }
    
    {
        auto J_body = model.bodyJacobian(franka::Frame::kEndEffector, state);
        std::copy(J_body.begin(), J_body.end(), dst->jacobian_body);
    }
}
