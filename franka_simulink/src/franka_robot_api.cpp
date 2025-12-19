// Copyright (c) 2025 Franka Robotics GmbH
// Franka Robot API - Implementation

#include "franka_robot_api.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

// ============================================================================
// Constructor / Destructor
// ============================================================================

FrankaRobotContext::FrankaRobotContext() = default;

FrankaRobotContext::~FrankaRobotContext() {
    shutdown();
}

// ============================================================================
// Lifecycle Methods
// ============================================================================

void FrankaRobotContext::initialize(const std::string& robot_ip) {
    // Defensive sanitation:
    // - Some codegen paths (esp. when strings are made tunable and then converted to
    //   fixed-width ASCII vectors) can introduce trailing whitespace or embedded '\0'.
    // - libfranka expects a clean hostname/IP string.
    auto is_ws = [](unsigned char c) -> bool {
        return c == ' ' || c == '\t' || c == '\r' || c == '\n';
    };

    std::string sanitized = robot_ip;
    const std::string::size_type nul_pos = sanitized.find('\0');
    if (nul_pos != std::string::npos) {
        sanitized.resize(nul_pos);
    }
    std::string::size_type start = 0;
    while (start < sanitized.size() && is_ws(static_cast<unsigned char>(sanitized[start]))) {
        ++start;
    }
    std::string::size_type end = sanitized.size();
    while (end > start && is_ws(static_cast<unsigned char>(sanitized[end - 1]))) {
        --end;
    }
    sanitized = sanitized.substr(start, end - start);

    if (sanitized.empty()) {
        throw std::invalid_argument("robot_ip is empty");
    }

    // If already connected to the same host, do nothing.
    if (robot_ && robot_ip_ == sanitized) {
        return;
    }

    // If a previous connection exists, cleanly tear it down first.
    // (Should not happen during running control, but keep it safe.)
    shutdown();

    robot_ip_ = sanitized;
    
    try {
        robot_ = std::make_unique<franka::Robot>(robot_ip_);
        
        // Attempt automatic error recovery
        try {
            robot_->automaticErrorRecovery();
        } catch (const franka::Exception&) {
            // Robot might already be in good state
        }
        
        model_ = std::make_unique<franka::Model>(robot_->loadModel());
        
    } catch (const franka::Exception& e) {
        std::cerr << "Failed to connect to robot: " << e.what() << std::endl;
        throw;
    }
}

bool FrankaRobotContext::isInitialized() const {
    return static_cast<bool>(robot_);
}

void FrankaRobotContext::setControllerCallback(ControllerCallback callback, void* user_data) {
    controller_callback_ = callback;
    controller_user_data_ = user_data;
}

void FrankaRobotContext::setStateOutputPointer(FrankaRobotStateBus* state_ptr) {
    state_out_ = state_ptr;
}

void FrankaRobotContext::setModelOutputPointer(FrankaModelDataBus* model_ptr) {
    model_out_ = model_ptr;
}

void FrankaRobotContext::setDtOutputPointer(double* dt_sec_ptr) {
    dt_sec_out_ = dt_sec_ptr;
}

void FrankaRobotContext::setInputPointers(const double* tau_J_d_ptr) {
    tau_J_d_in_ = tau_J_d_ptr;
}

void FrankaRobotContext::setControlMode(FrankaControlMode mode) {
    control_mode_ = mode;
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

// ============================================================================
// Dual-Callback Mode Input Setters
// ============================================================================

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

void FrankaRobotContext::setSettingsInputPointer(const FrankaRobotSettingsBus* settings_ptr) {
    settings_in_ = settings_ptr;
}

void FrankaRobotContext::applySettings() {
    if (!robot_ || !settings_in_) {
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
            
            std::copy(settings_in_->lower_torque_thresholds_acceleration,
                      settings_in_->lower_torque_thresholds_acceleration + 7,
                      lower_torque_accel.begin());
            std::copy(settings_in_->upper_torque_thresholds_acceleration,
                      settings_in_->upper_torque_thresholds_acceleration + 7,
                      upper_torque_accel.begin());
            std::copy(settings_in_->lower_torque_thresholds_nominal,
                      settings_in_->lower_torque_thresholds_nominal + 7,
                      lower_torque_nom.begin());
            std::copy(settings_in_->upper_torque_thresholds_nominal,
                      settings_in_->upper_torque_thresholds_nominal + 7,
                      upper_torque_nom.begin());
            
            std::copy(settings_in_->lower_force_thresholds_acceleration,
                      settings_in_->lower_force_thresholds_acceleration + 6,
                      lower_force_accel.begin());
            std::copy(settings_in_->upper_force_thresholds_acceleration,
                      settings_in_->upper_force_thresholds_acceleration + 6,
                      upper_force_accel.begin());
            std::copy(settings_in_->lower_force_thresholds_nominal,
                      settings_in_->lower_force_thresholds_nominal + 6,
                      lower_force_nom.begin());
            std::copy(settings_in_->upper_force_thresholds_nominal,
                      settings_in_->upper_force_thresholds_nominal + 6,
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
            std::copy(settings_in_->joint_impedance_stiffness,
                      settings_in_->joint_impedance_stiffness + 7,
                      K_theta.begin());
            robot_->setJointImpedance(K_theta);
        }
        
        // ====================================================================
        // Apply Cartesian Impedance
        // ====================================================================
        {
            std::array<double, 6> K_x;
            std::copy(settings_in_->cartesian_impedance_stiffness,
                      settings_in_->cartesian_impedance_stiffness + 6,
                      K_x.begin());
            robot_->setCartesianImpedance(K_x);
        }
        
        // ====================================================================
        // Apply End Effector Frame (NE_T_EE)
        // ====================================================================
        {
            std::array<double, 16> NE_T_EE;
            // Copy 4x4 matrix (column-major) to flat array
            std::copy(&settings_in_->NE_T_EE[0][0],
                      &settings_in_->NE_T_EE[0][0] + 16,
                      NE_T_EE.begin());
            robot_->setEE(NE_T_EE);
        }
        
        // ====================================================================
        // Apply Stiffness Frame (EE_T_K)
        // ====================================================================
        {
            std::array<double, 16> EE_T_K;
            std::copy(&settings_in_->EE_T_K[0][0],
                      &settings_in_->EE_T_K[0][0] + 16,
                      EE_T_K.begin());
            robot_->setK(EE_T_K);
        }
        
        // ====================================================================
        // Apply External Load
        // ====================================================================
        {
            double load_mass = settings_in_->load_mass;
            std::array<double, 3> F_x_Cload;
            std::array<double, 9> load_inertia;
            
            std::copy(settings_in_->load_center_of_mass,
                      settings_in_->load_center_of_mass + 3,
                      F_x_Cload.begin());
            std::copy(&settings_in_->load_inertia_matrix[0][0],
                      &settings_in_->load_inertia_matrix[0][0] + 9,
                      load_inertia.begin());
            
            robot_->setLoad(load_mass, F_x_Cload, load_inertia);
        }
        
        std::cout << "Robot settings applied successfully" << std::endl;
        
    } catch (const franka::Exception& e) {
        std::cerr << "Failed to apply robot settings: " << e.what() << std::endl;
    }
}

void FrankaRobotContext::shutdown() {
    if (running_) {
        requestStop();
    }

    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    model_.reset();
    robot_.reset();
}

void FrankaRobotContext::publishStateOnce(double dt_sec_override) {
    if (!robot_) {
        return;
    }

    try {
        const franka::RobotState state = robot_->readOnce();

        if (state_out_) {
            copyRobotState(state, state_out_);
        }
        if (model_out_ && model_) {
            computeModelData(state, model_out_);
        }
        if (dt_sec_out_) {
            *dt_sec_out_ = dt_sec_override;
        }
    } catch (const franka::Exception& e) {
        static std::atomic<bool> warned{false};
        if (!warned.exchange(true)) {
            std::cerr << "FrankaRobotContext: readOnce() failed while publishing boundary state: "
                      << e.what() << std::endl;
        }
    }
}

// ============================================================================
// Control Methods
// ============================================================================

void FrankaRobotContext::startControl() {
    if (running_) {
        return;
    }

    if (!robot_) {
        std::cerr << "FrankaRobotContext: startControl() called before initialize(); ignoring."
                  << std::endl;
        return;
    }
    
    // Join any previous control thread before starting a new one
    // This handles the case of enable cycling: 1 -> 0 -> 1
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    
    // Apply robot settings before starting control
    // Settings are re-read on each enable, allowing runtime changes
    applySettings();

    // Publish state once on the boundary, before entering robot.control().
    // This makes robot_mode visible even if control fails to start (robot in error).
    publishStateOnce(0.0);
    
    stop_requested_ = false;
    running_ = true;

    control_thread_ = std::thread(&FrankaRobotContext::controlThreadFunc, this);
}

void FrankaRobotContext::requestStop() {
    stop_requested_ = true;
}

bool FrankaRobotContext::isControlRunning() const {
    return running_;
}

// ============================================================================
// Control Thread
// ============================================================================

void FrankaRobotContext::controlThreadFunc() {
    // Get rate limiter and cutoff frequency from settings
    // Default to safe values if settings not provided
    bool limit_rate = true;
    double cutoff_frequency = 100.0;
    
    if (settings_in_) {
        limit_rate = (settings_in_->rate_limiter > 0.5);
        cutoff_frequency = settings_in_->cutoff_frequency;
    }
    
    // Publish boundary state BEFORE entering robot.control().
    // Important: readOnce() must not be called while robot.control() runs.
    publishStateOnce(0.0);

    try {
        switch (control_mode_) {
            // ================================================================
            // Single-callback modes (0-4)
            // ================================================================
            case FrankaControlMode::Torques:
                robot_->control(
                    [this](const franka::RobotState& state, franka::Duration period) 
                        -> franka::Torques {
                        return this->torqueCallback(state, period);
                    },
                    limit_rate,
                    cutoff_frequency
                );
                break;
                
            case FrankaControlMode::JointPositions:
                robot_->control(
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
                robot_->control(
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
                robot_->control(
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
                robot_->control(
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
                robot_->control(
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
                robot_->control(
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
                robot_->control(
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
                robot_->control(
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
        std::cerr << "Control exception: " << e.what() << std::endl;
    }

    // Publish boundary state AFTER robot.control() ends (normal finish / stop / exception).
    publishStateOnce(0.0);
    
    running_ = false;
}

// ============================================================================
// Common Pre-Callback Logic
// ============================================================================

bool FrankaRobotContext::executePreCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    // Check for stop request
    if (stop_requested_) {
        return false;
    }
    
    // ========================================================================
    // Step 1: Copy robot state to Simulink output bus
    // ========================================================================
    if (state_out_) {
        copyRobotState(state, state_out_);
    }
    
    // ========================================================================
    // Step 1b: Compute and copy model data (dynamics/kinematics)
    // ========================================================================
    if (model_out_ && model_) {
        computeModelData(state, model_out_);
    }

    // ========================================================================
    // Step 2: Compute dt_sec (franka::Duration) and execute controller
    // ========================================================================
    // dt_sec is time since previous callback:
    //   - First callback: period=0 (no previous callback)
    //   - Subsequent callbacks: typically ~0.001s, can be ~0.002s on jitter
    //
    // Following libfranka convention, we pass dt_sec to the controller callback
    // which advances taskTime0 BEFORE running the controller. On first callback,
    // dt_sec=0 so taskTime0 stays at 0, matching libfranka examples.
    const double dt_sec = period.toSec();
    if (dt_sec_out_) {
        *dt_sec_out_ = dt_sec;
    }
    
    if (controller_callback_) {
        controller_callback_(controller_user_data_, dt_sec);
    }
    
    return true;
}

// Helper to check and sanitize array values (guard against NaN/Inf)
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
            std::cerr << "FrankaRobotContext: non-finite " << name << " detected; "
                         "zeroing values. Check Simulink signal wiring / controller execution."
                      << std::endl;
        }
        arr.fill(0.0);
    }
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
    
    // Read commanded torques from Simulink input
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
        // Return current position on stop (smooth stop)
        return franka::MotionFinished(franka::JointPositions(state.q_d));
    }
    
    // Read commanded joint positions from Simulink input
    std::array<double, 7> q_cmd{};
    if (q_d_in_) {
        std::copy(q_d_in_, q_d_in_ + 7, q_cmd.begin());
    } else {
        // Default to current commanded position if no input connected
        q_cmd = state.q_d;
    }
    sanitizeArray(q_cmd, "q_d");
    
    return franka::JointPositions(q_cmd);
}

franka::JointVelocities FrankaRobotContext::jointVelocityCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (!executePreCallback(state, period)) {
        // Return zero velocity on stop (smooth deceleration handled by robot)
        return franka::MotionFinished(franka::JointVelocities({0, 0, 0, 0, 0, 0, 0}));
    }
    
    // Read commanded joint velocities from Simulink input
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
        // Return current pose on stop (smooth stop)
        return franka::MotionFinished(franka::CartesianPose(state.O_T_EE_d, state.elbow_d));
    }
    
    // Read commanded Cartesian pose from Simulink input (4x4 col-major)
    std::array<double, 16> pose_cmd{};
    if (O_T_EE_d_in_) {
        std::copy(O_T_EE_d_in_, O_T_EE_d_in_ + 16, pose_cmd.begin());
    } else {
        // Default to current commanded pose if no input connected
        pose_cmd = state.O_T_EE_d;
    }
    sanitizeArray(pose_cmd, "O_T_EE_d");
    
    // Read elbow configuration (required for Cartesian control)
    std::array<double, 2> elbow_cmd{};
    if (elbow_d_in_) {
        std::copy(elbow_d_in_, elbow_d_in_ + 2, elbow_cmd.begin());
    } else {
        // Default to current elbow configuration
        elbow_cmd = state.elbow_d;
    }
    sanitizeArray(elbow_cmd, "elbow_d");
    
    return franka::CartesianPose(pose_cmd, elbow_cmd);
}

franka::CartesianVelocities FrankaRobotContext::cartesianVelocityCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (!executePreCallback(state, period)) {
        // Return zero velocity on stop
        return franka::MotionFinished(franka::CartesianVelocities({0, 0, 0, 0, 0, 0}, state.elbow_d));
    }
    
    // Read commanded Cartesian velocity from Simulink input [vx, vy, vz, wx, wy, wz]
    std::array<double, 6> vel_cmd{};
    if (O_dP_EE_d_in_) {
        std::copy(O_dP_EE_d_in_, O_dP_EE_d_in_ + 6, vel_cmd.begin());
    }
    sanitizeArray(vel_cmd, "O_dP_EE_d");
    
    // Read elbow configuration (required for Cartesian control)
    std::array<double, 2> elbow_cmd{};
    if (elbow_d_in_) {
        std::copy(elbow_d_in_, elbow_d_in_ + 2, elbow_cmd.begin());
    } else {
        // Default to current elbow configuration
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
    
    // Note: In dual-callback mode, executePreCallback is called from the torque callback
    // The motion generator callback should NOT call executePreCallback again
    if (!executePreCallback(state, period)) {
        return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
    }
    
    // Read commanded torques from Simulink input
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
    
    // Check stop request (executePreCallback was called in torque callback)
    if (stop_requested_) {
        return franka::MotionFinished(franka::JointPositions(state.q_d));
    }
    
    // Read commanded joint positions from Simulink input
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
    
    if (stop_requested_) {
        return franka::MotionFinished(franka::JointVelocities({0, 0, 0, 0, 0, 0, 0}));
    }
    
    // Read commanded joint velocities from Simulink input
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
    
    if (stop_requested_) {
        return franka::MotionFinished(franka::CartesianPose(state.O_T_EE_d, state.elbow_d));
    }
    
    // Read commanded Cartesian pose from Simulink input
    std::array<double, 16> pose_cmd{};
    if (O_T_EE_d_in_) {
        std::copy(O_T_EE_d_in_, O_T_EE_d_in_ + 16, pose_cmd.begin());
    } else {
        pose_cmd = state.O_T_EE_d;
    }
    sanitizeArray(pose_cmd, "O_T_EE_d");
    
    // Read elbow configuration
    std::array<double, 2> elbow_cmd{};
    if (elbow_d_in_) {
        std::copy(elbow_d_in_, elbow_d_in_ + 2, elbow_cmd.begin());
    } else {
        elbow_cmd = state.elbow_d;
    }
    sanitizeArray(elbow_cmd, "elbow_d");
    
    return franka::CartesianPose(pose_cmd, elbow_cmd);
}

franka::CartesianVelocities FrankaRobotContext::dualCartesianVelocityMotionCallback(
    const franka::RobotState& state,
    franka::Duration period) {
    
    if (stop_requested_) {
        return franka::MotionFinished(franka::CartesianVelocities({0, 0, 0, 0, 0, 0}, state.elbow_d));
    }
    
    // Read commanded Cartesian velocity from Simulink input
    std::array<double, 6> vel_cmd{};
    if (O_dP_EE_d_in_) {
        std::copy(O_dP_EE_d_in_, O_dP_EE_d_in_ + 6, vel_cmd.begin());
    }
    sanitizeArray(vel_cmd, "O_dP_EE_d");
    
    // Read elbow configuration
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
// State Copying
// ============================================================================

void FrankaRobotContext::copyRobotState(const franka::RobotState& src, 
                                         FrankaRobotStateBus* dst) {
    // Helper macro to copy std::array to C array (1D)
    #define COPY_ARRAY(field) \
        std::copy(src.field.begin(), src.field.end(), dst->field)
    
    // Helper macro to copy std::array to 2D C array (contiguous memory)
    // Both libfranka and Simulink use column-major order, so direct copy works
    #define COPY_MATRIX_4x4(field) \
        std::copy(src.field.begin(), src.field.end(), &dst->field[0][0])
    
    #define COPY_MATRIX_3x3(field) \
        std::copy(src.field.begin(), src.field.end(), &dst->field[0][0])
    
    // ------------------------------------------------------------------------
    // Transformation Matrices (4x4)
    // ------------------------------------------------------------------------
    COPY_MATRIX_4x4(O_T_EE);
    COPY_MATRIX_4x4(O_T_EE_d);
    COPY_MATRIX_4x4(F_T_EE);
    COPY_MATRIX_4x4(F_T_NE);
    COPY_MATRIX_4x4(NE_T_EE);
    COPY_MATRIX_4x4(EE_T_K);
    COPY_MATRIX_4x4(O_T_EE_c);
    
    // ------------------------------------------------------------------------
    // End Effector Inertial Parameters
    // ------------------------------------------------------------------------
    dst->m_ee = src.m_ee;
    COPY_MATRIX_3x3(I_ee);
    COPY_ARRAY(F_x_Cee);
    
    // ------------------------------------------------------------------------
    // External Load Inertial Parameters
    // ------------------------------------------------------------------------
    dst->m_load = src.m_load;
    COPY_MATRIX_3x3(I_load);
    COPY_ARRAY(F_x_Cload);
    
    // ------------------------------------------------------------------------
    // Total Inertial Parameters
    // ------------------------------------------------------------------------
    dst->m_total = src.m_total;
    COPY_MATRIX_3x3(I_total);
    COPY_ARRAY(F_x_Ctotal);
    
    // ------------------------------------------------------------------------
    // Elbow Configuration
    // ------------------------------------------------------------------------
    COPY_ARRAY(elbow);
    COPY_ARRAY(elbow_d);
    COPY_ARRAY(elbow_c);
    COPY_ARRAY(delbow_c);
    COPY_ARRAY(ddelbow_c);
    
    // ------------------------------------------------------------------------
    // Joint-Space Signals
    // ------------------------------------------------------------------------
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
    
    // ------------------------------------------------------------------------
    // Contact and Collision Detection
    // ------------------------------------------------------------------------
    COPY_ARRAY(joint_contact);
    COPY_ARRAY(cartesian_contact);
    COPY_ARRAY(joint_collision);
    COPY_ARRAY(cartesian_collision);
    
    // ------------------------------------------------------------------------
    // External Force Estimates
    // ------------------------------------------------------------------------
    COPY_ARRAY(tau_ext_hat_filtered);
    COPY_ARRAY(O_F_ext_hat_K);
    COPY_ARRAY(K_F_ext_hat_K);
    
    // ------------------------------------------------------------------------
    // Cartesian Motion Signals
    // ------------------------------------------------------------------------
    COPY_ARRAY(O_dP_EE_d);
    COPY_ARRAY(O_ddP_O);
    COPY_ARRAY(O_dP_EE_c);
    COPY_ARRAY(O_ddP_EE_c);
    
    // ------------------------------------------------------------------------
    // Status Signals
    // ------------------------------------------------------------------------
    dst->control_command_success_rate = src.control_command_success_rate;
    dst->robot_mode = static_cast<int32_t>(src.robot_mode);
    dst->time = src.time.toSec();
    
    #undef COPY_ARRAY
    #undef COPY_MATRIX_4x4
    #undef COPY_MATRIX_3x3
}

// ============================================================================
// Model Data Computation
// ============================================================================

void FrankaRobotContext::computeModelData(const franka::RobotState& state,
                                           FrankaModelDataBus* dst) {
    // ------------------------------------------------------------------------
    // Dynamics: Mass matrix M(q)
    // ------------------------------------------------------------------------
    {
        auto M = model_->mass(state);
        std::copy(M.begin(), M.end(), &dst->mass[0][0]);
    }
    
    // ------------------------------------------------------------------------
    // Dynamics: Coriolis force vector c(q,dq)
    // ------------------------------------------------------------------------
    {
        auto c = model_->coriolis(state);
        std::copy(c.begin(), c.end(), dst->coriolis);
    }
    
    // ------------------------------------------------------------------------
    // Dynamics: Gravity vector g(q)
    // ------------------------------------------------------------------------
    {
        auto g = model_->gravity(state);
        std::copy(g.begin(), g.end(), dst->gravity);
    }
    
    // ------------------------------------------------------------------------
    // Kinematics: End effector Jacobian in base frame (zero Jacobian)
    // ------------------------------------------------------------------------
    {
        auto J = model_->zeroJacobian(franka::Frame::kEndEffector, state);
        std::copy(J.begin(), J.end(), &dst->jacobian[0][0]);
    }
    
    // ------------------------------------------------------------------------
    // Kinematics: End effector body Jacobian (in EE frame)
    // ------------------------------------------------------------------------
    {
        auto J_body = model_->bodyJacobian(franka::Frame::kEndEffector, state);
        std::copy(J_body.begin(), J_body.end(), &dst->jacobian_body[0][0]);
    }
}
