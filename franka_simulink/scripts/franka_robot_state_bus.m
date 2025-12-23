function bus = franka_robot_state_bus()
%FRANKA_ROBOT_STATE_BUS Creates Simulink bus definition for franka::RobotState
%
%   bus = FRANKA_ROBOT_STATE_BUS() returns a Simulink.Bus object that mirrors
%   the libfranka franka::RobotState structure. This bus is used by the
%   Franka Robot block to output the complete robot state.
%
%   Usage:
%       % Create and register the bus in base workspace
%       FrankaRobotStateBus = franka_robot_state_bus();
%       assignin('base', 'FrankaRobotStateBus', FrankaRobotStateBus);
%
%   The bus contains all fields from franka::RobotState:
%     - Transformation matrices (4x4, column-major as 16x1 vectors)
%     - Joint-space signals (7x1 vectors)
%     - Cartesian-space signals (6x1 vectors)
%     - Inertial parameters
%     - Contact/collision detection
%     - External force estimates
%     - Status information
%
%   See also: franka::RobotState in libfranka documentation
%
%   Copyright (c) 2025 Franka Robotics GmbH

    elems = Simulink.BusElement.empty;
    idx = 0;
    
    %% ====================================================================
    %% Transformation Matrices (4x4 homogeneous transforms)
    %% ====================================================================
    
    % O_T_EE: Measured end effector pose in base frame
    idx = idx + 1;
    elems(idx) = create_element('O_T_EE', [4 4], 'double', ...
        'Measured end effector pose in base frame');
    
    % O_T_EE_d: Last desired end effector pose of motion generation
    idx = idx + 1;
    elems(idx) = create_element('O_T_EE_d', [4 4], 'double', ...
        'Desired end effector pose in base frame');
    
    % F_T_EE: End effector frame pose in flange frame
    idx = idx + 1;
    elems(idx) = create_element('F_T_EE', [4 4], 'double', ...
        'End effector pose in flange frame');
    
    % F_T_NE: Nominal end effector frame pose in flange frame
    idx = idx + 1;
    elems(idx) = create_element('F_T_NE', [4 4], 'double', ...
        'Nominal end effector pose in flange frame');
    
    % NE_T_EE: End effector frame pose in nominal end effector frame
    idx = idx + 1;
    elems(idx) = create_element('NE_T_EE', [4 4], 'double', ...
        'End effector pose in nominal EE frame');
    
    % EE_T_K: Stiffness frame pose in end effector frame
    idx = idx + 1;
    elems(idx) = create_element('EE_T_K', [4 4], 'double', ...
        'Stiffness frame pose in EE frame');
    
    % O_T_EE_c: Last commanded end effector pose
    idx = idx + 1;
    elems(idx) = create_element('O_T_EE_c', [4 4], 'double', ...
        'Commanded end effector pose in base frame');
    
    %% ====================================================================
    %% End Effector Inertial Parameters
    %% ====================================================================
    
    % m_ee: Configured mass of end effector [kg]
    idx = idx + 1;
    elems(idx) = create_element('m_ee', [1 1], 'double', ...
        'Configured end effector mass [kg]');
    
    % I_ee: Configured rotational inertia of end effector (3x3 matrix)
    idx = idx + 1;
    elems(idx) = create_element('I_ee', [3 3], 'double', ...
        'End effector rotational inertia matrix');
    
    % F_x_Cee: Center of mass of end effector in flange frame [m]
    idx = idx + 1;
    elems(idx) = create_element('F_x_Cee', [3 1], 'double', ...
        'End effector center of mass in flange frame [m]');
    
    %% ====================================================================
    %% External Load Inertial Parameters
    %% ====================================================================
    
    % m_load: Configured mass of external load [kg]
    idx = idx + 1;
    elems(idx) = create_element('m_load', [1 1], 'double', ...
        'Configured external load mass [kg]');
    
    % I_load: Configured rotational inertia of external load (3x3 matrix)
    idx = idx + 1;
    elems(idx) = create_element('I_load', [3 3], 'double', ...
        'External load rotational inertia matrix');
    
    % F_x_Cload: Center of mass of external load in flange frame [m]
    idx = idx + 1;
    elems(idx) = create_element('F_x_Cload', [3 1], 'double', ...
        'External load center of mass in flange frame [m]');
    
    %% ====================================================================
    %% Total (EE + Load) Inertial Parameters
    %% ====================================================================
    
    % m_total: Sum of end effector and external load mass [kg]
    idx = idx + 1;
    elems(idx) = create_element('m_total', [1 1], 'double', ...
        'Total mass (EE + load) [kg]');
    
    % I_total: Combined rotational inertia (3x3 matrix)
    idx = idx + 1;
    elems(idx) = create_element('I_total', [3 3], 'double', ...
        'Total rotational inertia matrix');
    
    % F_x_Ctotal: Combined center of mass in flange frame [m]
    idx = idx + 1;
    elems(idx) = create_element('F_x_Ctotal', [3 1], 'double', ...
        'Total center of mass in flange frame [m]');
    
    %% ====================================================================
    %% Elbow Configuration
    %% ====================================================================
    
    % elbow: Current elbow configuration [q3, flip_direction]
    idx = idx + 1;
    elems(idx) = create_element('elbow', [2 1], 'double', ...
        'Elbow configuration [q3 rad, flip_direction]');
    
    % elbow_d: Desired elbow configuration
    idx = idx + 1;
    elems(idx) = create_element('elbow_d', [2 1], 'double', ...
        'Desired elbow configuration [q3 rad, flip_direction]');
    
    % elbow_c: Commanded elbow configuration
    idx = idx + 1;
    elems(idx) = create_element('elbow_c', [2 1], 'double', ...
        'Commanded elbow configuration [q3 rad, flip_direction]');
    
    % delbow_c: Commanded elbow velocity
    idx = idx + 1;
    elems(idx) = create_element('delbow_c', [2 1], 'double', ...
        'Commanded elbow velocity [rad/s, 0]');
    
    % ddelbow_c: Commanded elbow acceleration
    idx = idx + 1;
    elems(idx) = create_element('ddelbow_c', [2 1], 'double', ...
        'Commanded elbow acceleration [rad/s^2, 0]');
    
    %% ====================================================================
    %% Joint-Space Signals (7 DOF)
    %% ====================================================================
    
    % tau_J: Measured link-side joint torque sensor signals [Nm]
    idx = idx + 1;
    elems(idx) = create_element('tau_J', [7 1], 'double', ...
        'Measured joint torques [Nm]');
    
    % tau_J_d: Desired link-side joint torques (without gravity) [Nm]
    idx = idx + 1;
    elems(idx) = create_element('tau_J_d', [7 1], 'double', ...
        'Desired joint torques (no gravity) [Nm]');
    
    % dtau_J: Derivative of measured joint torques [Nm/s]
    idx = idx + 1;
    elems(idx) = create_element('dtau_J', [7 1], 'double', ...
        'Joint torque derivatives [Nm/s]');
    
    % q: Measured joint position [rad]
    idx = idx + 1;
    elems(idx) = create_element('q', [7 1], 'double', ...
        'Measured joint positions [rad]');
    
    % q_d: Desired joint position [rad]
    idx = idx + 1;
    elems(idx) = create_element('q_d', [7 1], 'double', ...
        'Desired joint positions [rad]');
    
    % dq: Measured joint velocity [rad/s]
    idx = idx + 1;
    elems(idx) = create_element('dq', [7 1], 'double', ...
        'Measured joint velocities [rad/s]');
    
    % dq_d: Desired joint velocity [rad/s]
    idx = idx + 1;
    elems(idx) = create_element('dq_d', [7 1], 'double', ...
        'Desired joint velocities [rad/s]');
    
    % ddq_d: Desired joint acceleration [rad/s^2]
    idx = idx + 1;
    elems(idx) = create_element('ddq_d', [7 1], 'double', ...
        'Desired joint accelerations [rad/s^2]');
    
    % theta: Motor position [rad]
    idx = idx + 1;
    elems(idx) = create_element('theta', [7 1], 'double', ...
        'Motor positions [rad]');
    
    % dtheta: Motor velocity [rad/s]
    idx = idx + 1;
    elems(idx) = create_element('dtheta', [7 1], 'double', ...
        'Motor velocities [rad/s]');
    
    %% ====================================================================
    %% Contact and Collision Detection
    %% ====================================================================
    
    % joint_contact: Contact level per joint (resets after contact)
    idx = idx + 1;
    elems(idx) = create_element('joint_contact', [7 1], 'double', ...
        'Joint contact levels (auto-reset)');
    
    % cartesian_contact: Contact level per Cartesian DOF (x,y,z,R,P,Y)
    idx = idx + 1;
    elems(idx) = create_element('cartesian_contact', [6 1], 'double', ...
        'Cartesian contact levels (auto-reset)');
    
    % joint_collision: Collision level per joint (latched until reset)
    idx = idx + 1;
    elems(idx) = create_element('joint_collision', [7 1], 'double', ...
        'Joint collision levels (latched)');
    
    % cartesian_collision: Collision level per Cartesian DOF (latched)
    idx = idx + 1;
    elems(idx) = create_element('cartesian_collision', [6 1], 'double', ...
        'Cartesian collision levels (latched)');
    
    %% ====================================================================
    %% External Force Estimates
    %% ====================================================================
    
    % tau_ext_hat_filtered: External torques on joints [Nm]
    idx = idx + 1;
    elems(idx) = create_element('tau_ext_hat_filtered', [7 1], 'double', ...
        'Estimated external joint torques (filtered) [Nm]');
    
    % O_F_ext_hat_K: External wrench on stiffness frame in base frame [N,Nm]
    idx = idx + 1;
    elems(idx) = create_element('O_F_ext_hat_K', [6 1], 'double', ...
        'External wrench in base frame [N,N,N,Nm,Nm,Nm]');
    
    % K_F_ext_hat_K: External wrench on stiffness frame in K frame [N,Nm]
    idx = idx + 1;
    elems(idx) = create_element('K_F_ext_hat_K', [6 1], 'double', ...
        'External wrench in stiffness frame [N,N,N,Nm,Nm,Nm]');
    
    %% ====================================================================
    %% Cartesian Motion Signals
    %% ====================================================================
    
    % O_dP_EE_d: Desired end effector twist in base frame
    idx = idx + 1;
    elems(idx) = create_element('O_dP_EE_d', [6 1], 'double', ...
        'Desired EE twist in base frame [m/s, rad/s]');
    
    % O_ddP_O: Base acceleration (gravity direction if stationary)
    idx = idx + 1;
    elems(idx) = create_element('O_ddP_O', [3 1], 'double', ...
        'Base linear acceleration [m/s^2]');
    
    % O_dP_EE_c: Last commanded end effector twist
    idx = idx + 1;
    elems(idx) = create_element('O_dP_EE_c', [6 1], 'double', ...
        'Commanded EE twist in base frame [m/s, rad/s]');
    
    % O_ddP_EE_c: Last commanded end effector acceleration
    idx = idx + 1;
    elems(idx) = create_element('O_ddP_EE_c', [6 1], 'double', ...
        'Commanded EE acceleration in base frame [m/s^2, rad/s^2]');
    
    %% ====================================================================
    %% Error State (franka::Errors)
    %% ====================================================================
    
    % current_errors: Current error state (41 boolean flags)
    idx = idx + 1;
    elems(idx) = create_bus_element('current_errors', 'FrankaErrorsBus', ...
        'Current error state (41 boolean flags)');
    
    % last_motion_errors: Errors that aborted the previous motion (41 boolean flags)
    idx = idx + 1;
    elems(idx) = create_bus_element('last_motion_errors', 'FrankaErrorsBus', ...
        'Errors that aborted previous motion (41 boolean flags)');
    
    %% ====================================================================
    %% Status Signals
    %% ====================================================================
    
    % control_command_success_rate: Percentage of successful commands [0-1]
    idx = idx + 1;
    elems(idx) = create_element('control_command_success_rate', [1 1], 'double', ...
        'Control command success rate [0-1]');
    
    % robot_mode: Current robot mode (enum as int32)
    % 0=kOther, 1=kIdle, 2=kMove, 3=kGuiding, 4=kReflex, 5=kUserStopped, 6=kAutomaticErrorRecovery
    idx = idx + 1;
    elems(idx) = create_element('robot_mode', [1 1], 'int32', ...
        'Robot mode (0=Other,1=Idle,2=Move,3=Guiding,4=Reflex,5=UserStopped,6=AutoRecovery)');
    
    % time: Time since robot start [s]
    idx = idx + 1;
    elems(idx) = create_element('time', [1 1], 'double', ...
        'Time since robot start [s]');
    
    %% ====================================================================
    %% Create the Bus
    %% ====================================================================
    
    bus = Simulink.Bus();
    bus.Description = 'Complete robot state from libfranka (franka::RobotState)';
    bus.Elements = elems;
    
    % Tell Simulink Coder to use our pre-defined C struct instead of generating one
    % This prevents redefinition errors during code generation
    bus.HeaderFile = 'franka_simulink_types.h';
    bus.DataScope = 'Imported';  % Type is defined externally (not generated)
end

function elem = create_element(name, dims, datatype, description)
%CREATE_ELEMENT Helper to create a Simulink.BusElement
    elem = Simulink.BusElement();
    elem.Name = name;
    elem.Dimensions = dims;
    elem.DataType = datatype;
    elem.Description = description;
    elem.Complexity = 'real';
    elem.SamplingMode = 'Sample based';
end

function elem = create_bus_element(name, busType, description)
%CREATE_BUS_ELEMENT Helper to create a nested bus Simulink.BusElement
    elem = Simulink.BusElement();
    elem.Name = name;
    elem.Dimensions = [1 1];
    elem.DataType = ['Bus: ' busType];
    elem.Description = description;
    elem.Complexity = 'real';
    elem.SamplingMode = 'Sample based';
end
