function bus = franka_robot_settings_bus()
%FRANKA_ROBOT_SETTINGS_BUS Creates Simulink bus definition for robot settings
%
%   bus = FRANKA_ROBOT_SETTINGS_BUS() returns a Simulink.Bus object that
%   contains all configurable robot settings. This bus is used by the
%   Franka Robot block as an input to configure the robot on enable.
%
%   Usage:
%       % Create and register the bus in base workspace
%       FrankaRobotSettingsBus = franka_robot_settings_bus();
%       assignin('base', 'FrankaRobotSettingsBus', FrankaRobotSettingsBus);
%
%   The bus contains:
%     - Control parameters (rate_limiter, cutoff_frequency)
%     - Impedance settings (joint and Cartesian stiffness)
%     - End effector frames (NE_T_EE, EE_T_K)
%     - Collision thresholds (torque and force)
%     - Load inertia (mass, center_of_mass, inertia_matrix)
%
%   Settings are applied when the Enable input transitions from 0 to 1.
%
%   See also: FrankaRobotSettings, franka_setup_bus
%
%   Copyright (c) 2025 Franka Robotics GmbH

    elems = Simulink.BusElement.empty;
    idx = 0;
    
    %% ====================================================================
    %% Control Parameters
    %% ====================================================================
    
    % rate_limiter: Enable rate limiting in libfranka
    idx = idx + 1;
    elems(idx) = create_element('rate_limiter', [1 1], 'double', ...
        'Enable rate limiting (0=false, 1=true)');
    
    % cutoff_frequency: Low-pass filter cutoff frequency [Hz]
    idx = idx + 1;
    elems(idx) = create_element('cutoff_frequency', [1 1], 'double', ...
        'Low-pass filter cutoff frequency [Hz]');
    
    %% ====================================================================
    %% Impedance Settings
    %% ====================================================================
    
    % joint_impedance_stiffness: Joint stiffness for internal impedance controller
    idx = idx + 1;
    elems(idx) = create_element('joint_impedance_stiffness', [7 1], 'double', ...
        'Joint impedance stiffness [Nm/rad]');
    
    % cartesian_impedance_stiffness: Cartesian stiffness [x,y,z,R,P,Y]
    idx = idx + 1;
    elems(idx) = create_element('cartesian_impedance_stiffness', [6 1], 'double', ...
        'Cartesian impedance stiffness [N/m, Nm/rad]');
    
    %% ====================================================================
    %% End Effector Frame Configuration
    %% ====================================================================
    
    % NE_T_EE: Transformation from nominal EE to EE frame (4x4 col-major)
    idx = idx + 1;
    elems(idx) = create_element('NE_T_EE', [4 4], 'double', ...
        'Nominal EE to EE transformation (4x4)');
    
    % EE_T_K: Transformation from EE to stiffness frame (4x4 col-major)
    idx = idx + 1;
    elems(idx) = create_element('EE_T_K', [4 4], 'double', ...
        'EE to stiffness frame transformation (4x4)');
    
    %% ====================================================================
    %% Collision Thresholds - Torque (7 joints)
    %% ====================================================================
    
    % lower_torque_thresholds_acceleration: Lower torque threshold during accel [Nm]
    idx = idx + 1;
    elems(idx) = create_element('lower_torque_thresholds_acceleration', [7 1], 'double', ...
        'Lower torque threshold during acceleration [Nm]');
    
    % upper_torque_thresholds_acceleration: Upper torque threshold during accel [Nm]
    idx = idx + 1;
    elems(idx) = create_element('upper_torque_thresholds_acceleration', [7 1], 'double', ...
        'Upper torque threshold during acceleration [Nm]');
    
    % lower_torque_thresholds_nominal: Lower torque threshold nominal [Nm]
    idx = idx + 1;
    elems(idx) = create_element('lower_torque_thresholds_nominal', [7 1], 'double', ...
        'Lower torque threshold nominal [Nm]');
    
    % upper_torque_thresholds_nominal: Upper torque threshold nominal [Nm]
    idx = idx + 1;
    elems(idx) = create_element('upper_torque_thresholds_nominal', [7 1], 'double', ...
        'Upper torque threshold nominal [Nm]');
    
    %% ====================================================================
    %% Collision Thresholds - Force (6 Cartesian DOF)
    %% ====================================================================
    
    % lower_force_thresholds_acceleration: Lower force threshold during accel [N]
    idx = idx + 1;
    elems(idx) = create_element('lower_force_thresholds_acceleration', [6 1], 'double', ...
        'Lower force threshold during acceleration [N]');
    
    % upper_force_thresholds_acceleration: Upper force threshold during accel [N]
    idx = idx + 1;
    elems(idx) = create_element('upper_force_thresholds_acceleration', [6 1], 'double', ...
        'Upper force threshold during acceleration [N]');
    
    % lower_force_thresholds_nominal: Lower force threshold nominal [N]
    idx = idx + 1;
    elems(idx) = create_element('lower_force_thresholds_nominal', [6 1], 'double', ...
        'Lower force threshold nominal [N]');
    
    % upper_force_thresholds_nominal: Upper force threshold nominal [N]
    idx = idx + 1;
    elems(idx) = create_element('upper_force_thresholds_nominal', [6 1], 'double', ...
        'Upper force threshold nominal [N]');
    
    %% ====================================================================
    %% Load Inertia Parameters
    %% ====================================================================
    
    % load_mass: Mass of external load attached to end effector [kg]
    idx = idx + 1;
    elems(idx) = create_element('load_mass', [1 1], 'double', ...
        'External load mass [kg]');
    
    % load_center_of_mass: Center of mass in flange frame [m]
    idx = idx + 1;
    elems(idx) = create_element('load_center_of_mass', [3 1], 'double', ...
        'External load center of mass in flange frame [m]');
    
    % load_inertia_matrix: Rotational inertia matrix (3x3 row-major as 9x1)
    idx = idx + 1;
    elems(idx) = create_element('load_inertia_matrix', [3 3], 'double', ...
        'External load rotational inertia matrix [kg*m^2]');
    
    %% ====================================================================
    %% Create the Bus
    %% ====================================================================
    
    bus = Simulink.Bus();
    bus.Description = 'Robot settings for Franka Robot block';
    bus.Elements = elems;
    
    % Tell Simulink Coder to use our pre-defined C struct
    bus.HeaderFile = 'franka_simulink_types.h';
    bus.DataScope = 'Imported';
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
