function settings = franka_default_settings(robot_settings)
%FRANKA_DEFAULT_SETTINGS Create settings struct for FrankaRobotSettingsBus
%
%   settings = FRANKA_DEFAULT_SETTINGS() returns a struct with default values
%   for all robot settings, compatible with the FrankaRobotSettingsBus.
%
%   settings = FRANKA_DEFAULT_SETTINGS(robot_settings) converts a
%   FrankaRobotSettings object to a struct compatible with FrankaRobotSettingsBus.
%
%   The returned struct can be used with a Bus Creator block to provide
%   the settings input to the Franka Robot block.
%
%   Example:
%       % Use default settings
%       settings = franka_default_settings();
%
%       % Use custom settings
%       my_settings = FrankaRobotSettings;
%       my_settings.cutoff_frequency = 50.0;
%       settings = franka_default_settings(my_settings);
%
%   See also: FrankaRobotSettings, FrankaRobotSettingsBus, franka_setup_bus
%
%   Copyright (c) 2025 Franka Robotics GmbH

    if nargin < 1
        robot_settings = FrankaRobotSettings;
    end
    
    % Get collision thresholds
    ct = robot_settings.collision_thresholds;
    
    % Get load inertia
    li = robot_settings.load_inertia;
    
    % Build settings struct matching FrankaRobotSettingsBus element order
    settings = struct();
    
    % Control parameters
    settings.rate_limiter = double(robot_settings.rate_limiter);
    settings.cutoff_frequency = robot_settings.cutoff_frequency;
    
    % Impedance settings
    settings.joint_impedance_stiffness = robot_settings.joint_impedance_stiffness(:);
    settings.cartesian_impedance_stiffness = robot_settings.cartesian_impedance_stiffness(:);
    
    % End effector frames (4x4 matrices)
    settings.NE_T_EE = robot_settings.NE_T_EE;
    settings.EE_T_K = robot_settings.EE_T_K;
    
    % Collision thresholds - Torque
    settings.lower_torque_thresholds_acceleration = ct.lower_torque_thresholds_acceleration(:);
    settings.upper_torque_thresholds_acceleration = ct.upper_torque_thresholds_acceleration(:);
    settings.lower_torque_thresholds_nominal = ct.lower_torque_thresholds_nominal(:);
    settings.upper_torque_thresholds_nominal = ct.upper_torque_thresholds_nominal(:);
    
    % Collision thresholds - Force
    settings.lower_force_thresholds_acceleration = ct.lower_force_thresholds_acceleration(:);
    settings.upper_force_thresholds_acceleration = ct.upper_force_thresholds_acceleration(:);
    settings.lower_force_thresholds_nominal = ct.lower_force_thresholds_nominal(:);
    settings.upper_force_thresholds_nominal = ct.upper_force_thresholds_nominal(:);
    
    % Load inertia
    settings.load_mass = li.mass;
    settings.load_center_of_mass = li.center_of_mass(:);
    settings.load_inertia_matrix = li.inertia_matrix;
end
