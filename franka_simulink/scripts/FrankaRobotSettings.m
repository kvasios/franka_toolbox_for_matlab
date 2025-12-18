classdef FrankaRobotSettings
    % FRANKAROBOTSETTINGS Robot configuration settings for Franka Robot block
    %
    %   This class contains all configurable robot settings including:
    %     - Connection settings (robot_ip)
    %     - Control parameters (rate_limiter, cutoff_frequency)
    %     - Impedance settings (joint and Cartesian stiffness)
    %     - End effector frames (NE_T_EE, EE_T_K)
    %     - Collision thresholds
    %     - Load inertia
    %
    %   Example:
    %       settings = FrankaRobotSettings;
    %       settings.robot_ip = '192.168.1.100';
    %       settings.cutoff_frequency = 50.0;
    %
    %       % Convert to bus-compatible struct for Simulink
    %       bus_struct = settings.toBusStruct();
    %
    %   Copyright (c) 2023 Franka Robotics GmbH - All Rights Reserved
    %   This file is subject to the terms and conditions defined in the file
    %   'LICENSE', which is part of this package
    
    properties
        robot_ip = '172.16.0.2'
        collision_thresholds = FrankaRobotCollisionThresholds
        home_configuration = [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]
        home_configuration_O_T_EE = [0.707 -0.707 -0.0  0.3071;...
                                     -0.707 -0.707 -0.0 -0.0;...
                                     -0.0    0.0    -1    0.59;...
                                      0      0      0    1]'
        rate_limiter = true
        cutoff_frequency = 100.0
        joint_impedance_stiffness = [3000, 3000, 3000, 2500, 2500, 2000, 2000]
        cartesian_impedance_stiffness = [3000, 3000, 3000, 300, 300, 300]
        % Flange-to-end-effector transformation `F_T_EE` is split up into two transformations:
        % `F_T_NE`, only settable in Desk, and `NE_T_EE`
        NE_T_EE = [1 0 0 0;...
                   0 1 0 0;...
                   0 0 1 0;...
                   0 0 0 1]  % Transformation from nominal end effector to end effector frame
        EE_T_K = [1 0 0 0;...
                  0 1 0 0;...
                  0 0 1 0;...
                  0 0 0 1]  % Transformation from end effector frame to stiffness frame
        load_inertia = FrankaRobotLoadInertia
    end
    
    methods
        function bus_struct = toBusStruct(obj)
            %TOBUSSTRUCT Convert settings to FrankaRobotSettingsBus-compatible struct
            %
            %   bus_struct = toBusStruct(obj) returns a struct that matches the
            %   FrankaRobotSettingsBus layout. Use this with a Constant block
            %   or Bus Creator to provide settings to the Franka Robot block.
            %
            %   Example:
            %       settings = FrankaRobotSettings;
            %       bus_struct = settings.toBusStruct();
            %       % Use bus_struct with a Constant block (Output data type: Bus: FrankaRobotSettingsBus)
            
            ct = obj.collision_thresholds;
            li = obj.load_inertia;
            
            bus_struct = struct();
            
            % Control parameters
            bus_struct.rate_limiter = double(obj.rate_limiter);
            bus_struct.cutoff_frequency = obj.cutoff_frequency;
            
            % Impedance settings
            bus_struct.joint_impedance_stiffness = obj.joint_impedance_stiffness(:);
            bus_struct.cartesian_impedance_stiffness = obj.cartesian_impedance_stiffness(:);
            
            % End effector frames
            bus_struct.NE_T_EE = obj.NE_T_EE;
            bus_struct.EE_T_K = obj.EE_T_K;
            
            % Collision thresholds - Torque
            bus_struct.lower_torque_thresholds_acceleration = ct.lower_torque_thresholds_acceleration(:);
            bus_struct.upper_torque_thresholds_acceleration = ct.upper_torque_thresholds_acceleration(:);
            bus_struct.lower_torque_thresholds_nominal = ct.lower_torque_thresholds_nominal(:);
            bus_struct.upper_torque_thresholds_nominal = ct.upper_torque_thresholds_nominal(:);
            
            % Collision thresholds - Force
            bus_struct.lower_force_thresholds_acceleration = ct.lower_force_thresholds_acceleration(:);
            bus_struct.upper_force_thresholds_acceleration = ct.upper_force_thresholds_acceleration(:);
            bus_struct.lower_force_thresholds_nominal = ct.lower_force_thresholds_nominal(:);
            bus_struct.upper_force_thresholds_nominal = ct.upper_force_thresholds_nominal(:);
            
            % Load inertia
            bus_struct.load_mass = li.mass;
            bus_struct.load_center_of_mass = li.center_of_mass(:);
            bus_struct.load_inertia_matrix = li.inertia_matrix;
        end
    end
end