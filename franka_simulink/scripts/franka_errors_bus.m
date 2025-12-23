function bus = franka_errors_bus()
%FRANKA_ERRORS_BUS Creates Simulink bus definition for franka::Errors
%
%   bus = FRANKA_ERRORS_BUS() returns a Simulink.Bus object that mirrors
%   the libfranka franka::Errors structure. This bus contains 41 boolean
%   error flags indicating various robot error conditions.
%
%   Usage:
%       % Create and register the bus in base workspace
%       FrankaErrorsBus = franka_errors_bus();
%       assignin('base', 'FrankaErrorsBus', FrankaErrorsBus);
%
%   This bus is used as a nested type within FrankaRobotStateBus for:
%     - current_errors: Current error state
%     - last_motion_errors: Errors that aborted the previous motion
%
%   All fields are boolean (0 or 1). A value of 1 indicates the error
%   condition is active.
%
%   See also: franka::Errors in libfranka documentation, franka_robot_state_bus
%
%   Copyright (c) 2025 Franka Robotics GmbH

    elems = Simulink.BusElement.empty;
    idx = 0;
    
    %% ====================================================================
    %% Position and Velocity Limit Violations
    %% ====================================================================
    
    idx = idx + 1;
    elems(idx) = create_element('joint_position_limits_violation', ...
        'Robot moved past joint limits');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_position_limits_violation', ...
        'Robot moved past virtual walls');
    
    idx = idx + 1;
    elems(idx) = create_element('self_collision_avoidance_violation', ...
        'Robot would have collided with itself');
    
    idx = idx + 1;
    elems(idx) = create_element('joint_velocity_violation', ...
        'Robot exceeded joint velocity limits');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_velocity_violation', ...
        'Robot exceeded Cartesian velocity limits');
    
    %% ====================================================================
    %% Force and Collision Detection
    %% ====================================================================
    
    idx = idx + 1;
    elems(idx) = create_element('force_control_safety_violation', ...
        'Robot exceeded safety threshold during force control');
    
    idx = idx + 1;
    elems(idx) = create_element('joint_reflex', ...
        'Collision detected: exceeded torque threshold in joint motion');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_reflex', ...
        'Collision detected: exceeded torque threshold in Cartesian motion');
    
    %% ====================================================================
    %% Internal Motion Generator Errors
    %% ====================================================================
    
    idx = idx + 1;
    elems(idx) = create_element('max_goal_pose_deviation_violation', ...
        'Internal motion generator did not reach goal pose');
    
    idx = idx + 1;
    elems(idx) = create_element('max_path_pose_deviation_violation', ...
        'Internal motion generator deviated from path');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_velocity_profile_safety_violation', ...
        'Cartesian velocity profile for internal motions exceeded');
    
    %% ====================================================================
    %% Joint Motion Generator Errors
    %% ====================================================================
    
    idx = idx + 1;
    elems(idx) = create_element('joint_position_motion_generator_start_pose_invalid', ...
        'External joint motion started with pose too far from current');
    
    idx = idx + 1;
    elems(idx) = create_element('joint_motion_generator_position_limits_violation', ...
        'External joint motion would move into joint limit');
    
    idx = idx + 1;
    elems(idx) = create_element('joint_motion_generator_velocity_limits_violation', ...
        'External joint motion exceeded velocity limits');
    
    idx = idx + 1;
    elems(idx) = create_element('joint_motion_generator_velocity_discontinuity', ...
        'Commanded joint velocity is discontinuous (values too far apart)');
    
    idx = idx + 1;
    elems(idx) = create_element('joint_motion_generator_acceleration_discontinuity', ...
        'Commanded joint acceleration is discontinuous (values too far apart)');
    
    %% ====================================================================
    %% Cartesian Motion Generator Errors
    %% ====================================================================
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_position_motion_generator_start_pose_invalid', ...
        'External Cartesian motion started with pose too far from current');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_motion_generator_elbow_limit_violation', ...
        'External Cartesian motion would move into elbow limit');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_motion_generator_velocity_limits_violation', ...
        'External Cartesian motion would exceed velocity limits');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_motion_generator_velocity_discontinuity', ...
        'Commanded Cartesian velocity is discontinuous (values too far apart)');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_motion_generator_acceleration_discontinuity', ...
        'Commanded Cartesian acceleration is discontinuous (values too far apart)');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_motion_generator_elbow_sign_inconsistent', ...
        'Commanded elbow values in Cartesian motion are inconsistent');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_motion_generator_start_elbow_invalid', ...
        'First elbow value in Cartesian motion too far from initial');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_motion_generator_joint_position_limits_violation', ...
        'Joint position limits would be exceeded after IK');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_motion_generator_joint_velocity_limits_violation', ...
        'Joint velocity limits would be exceeded after IK');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_motion_generator_joint_velocity_discontinuity', ...
        'Joint velocity is discontinuous after IK');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_motion_generator_joint_acceleration_discontinuity', ...
        'Joint acceleration is discontinuous after IK');
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_position_motion_generator_invalid_frame', ...
        'Cartesian pose is not a valid transformation matrix');
    
    %% ====================================================================
    %% Controller and Communication Errors
    %% ====================================================================
    
    idx = idx + 1;
    elems(idx) = create_element('force_controller_desired_force_tolerance_violation', ...
        'Desired force exceeds safety thresholds');
    
    idx = idx + 1;
    elems(idx) = create_element('controller_torque_discontinuity', ...
        'Torque set by external controller is discontinuous');
    
    idx = idx + 1;
    elems(idx) = create_element('start_elbow_sign_inconsistent', ...
        'Start elbow sign was inconsistent (Desk motions only)');
    
    idx = idx + 1;
    elems(idx) = create_element('communication_constraints_violation', ...
        'Minimum network communication quality not maintained');
    
    idx = idx + 1;
    elems(idx) = create_element('power_limit_violation', ...
        'Commanded values would exceed power limit');
    
    %% ====================================================================
    %% Planning and System Errors
    %% ====================================================================
    
    idx = idx + 1;
    elems(idx) = create_element('joint_p2p_insufficient_torque_for_planning', ...
        'Robot overloaded for required motion (Desk motions only)');
    
    idx = idx + 1;
    elems(idx) = create_element('tau_j_range_violation', ...
        'Measured torque signal out of safe range');
    
    idx = idx + 1;
    elems(idx) = create_element('instability_detected', ...
        'Instability detected');
    
    idx = idx + 1;
    elems(idx) = create_element('joint_move_in_wrong_direction', ...
        'In joint limit error, user guides robot further towards limit');
    
    %% ====================================================================
    %% Spline and Via Point Motion Errors
    %% ====================================================================
    
    idx = idx + 1;
    elems(idx) = create_element('cartesian_spline_motion_generator_violation', ...
        'Generated spline motion violates joint limit');
    
    idx = idx + 1;
    elems(idx) = create_element('joint_via_motion_generator_planning_joint_limit_violation', ...
        'Generated via-point motion violates joint limit');
    
    %% ====================================================================
    %% Base Acceleration Errors
    %% ====================================================================
    
    idx = idx + 1;
    elems(idx) = create_element('base_acceleration_initialization_timeout', ...
        'Gravity vector initialization by base acceleration timed out');
    
    idx = idx + 1;
    elems(idx) = create_element('base_acceleration_invalid_reading', ...
        'Base acceleration O_ddP_O cannot be determined');
    
    %% ====================================================================
    %% Create the Bus
    %% ====================================================================
    
    bus = Simulink.Bus();
    bus.Description = 'Robot error flags from libfranka (franka::Errors)';
    bus.Elements = elems;
    
    % Tell Simulink Coder to use our pre-defined C struct instead of generating one
    bus.HeaderFile = 'franka_simulink_types.h';
    bus.DataScope = 'Imported';
end

function elem = create_element(name, description)
%CREATE_ELEMENT Helper to create a boolean Simulink.BusElement
    elem = Simulink.BusElement();
    elem.Name = name;
    elem.Dimensions = [1 1];
    elem.DataType = 'boolean';
    elem.Description = description;
    elem.Complexity = 'real';
    elem.SamplingMode = 'Sample based';
end

