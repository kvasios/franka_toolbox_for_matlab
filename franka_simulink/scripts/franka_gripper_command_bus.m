function bus = franka_gripper_command_bus()
%FRANKA_GRIPPER_COMMAND_BUS Creates Simulink bus definition for gripper commands
%
%   bus = FRANKA_GRIPPER_COMMAND_BUS() returns a Simulink.Bus object that
%   contains all parameters for gripper commands.
%
%   Usage:
%       % Create and register the bus in base workspace
%       FrankaGripperCommandBus = franka_gripper_command_bus();
%       assignin('base', 'FrankaGripperCommandBus', FrankaGripperCommandBus);
%
%   The bus contains parameters for:
%     - Grasp command: width, speed, force, epsilon_inner, epsilon_outer
%     - Move command: width, speed
%
%   Command parameters are read from this bus when the corresponding
%   trigger input (homing, grasp, move, stop) has a rising edge.
%
%   See also: franka::Gripper::grasp, franka::Gripper::move
%
%   Copyright (c) 2025 Franka Robotics GmbH

    elems = Simulink.BusElement.empty;
    idx = 0;
    
    %% ====================================================================
    %% Grasp Command Parameters
    %% These are read when the 'grasp' trigger has a rising edge
    %% ====================================================================
    
    % grasp_width: Target grasp width [m]
    % Size of the object to grasp
    idx = idx + 1;
    elems(idx) = create_element('grasp_width', [1 1], 'double', ...
        'Target grasp width [m]');
    
    % grasp_speed: Closing speed [m/s]
    idx = idx + 1;
    elems(idx) = create_element('grasp_speed', [1 1], 'double', ...
        'Closing speed [m/s]');
    
    % grasp_force: Grasping force [N]
    idx = idx + 1;
    elems(idx) = create_element('grasp_force', [1 1], 'double', ...
        'Grasping force [N]');
    
    % grasp_epsilon_inner: Inner tolerance for grasp detection [m]
    % Maximum tolerated deviation when actual width is smaller than commanded
    idx = idx + 1;
    elems(idx) = create_element('grasp_epsilon_inner', [1 1], 'double', ...
        'Inner tolerance for grasp detection [m] (default 0.005)');
    
    % grasp_epsilon_outer: Outer tolerance for grasp detection [m]
    % Maximum tolerated deviation when actual width is larger than commanded
    idx = idx + 1;
    elems(idx) = create_element('grasp_epsilon_outer', [1 1], 'double', ...
        'Outer tolerance for grasp detection [m] (default 0.005)');
    
    %% ====================================================================
    %% Move Command Parameters
    %% These are read when the 'move' trigger has a rising edge
    %% ====================================================================
    
    % move_width: Target opening width [m]
    idx = idx + 1;
    elems(idx) = create_element('move_width', [1 1], 'double', ...
        'Target move width [m]');
    
    % move_speed: Movement speed [m/s]
    idx = idx + 1;
    elems(idx) = create_element('move_speed', [1 1], 'double', ...
        'Movement speed [m/s]');
    
    %% ====================================================================
    %% Create the Bus
    %% ====================================================================
    
    bus = Simulink.Bus();
    bus.Description = 'Gripper command parameters for grasp and move operations';
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
