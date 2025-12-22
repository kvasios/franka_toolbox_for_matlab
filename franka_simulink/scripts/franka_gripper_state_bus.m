function bus = franka_gripper_state_bus()
%FRANKA_GRIPPER_STATE_BUS Creates Simulink bus definition for gripper state
%
%   bus = FRANKA_GRIPPER_STATE_BUS() returns a Simulink.Bus object that
%   contains the gripper state from libfranka plus command status fields.
%
%   Usage:
%       % Create and register the bus in base workspace
%       FrankaGripperStateBus = franka_gripper_state_bus();
%       assignin('base', 'FrankaGripperStateBus', FrankaGripperStateBus);
%
%   The bus contains:
%     - Gripper state (width, max_width, is_grasped, temperature, time)
%     - Command status (status, last_command, success, error_code)
%
%   See also: franka::GripperState in libfranka documentation
%
%   Copyright (c) 2025 Franka Robotics GmbH

    elems = Simulink.BusElement.empty;
    idx = 0;
    
    %% ====================================================================
    %% Gripper State (from franka::GripperState)
    %% ====================================================================
    
    % width: Current gripper opening width [m]
    idx = idx + 1;
    elems(idx) = create_element('width', [1 1], 'double', ...
        'Current gripper opening width [m]');
    
    % max_width: Maximum gripper opening width [m] (estimated by homing)
    idx = idx + 1;
    elems(idx) = create_element('max_width', [1 1], 'double', ...
        'Maximum gripper opening width [m] (from homing)');
    
    % is_grasped: Whether an object is currently grasped
    idx = idx + 1;
    elems(idx) = create_element('is_grasped', [1 1], 'double', ...
        'Whether an object is grasped (0=no, 1=yes)');
    
    % temperature: Current gripper temperature [°C]
    idx = idx + 1;
    elems(idx) = create_element('temperature', [1 1], 'double', ...
        'Gripper temperature [°C]');
    
    % time: Time since robot start [s]
    idx = idx + 1;
    elems(idx) = create_element('time', [1 1], 'double', ...
        'Time since robot start [s]');
    
    %% ====================================================================
    %% Command Status
    %% ====================================================================
    
    % command_status: Current status of command execution
    % 0=Idle, 1=Busy, 2=Success, 3=Failed, 4=Error
    idx = idx + 1;
    elems(idx) = create_element('command_status', [1 1], 'int32', ...
        'Command status (0=Idle,1=Busy,2=Success,3=Failed,4=Error)');
    
    % last_command: Last executed command
    % 0=None, 1=Homing, 2=Grasp, 3=Move, 4=Stop
    idx = idx + 1;
    elems(idx) = create_element('last_command', [1 1], 'int32', ...
        'Last command (0=None,1=Homing,2=Grasp,3=Move,4=Stop)');
    
    % command_success: Result of last command
    idx = idx + 1;
    elems(idx) = create_element('command_success', [1 1], 'int32', ...
        'Last command result (0=failed, 1=success)');
    
    % error_code: Error code if command_status == Error
    % 0=None, 1=Unsuccessful, 2=CommandException, 3=NetworkException, 4=Exception, 5=Other
    idx = idx + 1;
    elems(idx) = create_element('error_code', [1 1], 'int32', ...
        'Error code (0=None,1=Unsuccessful,2=CmdErr,3=NetErr,4=Except,5=Other)');
    
    %% ====================================================================
    %% Create the Bus
    %% ====================================================================
    
    bus = Simulink.Bus();
    bus.Description = 'Gripper state from libfranka with command status';
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
