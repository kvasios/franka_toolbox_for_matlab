function bus = franka_vacuum_gripper_state_bus()
%FRANKA_VACUUM_GRIPPER_STATE_BUS Creates Simulink bus definition for vacuum gripper state
%
%   bus = FRANKA_VACUUM_GRIPPER_STATE_BUS() returns a Simulink.Bus object that
%   contains the vacuum gripper state from libfranka plus command status fields.
%
%   Usage:
%       FrankaVacuumGripperStateBus = franka_vacuum_gripper_state_bus();
%       assignin('base', 'FrankaVacuumGripperStateBus', FrankaVacuumGripperStateBus);
%
%   The bus contains:
%     - Vacuum gripper state (in_control_range, part_detached, part_present,
%       device_status, actual_power, vacuum, time)
%     - Command status (status, last_command, success, error_code)
%
%   See also: franka::VacuumGripperState in libfranka documentation
%
%   Copyright (c) 2025 Franka Robotics GmbH

    elems = Simulink.BusElement.empty;
    idx = 0;
    
    %% ====================================================================
    %% Vacuum Gripper State (from franka::VacuumGripperState)
    %% ====================================================================
    
    % in_control_range: Vacuum within setpoint area
    idx = idx + 1;
    elems(idx) = create_element('in_control_range', [1 1], 'double', ...
        'Vacuum within setpoint area (0=no, 1=yes)');
    
    % part_detached: Part detached after suction cycle
    idx = idx + 1;
    elems(idx) = create_element('part_detached', [1 1], 'double', ...
        'Part detached after suction cycle (0=no, 1=yes)');
    
    % part_present: Part is present/gripped
    idx = idx + 1;
    elems(idx) = create_element('part_present', [1 1], 'double', ...
        'Part is present/gripped (0=no, 1=yes)');
    
    % device_status: Device status
    % 0=Green (optimal), 1=Yellow (warnings), 2=Orange (severe), 3=Red (error)
    idx = idx + 1;
    elems(idx) = create_element('device_status', [1 1], 'int32', ...
        'Device status (0=Green,1=Yellow,2=Orange,3=Red)');
    
    % actual_power: Current power consumption [%]
    idx = idx + 1;
    elems(idx) = create_element('actual_power', [1 1], 'double', ...
        'Current power consumption [%]');
    
    % vacuum: Current vacuum level [mbar]
    idx = idx + 1;
    elems(idx) = create_element('vacuum', [1 1], 'double', ...
        'Current vacuum level [mbar]');
    
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
    % 0=None, 1=Vacuum, 2=DropOff, 3=Stop
    idx = idx + 1;
    elems(idx) = create_element('last_command', [1 1], 'int32', ...
        'Last command (0=None,1=Vacuum,2=DropOff,3=Stop)');
    
    % command_success: Result of last command
    idx = idx + 1;
    elems(idx) = create_element('command_success', [1 1], 'int32', ...
        'Last command result (0=failed, 1=success)');
    
    % error_code: Error code if command_status == Error
    idx = idx + 1;
    elems(idx) = create_element('error_code', [1 1], 'int32', ...
        'Error code (0=None,1=Unsuccessful,2=CmdErr,3=NetErr,4=Except,5=Other)');
    
    %% ====================================================================
    %% Create the Bus
    %% ====================================================================
    
    bus = Simulink.Bus();
    bus.Description = 'Vacuum gripper state from libfranka with command status';
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
