function bus = franka_vacuum_gripper_command_bus()
%FRANKA_VACUUM_GRIPPER_COMMAND_BUS Creates Simulink bus definition for vacuum gripper commands
%
%   bus = FRANKA_VACUUM_GRIPPER_COMMAND_BUS() returns a Simulink.Bus object
%   that contains all parameters for vacuum gripper commands.
%
%   Usage:
%       FrankaVacuumGripperCommandBus = franka_vacuum_gripper_command_bus();
%       assignin('base', 'FrankaVacuumGripperCommandBus', FrankaVacuumGripperCommandBus);
%
%   The bus contains parameters for:
%     - Vacuum command: setpoint, timeout, profile
%     - Drop off command: timeout
%
%   Command parameters are read from this bus when the corresponding
%   trigger input (vacuum, drop_off, stop) has a rising edge.
%
%   See also: franka::VacuumGripper::vacuum, franka::VacuumGripper::dropOff
%
%   Copyright (c) 2025 Franka Robotics GmbH

    elems = Simulink.BusElement.empty;
    idx = 0;
    
    %% ====================================================================
    %% Vacuum Command Parameters
    %% These are read when the 'vacuum' trigger has a rising edge
    %% ====================================================================
    
    % vacuum_setpoint: Vacuum setpoint in [10*mbar]
    % Example: 50 means 500 mbar vacuum
    idx = idx + 1;
    elems(idx) = create_element('vacuum_setpoint', [1 1], 'double', ...
        'Vacuum setpoint [10*mbar] (e.g., 50 = 500 mbar)');
    
    % vacuum_timeout: Timeout for vacuum operation [ms]
    idx = idx + 1;
    elems(idx) = create_element('vacuum_timeout', [1 1], 'double', ...
        'Vacuum timeout [ms]');
    
    % vacuum_profile: Production setup profile
    % 0=P0, 1=P1, 2=P2, 3=P3
    idx = idx + 1;
    elems(idx) = create_element('vacuum_profile', [1 1], 'int32', ...
        'Production profile (0=P0, 1=P1, 2=P2, 3=P3)');
    
    %% ====================================================================
    %% Drop Off Command Parameters
    %% These are read when the 'drop_off' trigger has a rising edge
    %% ====================================================================
    
    % dropoff_timeout: Timeout for drop off operation [ms]
    idx = idx + 1;
    elems(idx) = create_element('dropoff_timeout', [1 1], 'double', ...
        'Drop off timeout [ms]');
    
    %% ====================================================================
    %% Create the Bus
    %% ====================================================================
    
    bus = Simulink.Bus();
    bus.Description = 'Vacuum gripper command parameters for vacuum and drop off operations';
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
