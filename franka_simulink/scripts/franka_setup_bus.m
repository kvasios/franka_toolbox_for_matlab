function franka_setup_bus()
%FRANKA_SETUP_BUS Register FrankaRobotStateBus in base workspace
%
%   FRANKA_SETUP_BUS() creates the FrankaRobotStateBus Simulink.Bus object
%   and registers it in the base workspace. This must be called before
%   simulating or generating code for models that use the Franka Robot block.
%
%   Usage:
%       % Call once before opening your model
%       franka_setup_bus();
%
%       % Or add to your model's InitFcn callback:
%       % Model Properties > Callbacks > InitFcn: franka_setup_bus
%
%   The bus type will be available as 'FrankaRobotStateBus' in the base
%   workspace and can be used with Bus Selector blocks to extract specific
%   signals from the robot state.
%
%   Example (in a Simulink model):
%       1. Add Franka Robot block
%       2. Connect robot_state output to a Bus Selector
%       3. In Bus Selector, select signals: q, dq, tau_J, O_T_EE, etc.
%
%   See also: franka_robot_state_bus, Simulink.Bus
%
%   Copyright (c) 2025 Franka Robotics GmbH

    % Check if bus already exists and matches
    if evalin('base', 'exist(''FrankaRobotStateBus'', ''var'')')
        existing = evalin('base', 'FrankaRobotStateBus');
        if isa(existing, 'Simulink.Bus')
            % Bus exists, check if it needs updating
            newBus = franka_robot_state_bus();
            if isequal(length(existing.Elements), length(newBus.Elements))
                fprintf('FrankaRobotStateBus already registered (%d elements)\n', ...
                    length(existing.Elements));
                return;
            end
        end
    end
    
    % Create and register the bus
    FrankaRobotStateBus = franka_robot_state_bus(); %#ok<NASGU>
    assignin('base', 'FrankaRobotStateBus', FrankaRobotStateBus);
    
    fprintf('Registered FrankaRobotStateBus in base workspace (%d elements)\n', ...
        length(franka_robot_state_bus().Elements));
end
