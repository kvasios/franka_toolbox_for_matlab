function franka_setup_bus()
%FRANKA_SETUP_BUS Register Franka bus types in base workspace
%
%   FRANKA_SETUP_BUS() creates the Simulink.Bus objects for Franka Robot
%   block outputs and registers them in the base workspace. This must be
%   called before simulating or generating code for models that use the
%   Franka Robot block.
%
%   Registered bus types:
%     - FrankaRobotStateBus: Complete robot state from libfranka
%     - FrankaModelDataBus: Computed dynamics/kinematics (M, c, g, J)
%
%   Usage:
%       % Call once before opening your model
%       franka_setup_bus();
%
%       % Or add to your model's InitFcn callback:
%       % Model Properties > Callbacks > InitFcn: franka_setup_bus
%
%   Example (in a Simulink model):
%       1. Add Franka Robot block
%       2. Connect robot_state output to a Bus Selector
%       3. In Bus Selector, select signals: q, dq, tau_J, O_T_EE, etc.
%       4. Connect model_data output for dynamics: mass, coriolis, gravity, jacobian
%
%   See also: franka_robot_state_bus, franka_model_data_bus, Simulink.Bus
%
%   Copyright (c) 2025 Franka Robotics GmbH

    % ========================================================================
    % Register FrankaRobotStateBus
    % ========================================================================
    stateBus = franka_robot_state_bus();
    assignin('base', 'FrankaRobotStateBus', stateBus);
    fprintf('Registered FrankaRobotStateBus (%d elements)\n', length(stateBus.Elements));
    
    % ========================================================================
    % Register FrankaModelDataBus
    % ========================================================================
    modelBus = franka_model_data_bus();
    assignin('base', 'FrankaModelDataBus', modelBus);
    fprintf('Registered FrankaModelDataBus (%d elements)\n', length(modelBus.Elements));
end
