function franka_setup_bus()
%FRANKA_SETUP_BUS Register Franka bus types in base workspace
%
%   FRANKA_SETUP_BUS() creates the Simulink.Bus objects for Franka Robot
%   and Gripper block inputs and outputs, registering them in the base
%   workspace. This must be called before simulating or generating code
%   for models that use the Franka Robot or Gripper blocks.
%
%   Registered bus types:
%     Robot:
%       - FrankaRobotStateBus: Complete robot state from libfranka (output)
%       - FrankaErrorsBus: Error flags (nested in FrankaRobotStateBus)
%       - FrankaModelDataBus: Computed dynamics/kinematics M, c, g, J (output)
%       - FrankaRobotSettingsBus: Robot configuration settings (input)
%
%   Related enumerations (for Simulink Data Type Conversion blocks):
%       - FrankaConnectionStatus: Connection lifecycle (Disconnected, Connected, etc.)
%       - FrankaConnectionErrorCode: Error codes (None, Network, Control, etc.)
%     Gripper (Finger):
%       - FrankaGripperStateBus: Gripper state with command status (output)
%       - FrankaGripperCommandBus: Gripper command parameters (input)
%     Vacuum Gripper:
%       - FrankaVacuumGripperStateBus: Vacuum gripper state with command status (output)
%       - FrankaVacuumGripperCommandBus: Vacuum gripper command parameters (input)
%
%   Usage:
%       % Call once before opening your model
%       franka_setup_bus();
%
%       % Or add to your model's InitFcn callback:
%       % Model Properties > Callbacks > InitFcn: franka_setup_bus
%
%   Example (Robot block):
%       1. Add Franka Robot block
%       2. Connect settings input from a Bus Creator with FrankaRobotSettings
%       3. Connect robot_state output to a Bus Selector
%       4. In Bus Selector, select signals: q, dq, tau_J, O_T_EE, etc.
%       5. Connect model_data output for dynamics: mass, coriolis, gravity, jacobian
%
%   Example (Gripper block):
%       1. Add Franka Gripper block
%       2. Connect command bus input with grasp/move parameters
%       3. Connect trigger signals (homing, grasp, move, stop)
%       4. Connect gripper_state output to monitor width, is_grasped, status
%
%   See also: franka_robot_state_bus, franka_errors_bus, franka_model_data_bus, 
%             franka_robot_settings_bus, franka_gripper_state_bus,
%             franka_gripper_command_bus, FrankaConnectionStatus,
%             FrankaConnectionErrorCode, Simulink.Bus
%
%   Copyright (c) 2025 Franka Robotics GmbH

    fprintf('Registering Franka bus types...\n');
    
    % ========================================================================
    % ROBOT BUSES
    % ========================================================================
    
    % Register FrankaRobotSettingsBus (Input)
    settingsBus = franka_robot_settings_bus();
    assignin('base', 'FrankaRobotSettingsBus', settingsBus);
    fprintf('  FrankaRobotSettingsBus (%d elements)\n', length(settingsBus.Elements));
    
    % Register FrankaErrorsBus (nested bus for error flags)
    % NOTE: Must be registered BEFORE FrankaRobotStateBus since it's nested
    errorsBus = franka_errors_bus();
    assignin('base', 'FrankaErrorsBus', errorsBus);
    fprintf('  FrankaErrorsBus (%d error flags)\n', length(errorsBus.Elements));
    
    % Register FrankaRobotStateBus (Output)
    % Contains nested FrankaErrorsBus for current_errors and last_motion_errors
    stateBus = franka_robot_state_bus();
    assignin('base', 'FrankaRobotStateBus', stateBus);
    fprintf('  FrankaRobotStateBus (%d elements)\n', length(stateBus.Elements));
    
    % Register FrankaModelDataBus (Output)
    modelBus = franka_model_data_bus();
    assignin('base', 'FrankaModelDataBus', modelBus);
    fprintf('  FrankaModelDataBus (%d elements)\n', length(modelBus.Elements));
    
    % ========================================================================
    % GRIPPER BUSES
    % ========================================================================
    
    % Register FrankaGripperCommandBus (Input)
    gripperCommandBus = franka_gripper_command_bus();
    assignin('base', 'FrankaGripperCommandBus', gripperCommandBus);
    fprintf('  FrankaGripperCommandBus (%d elements)\n', length(gripperCommandBus.Elements));
    
    % Register FrankaGripperStateBus (Output)
    gripperStateBus = franka_gripper_state_bus();
    assignin('base', 'FrankaGripperStateBus', gripperStateBus);
    fprintf('  FrankaGripperStateBus (%d elements)\n', length(gripperStateBus.Elements));
    
    % ========================================================================
    % VACUUM GRIPPER BUSES
    % ========================================================================
    
    % Register FrankaVacuumGripperCommandBus (Input)
    vacuumCommandBus = franka_vacuum_gripper_command_bus();
    assignin('base', 'FrankaVacuumGripperCommandBus', vacuumCommandBus);
    fprintf('  FrankaVacuumGripperCommandBus (%d elements)\n', length(vacuumCommandBus.Elements));
    
    % Register FrankaVacuumGripperStateBus (Output)
    vacuumStateBus = franka_vacuum_gripper_state_bus();
    assignin('base', 'FrankaVacuumGripperStateBus', vacuumStateBus);
    fprintf('  FrankaVacuumGripperStateBus (%d elements)\n', length(vacuumStateBus.Elements));
    
    fprintf('Done! All Franka bus types registered.\n');
end
