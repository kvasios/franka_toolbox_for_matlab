function cmd = franka_default_vacuum_gripper_command()
%FRANKA_DEFAULT_VACUUM_GRIPPER_COMMAND Returns default vacuum gripper command parameters
%
%   cmd = FRANKA_DEFAULT_VACUUM_GRIPPER_COMMAND() returns a struct with default
%   values for the FrankaVacuumGripperCommandBus. This can be used as initial
%   values for the command bus input.
%
%   Vacuum Parameters:
%     vacuum_setpoint - 50 [10*mbar] (= 500 mbar, typical for light objects)
%     vacuum_timeout  - 5000 [ms] (5 seconds)
%     vacuum_profile  - 0 (P0 - default production profile)
%
%   Drop Off Parameters:
%     dropoff_timeout - 1000 [ms] (1 second)
%
%   Example:
%       % Create default command bus
%       cmd = franka_default_vacuum_gripper_command();
%       
%       % Modify for specific use case
%       cmd.vacuum_setpoint = 80;  % Higher vacuum for heavier objects
%       cmd.vacuum_timeout = 10000; % Longer timeout
%
%   Vacuum Setpoint Units:
%     The vacuum_setpoint is in units of [10*mbar].
%     Example values:
%       20 = 200 mbar (light vacuum, delicate objects)
%       50 = 500 mbar (moderate vacuum, typical use)
%       80 = 800 mbar (strong vacuum, heavy objects)
%
%   Production Profiles:
%     0 = P0 (default)
%     1 = P1
%     2 = P2
%     3 = P3
%     See cobot-pump manual for profile details.
%
%   See also: franka_vacuum_gripper_command_bus, franka_setup_bus
%
%   Copyright (c) 2025 Franka Robotics GmbH

    % ========================================================================
    % Vacuum Command Parameters
    % ========================================================================
    
    % Vacuum setpoint [10*mbar]
    % 50 = 500 mbar, suitable for most objects
    cmd.vacuum_setpoint = 50;
    
    % Vacuum timeout [ms]
    % Time allowed to establish vacuum
    cmd.vacuum_timeout = 5000;
    
    % Production setup profile
    % 0=P0, 1=P1, 2=P2, 3=P3 (see cobot-pump manual)
    cmd.vacuum_profile = int32(0);
    
    % ========================================================================
    % Drop Off Command Parameters
    % ========================================================================
    
    % Drop off timeout [ms]
    % Time allowed to release vacuum and drop object
    cmd.dropoff_timeout = 1000;
end
