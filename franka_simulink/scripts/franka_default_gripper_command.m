function cmd = franka_default_gripper_command()
%FRANKA_DEFAULT_GRIPPER_COMMAND Returns default gripper command parameters
%
%   cmd = FRANKA_DEFAULT_GRIPPER_COMMAND() returns a struct with default
%   values for the FrankaGripperCommandBus. This can be used as initial
%   values for the command bus input.
%
%   Grasp Parameters:
%     grasp_width         - 0.04 m (40mm, typical small object)
%     grasp_speed         - 0.05 m/s (moderate closing speed)
%     grasp_force         - 40 N (moderate grasping force)
%     grasp_epsilon_inner - 0.005 m (5mm inner tolerance)
%     grasp_epsilon_outer - 0.005 m (5mm outer tolerance)
%
%   Move Parameters:
%     move_width          - 0.08 m (80mm, fully open)
%     move_speed          - 0.1 m/s (fast movement)
%
%   Example:
%       % Create default command bus
%       cmd = franka_default_gripper_command();
%       
%       % Modify for specific use case
%       cmd.grasp_width = 0.02;  % Grasp smaller object
%       cmd.grasp_force = 60;    % Higher force
%
%   See also: franka_gripper_command_bus, franka_setup_bus
%
%   Copyright (c) 2025 Franka Robotics GmbH

    % ========================================================================
    % Grasp Command Parameters
    % ========================================================================
    
    % Target width for grasping [m]
    cmd.grasp_width = 0.04;
    
    % Closing speed [m/s]
    cmd.grasp_speed = 0.05;
    
    % Grasping force [N]
    % Range: 0.01 to 70 N (limited by gripper hardware)
    cmd.grasp_force = 40;
    
    % Inner tolerance for grasp detection [m]
    % Object is considered grasped if: (width - epsilon_inner) < actual < (width + epsilon_outer)
    cmd.grasp_epsilon_inner = 0.005;
    
    % Outer tolerance for grasp detection [m]
    cmd.grasp_epsilon_outer = 0.005;
    
    % ========================================================================
    % Move Command Parameters
    % ========================================================================
    
    % Target width for movement [m]
    % Max width is typically ~0.08m (depends on gripper fingers)
    cmd.move_width = 0.08;
    
    % Movement speed [m/s]
    cmd.move_speed = 0.1;
end
