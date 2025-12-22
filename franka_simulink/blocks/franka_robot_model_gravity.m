function franka_robot_model_gravity(block)
%FRANKA_ROBOT_MODEL_GRAVITY Compute gravity vector g(q) using libfranka Model
%
%  This block computes the gravity compensation torques from explicit inputs
%  using the robot's kinematic/dynamic model. It integrates with FrankaRobotManager
%  to share the robot connection with other blocks.
%
%  Parameters:
%    robot_ip           - IP address of the Franka robot (for model lookup)
%    use_custom_gravity - 0 = use default gravity {0,0,-9.81}, 1 = use input
%
%  Inputs (use_custom_gravity = 0):
%    q          - Joint positions [rad] (7x1)
%    m_total    - Total load mass [kg] (1x1)
%    F_x_Ctotal - Total load CoM in flange frame [m] (3x1)
%
%  Inputs (use_custom_gravity = 1):
%    q             - Joint positions [rad] (7x1)
%    m_total       - Total load mass [kg] (1x1)
%    F_x_Ctotal    - Total load CoM in flange frame [m] (3x1)
%    gravity_earth - Custom gravity vector [m/s^2] (3x1)
%
%  Outputs:
%    gravity - Gravity compensation torques g(q) [Nm] (7x1)
%
%  Usage Notes:
%    - For current robot state model data, use the model_data output from
%      franka_robot block instead (FrankaModelDataBus.gravity)
%    - This block is for computing model data from arbitrary configurations
%    - Works both standalone AND under function-called subsystems
%
%  Copyright (c) 2025 Franka Robotics GmbH
%  This file is subject to the terms and conditions defined in the file
%  'LICENSE', which is part of this package

setup(block);

function setup(block)

    % Register parameters: robot_ip, use_custom_gravity
    block.NumDialogPrms     = 2;
    block.DialogPrmsTunable = {'Nontunable', 'Nontunable'};
    
    use_custom_gravity = boolean(block.DialogPrm(2).Data);
    
    % Determine number of inputs
    if use_custom_gravity
        numInputs = 4;  % q, m_total, F_x_Ctotal, gravity_earth
    else
        numInputs = 3;  % q, m_total, F_x_Ctotal
    end
    
    % Register number of ports
    block.NumInputPorts  = numInputs;
    block.NumOutputPorts = 1;

    % Setup port properties to be inherited or dynamic
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    % Input 1: q - Joint positions (7x1)
    block.InputPort(1).Dimensions  = 7;
    block.InputPort(1).DatatypeID  = 0;  % double
    block.InputPort(1).Complexity  = 'Real';
    block.InputPort(1).DirectFeedthrough = true;

    % Input 2: m_total - Total load mass (1x1)
    block.InputPort(2).Dimensions  = 1;
    block.InputPort(2).DatatypeID  = 0;  % double
    block.InputPort(2).Complexity  = 'Real';
    block.InputPort(2).DirectFeedthrough = true;

    % Input 3: F_x_Ctotal - Total load CoM (3x1)
    block.InputPort(3).Dimensions  = 3;
    block.InputPort(3).DatatypeID  = 0;  % double
    block.InputPort(3).Complexity  = 'Real';
    block.InputPort(3).DirectFeedthrough = true;

    % Input 4 (optional): gravity_earth - Custom gravity vector (3x1)
    if use_custom_gravity
        block.InputPort(4).Dimensions  = 3;
        block.InputPort(4).DatatypeID  = 0;  % double
        block.InputPort(4).Complexity  = 'Real';
        block.InputPort(4).DirectFeedthrough = true;
    end

    % Output: gravity vector (7x1)
    block.OutputPort(1).Dimensions  = 7;
    block.OutputPort(1).DatatypeID  = 0;  % double
    block.OutputPort(1).Complexity  = 'Real';
    block.OutputPort(1).SamplingMode = 'Sample';
    
    % Inherited sample time: works in both standalone and function-called contexts
    block.SampleTimes = [-1 0];

    block.SimStateCompliance = 'DefaultSimState';

    % Register methods
    block.RegBlockMethod('CheckParameters',         @CheckPrms);
    block.RegBlockMethod('ProcessParameters',       @ProcessPrms);
    block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
    block.RegBlockMethod('Start',                   @Start); 
    block.RegBlockMethod('WriteRTW',                @WriteRTW);
    block.RegBlockMethod('Outputs',                 @Outputs);

function CheckPrms(block)
    % Validate robot_ip parameter
    robot_ip = block.DialogPrm(1).Data;
    if ~ischar(robot_ip) && ~isstring(robot_ip)
        error('robot_ip must be a string');
    end

function DoPostPropSetup(block)
    % No DWork needed

function ProcessPrms(block)
    block.AutoUpdateRuntimePrms;

function Start(block)
    % Initialization - nothing to do here

function Outputs(block)
    % For simulation only - actual implementation is in TLC
    block.OutputPort(1).Data = zeros(7, 1);

function WriteRTW(block)
    % Write parameters to RTW file for TLC access
    robot_ip = block.DialogPrm(1).Data;
    use_custom_gravity = block.DialogPrm(2).Data;
    
    robot_ip = char(['''', robot_ip, '''']);
    
    block.WriteRTWParam('string', 'robot_ip', robot_ip);
    block.WriteRTWParam('matrix', 'use_custom_gravity', int8(use_custom_gravity));
