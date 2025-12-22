function franka_robot_model_mass_sfunction(block)
%FRANKA_ROBOT_MODEL_MASS Compute mass matrix M(q) using libfranka Model
%
%  This block computes the 7x7 mass matrix from explicit inputs using
%  the robot's kinematic/dynamic model. It integrates with FrankaRobotManager
%  to share the robot connection with other blocks.
%
%  Parameters:
%    robot_ip - IP address of the Franka robot (for model lookup)
%
%  Inputs:
%    q          - Joint positions [rad] (7x1)
%    I_total    - Total load inertia [kg*m^2] (3x3 column-major as 9x1)
%    m_total    - Total load mass [kg] (1x1)
%    F_x_Ctotal - Total load CoM in flange frame [m] (3x1)
%
%  Outputs:
%    mass - Mass matrix M(q) [kg*m^2] (7x7)
%
%  Usage Notes:
%    - For current robot state model data, use the model_data output from
%      franka_robot block instead (FrankaModelDataBus.mass)
%    - This block is for computing model data from arbitrary configurations
%    - Works both standalone AND under function-called subsystems
%    - Attempts to connect to robot on first execution if not already connected
%
%  Copyright (c) 2025 Franka Robotics GmbH
%  This file is subject to the terms and conditions defined in the file
%  'LICENSE', which is part of this package

setup(block);

function setup(block)

    % Register parameters: robot_ip only
    block.NumDialogPrms     = 1;
    block.DialogPrmsTunable = {'Nontunable'};
    
    % Register number of ports
    % Inputs: q(7), I_total(9), m_total(1), F_x_Ctotal(3)
    block.NumInputPorts  = 4;
    block.NumOutputPorts = 1;

    % Setup port properties to be inherited or dynamic
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    % Input 1: q - Joint positions (7x1)
    block.InputPort(1).Dimensions  = 7;
    block.InputPort(1).DatatypeID  = 0;  % double
    block.InputPort(1).Complexity  = 'Real';
    block.InputPort(1).DirectFeedthrough = true;

    % Input 2: I_total - Total load inertia (9x1, 3x3 column-major)
    block.InputPort(2).Dimensions  = 9;
    block.InputPort(2).DatatypeID  = 0;  % double
    block.InputPort(2).Complexity  = 'Real';
    block.InputPort(2).DirectFeedthrough = true;

    % Input 3: m_total - Total load mass (1x1)
    block.InputPort(3).Dimensions  = 1;
    block.InputPort(3).DatatypeID  = 0;  % double
    block.InputPort(3).Complexity  = 'Real';
    block.InputPort(3).DirectFeedthrough = true;

    % Input 4: F_x_Ctotal - Total load CoM (3x1)
    block.InputPort(4).Dimensions  = 3;
    block.InputPort(4).DatatypeID  = 0;  % double
    block.InputPort(4).Complexity  = 'Real';
    block.InputPort(4).DirectFeedthrough = true;

    % Output: mass matrix (7x7 column-major)
    block.OutputPort(1).Dimensions  = [7, 7];
    block.OutputPort(1).DatatypeID  = 0;  % double
    block.OutputPort(1).Complexity  = 'Real';
    block.OutputPort(1).SamplingMode = 'Sample';
    
    % Inherited sample time: works in both standalone and function-called contexts
    % - Standalone: inherits from driving source (e.g., constant blocks at model rate)
    % - Function-called subsystem: inherits from fcall trigger
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
    % No DWork needed for this simplified version

function ProcessPrms(block)
    block.AutoUpdateRuntimePrms;

function Start(block)
    % Initialization - nothing to do here
    % Connection is handled lazily by FrankaRobotManager in generated code

function Outputs(block)
    % For simulation only - actual implementation is in TLC
    % Output zeros during simulation
    block.OutputPort(1).Data = zeros(7, 7);

function WriteRTW(block)
    % Write parameters to RTW file for TLC access
    robot_ip = block.DialogPrm(1).Data;
    
    % Format robot_ip for TLC
    robot_ip = char(['''', robot_ip, '''']);
    
    block.WriteRTWParam('string', 'robot_ip', robot_ip);
