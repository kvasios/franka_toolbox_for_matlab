function franka_robot_model_mass_sfunction(block)
%FRANKA_ROBOT_MODEL_MASS_SFUNCTION Compute mass matrix M(q) using libfranka Model
%
%  This block computes the 7x7 mass matrix from explicit inputs using
%  the robot's kinematic/dynamic model. It integrates with FrankaRobotManager
%  to share the robot connection with other blocks.
%
%  Inputs:
%    robot_ip   - IP address of the Franka robot (16 ASCII chars)
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

    % No dialog parameters - robot_ip is now an input port
    block.NumDialogPrms     = 0;
    
    % Register number of ports
    % Inputs: robot_ip(16), q(7), I_total(9), m_total(1), F_x_Ctotal(3)
    block.NumInputPorts  = 5;
    block.NumOutputPorts = 1;

    % Setup port properties to be inherited or dynamic
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    % Input 1: robot_ip - Robot IP address (16 ASCII chars)
    block.InputPort(1).Dimensions  = 16;
    block.InputPort(1).DatatypeID  = 3;  % uint8
    block.InputPort(1).Complexity  = 'Real';
    block.InputPort(1).DirectFeedthrough = true;

    % Input 2: q - Joint positions (7x1)
    block.InputPort(2).Dimensions  = 7;
    block.InputPort(2).DatatypeID  = 0;  % double
    block.InputPort(2).Complexity  = 'Real';
    block.InputPort(2).DirectFeedthrough = true;

    % Input 3: I_total - Total load inertia (9x1, 3x3 column-major)
    block.InputPort(3).Dimensions  = 9;
    block.InputPort(3).DatatypeID  = 0;  % double
    block.InputPort(3).Complexity  = 'Real';
    block.InputPort(3).DirectFeedthrough = true;

    % Input 4: m_total - Total load mass (1x1)
    block.InputPort(4).Dimensions  = 1;
    block.InputPort(4).DatatypeID  = 0;  % double
    block.InputPort(4).Complexity  = 'Real';
    block.InputPort(4).DirectFeedthrough = true;

    % Input 5: F_x_Ctotal - Total load CoM (3x1)
    block.InputPort(5).Dimensions  = 3;
    block.InputPort(5).DatatypeID  = 0;  % double
    block.InputPort(5).Complexity  = 'Real';
    block.InputPort(5).DirectFeedthrough = true;

    % Output: mass matrix (7x7 column-major)
    block.OutputPort(1).Dimensions  = [7, 7];
    block.OutputPort(1).DatatypeID  = 0;  % double
    block.OutputPort(1).Complexity  = 'Real';
    block.OutputPort(1).SamplingMode = 'Sample';
    
    % Inherited sample time: works in both standalone and function-called contexts
    block.SampleTimes = [-1 0];

    block.SimStateCompliance = 'DefaultSimState';

    % Register methods
    block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
    block.RegBlockMethod('Start',                   @Start); 
    block.RegBlockMethod('Outputs',                 @Outputs);

function DoPostPropSetup(block)
    % No DWork needed

function Start(block)
    % Initialization - nothing to do here

function Outputs(block)
    % For simulation only - actual implementation is in TLC
    block.OutputPort(1).Data = zeros(7, 7);
