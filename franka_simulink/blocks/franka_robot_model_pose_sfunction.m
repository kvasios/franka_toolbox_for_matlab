function franka_robot_model_pose_sfunction(block)
%FRANKA_ROBOT_MODEL_POSE Compute forward kinematics pose using libfranka Model
%
%  This block computes the 4x4 pose transformation for a specified frame
%  from explicit inputs using the robot's kinematic model. It integrates
%  with FrankaRobotManager to share the robot connection with other blocks.
%
%  Parameters:
%    robot_ip - IP address of the Franka robot (for model lookup)
%    frame    - Target frame: 0-6 = Joint1-7, 7 = Flange, 8 = EndEffector, 9 = Stiffness
%
%  Inputs:
%    q       - Joint positions [rad] (7x1)
%    F_T_EE  - End effector in flange frame (4x4 column-major as 16x1)
%    EE_T_K  - Stiffness frame in EE frame (4x4 column-major as 16x1)
%
%  Outputs:
%    pose - Transformation matrix in base frame (4x4)
%
%  Usage Notes:
%    - For current robot state poses (O_T_EE, etc.), use the state output from
%      franka_robot block (FrankaRobotStateBus.O_T_EE)
%    - This block is for computing poses from arbitrary configurations
%    - Works both standalone AND under function-called subsystems
%
%  Copyright (c) 2025 Franka Robotics GmbH
%  This file is subject to the terms and conditions defined in the file
%  'LICENSE', which is part of this package

setup(block);

function setup(block)

    % Register parameters: robot_ip, frame
    block.NumDialogPrms     = 2;
    block.DialogPrmsTunable = {'Nontunable', 'Nontunable'};
    
    % Register number of ports
    % Inputs: q(7), F_T_EE(16), EE_T_K(16)
    block.NumInputPorts  = 3;
    block.NumOutputPorts = 1;

    % Setup port properties to be inherited or dynamic
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    % Input 1: q - Joint positions (7x1)
    block.InputPort(1).Dimensions  = 7;
    block.InputPort(1).DatatypeID  = 0;  % double
    block.InputPort(1).Complexity  = 'Real';
    block.InputPort(1).DirectFeedthrough = true;

    % Input 2: F_T_EE - End effector in flange frame (16x1)
    block.InputPort(2).Dimensions  = 16;
    block.InputPort(2).DatatypeID  = 0;  % double
    block.InputPort(2).Complexity  = 'Real';
    block.InputPort(2).DirectFeedthrough = true;

    % Input 3: EE_T_K - Stiffness frame in EE frame (16x1)
    block.InputPort(3).Dimensions  = 16;
    block.InputPort(3).DatatypeID  = 0;  % double
    block.InputPort(3).Complexity  = 'Real';
    block.InputPort(3).DirectFeedthrough = true;

    % Output: pose transformation (4x4 column-major)
    block.OutputPort(1).Dimensions  = [4, 4];
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
    
    % Validate frame (0-9)
    frame = block.DialogPrm(2).Data;
    if frame < 0 || frame > 9
        error('frame must be 0-9');
    end

function DoPostPropSetup(block)
    % No DWork needed

function ProcessPrms(block)
    block.AutoUpdateRuntimePrms;

function Start(block)
    % Initialization - nothing to do here

function Outputs(block)
    % For simulation only - actual implementation is in TLC
    % Return identity matrix
    block.OutputPort(1).Data = eye(4);

function WriteRTW(block)
    % Write parameters to RTW file for TLC access
    robot_ip = block.DialogPrm(1).Data;
    frame = block.DialogPrm(2).Data;
    
    robot_ip = char(['''', robot_ip, '''']);
    
    block.WriteRTWParam('string', 'robot_ip', robot_ip);
    block.WriteRTWParam('matrix', 'frame', int8(frame));
