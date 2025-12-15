function franka_torque_control(block)
%FRANKA_TORQUE_CONTROL Level-2 MATLAB S-Function for Franka robot torque control
%  
%  This block wraps libfranka's robot.control() with torque callback.
%  It outputs a function-call signal to trigger an external controller subsystem.
%
%  Inputs:
%    1. Enable    (1x1) - Rising edge starts control, falling edge stops
%    2. tau_J_d   (7x1) - Commanded joint torques (from controller subsystem)
%
%  Outputs:
%    1. fcall     (function-call) - Triggers controller subsystem at 1kHz
%    2. q         (7x1) - Measured joint positions
%    3. dq        (7x1) - Measured joint velocities
%    4. status    (1x1) - Control status: 0=Idle, 1=Running, 2=Error
%
%  Parameters:
%    1. robot_ip  (string) - Robot IP address
%
%  Copyright (c) 2025 Franka Robotics GmbH

setup(block);

%% ========================================================================
function setup(block)

    % Register number of dialog parameters
    block.NumDialogPrms = 1;  % robot_ip
    block.DialogPrmsTunable = {'Nontunable'};
    
    % Register number of input ports
    block.NumInputPorts = 2;
    
    % Register number of output ports  
    % Output 1 MUST be the function-call (Simulink requirement)
    block.NumOutputPorts = 4;
    
    % Setup input port properties
    % Port 1: Enable
    block.InputPort(1).Dimensions = 1;
    block.InputPort(1).DatatypeID = 0;  % double
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(1).SamplingMode = 'Sample';
    
    % Port 2: tau_J_d (joint torques)
    block.InputPort(2).Dimensions = 7;
    block.InputPort(2).DatatypeID = 0;  % double
    block.InputPort(2).Complexity = 'Real';
    block.InputPort(2).DirectFeedthrough = true;
    block.InputPort(2).SamplingMode = 'Sample';
    
    % Setup output port properties
    % Port 1: Function-call output (MUST be first!)
    block.OutputPort(1).Dimensions = 1;
    block.OutputPort(1).DatatypeID = 0;  % Will be overridden to fcn_call in TLC
    block.OutputPort(1).Complexity = 'Real';
    block.OutputPort(1).SamplingMode = 'Sample';
    
    % Port 2: q (joint positions)
    block.OutputPort(2).Dimensions = 7;
    block.OutputPort(2).DatatypeID = 0;  % double
    block.OutputPort(2).Complexity = 'Real';
    block.OutputPort(2).SamplingMode = 'Sample';
    
    % Port 3: dq (joint velocities)
    block.OutputPort(3).Dimensions = 7;
    block.OutputPort(3).DatatypeID = 0;  % double
    block.OutputPort(3).Complexity = 'Real';
    block.OutputPort(3).SamplingMode = 'Sample';
    
    % Port 4: status
    block.OutputPort(4).Dimensions = 1;
    block.OutputPort(4).DatatypeID = 0;  % double
    block.OutputPort(4).Complexity = 'Real';
    block.OutputPort(4).SamplingMode = 'Sample';
    
    % Register sample times
    % [0.001 0] = 1kHz discrete sample time
    block.SampleTimes = [0.001 0];
    
    % Specify block simStateCompliance
    block.SimStateCompliance = 'DefaultSimState';
    
    % Register methods
    block.RegBlockMethod('CheckParameters', @CheckPrms);
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Terminate', @Terminate);
    block.RegBlockMethod('WriteRTW', @WriteRTW);

%% ========================================================================
function CheckPrms(block)
    % Validate robot_ip parameter
    robot_ip = block.DialogPrm(1).Data;
    if ~ischar(robot_ip) && ~isstring(robot_ip)
        error('Robot IP must be a string');
    end

%% ========================================================================
function DoPostPropSetup(block)
    % Setup DWork vectors for state management
    block.NumDworks = 2;
    
    % DWork 1: Previous enable value (for edge detection)
    block.Dwork(1).Name = 'PrevEnable';
    block.Dwork(1).Dimensions = 1;
    block.Dwork(1).DatatypeID = 0;  % double
    block.Dwork(1).Complexity = 'Real';
    block.Dwork(1).UsedAsDiscState = true;
    
    % DWork 2: Control state (0=idle, 1=running, 2=error)
    block.Dwork(2).Name = 'ControlState';
    block.Dwork(2).Dimensions = 1;
    block.Dwork(2).DatatypeID = 0;  % double
    block.Dwork(2).Complexity = 'Real';
    block.Dwork(2).UsedAsDiscState = true;

%% ========================================================================
function Start(block)
    % Initialize DWork
    block.Dwork(1).Data = 0;  % PrevEnable = 0
    block.Dwork(2).Data = 0;  % ControlState = Idle

%% ========================================================================
function Outputs(block)
    % This is a placeholder for simulation
    % Actual implementation is in TLC for code generation
    
    % For simulation, just pass through zeros
    block.OutputPort(2).Data = zeros(7, 1);  % q
    block.OutputPort(3).Data = zeros(7, 1);  % dq
    block.OutputPort(4).Data = 0;            % status

%% ========================================================================
function Terminate(block)
    % Cleanup (placeholder for simulation)

%% ========================================================================
function WriteRTW(block)
    % Write parameters to RTW file for code generation
    
    robot_ip = block.DialogPrm(1).Data;
    
    % Create robot_id from IP (remove dots)
    robot_id = strrep(robot_ip, '.', '');
    
    % Quote strings for TLC
    robot_ip_quoted = sprintf('''%s''', robot_ip);
    robot_id_quoted = sprintf('''%s''', robot_id);
    
    % Write parameters
    block.WriteRTWParam('string', 'robot_ip', robot_ip_quoted);
    block.WriteRTWParam('string', 'robot_id', robot_id_quoted);

