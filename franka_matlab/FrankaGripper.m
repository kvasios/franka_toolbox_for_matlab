classdef FrankaGripper < handle
    %FRANKAGRIPPER Interface to Franka gripper with async support
    %
    %   The gripper provides both synchronous and asynchronous execution:
    %
    %   Synchronous (blocking, default):
    %       gripper.move(0.08, 0.1)           % Blocks until complete
    %       gripper.grasp(0.02, 0.1, 40)      % Blocks until complete
    %
    %   Asynchronous (non-blocking):
    %       gripper.move(0.08, 0.1, 'Async', true)   % Returns immediately
    %       gripper.grasp(0.02, 0.1, 40, 'Async', true)
    %
    %   Async control methods:
    %       status()    - Get current command status (includes gripper state)
    %       wait()      - Wait for async command completion
    %       isBusy()    - Check if command is in progress
    %       stop()      - Stop/interrupt current motion
    %
    %   Example (async move with polling):
    %       gripper.move(0.08, 0.1, 'Async', true);
    %       while gripper.isBusy()
    %           s = gripper.status();
    %           fprintf('Width: %.3f m\n', s.width);
    %           pause(0.1);
    %       end
    %
    %   Example (async grasp with wait):
    %       gripper.grasp(0.02, 0.1, 40, 'Async', true);
    %       result = gripper.wait(10);  % Wait up to 10 seconds
    %       if strcmp(result.command_status, 'success')
    %           disp('Object grasped!');
    %       end
    %
    %   Example (stop during async command):
    %       gripper.move(0.08, 0.02, 'Async', true, 'Timeout', 30);
    %       pause(1);
    %       gripper.stop();  % Interrupt the move
    
    properties (Access = private)
        frankaRobotHandle
        isInitialized = false
    end
    
    properties (Constant, Access = private)
        DefaultSpeed = 0.1           % m/s
        DefaultForce = 50            % N
        DefaultEpsilon = 0.005         % m (for grasp tolerance)
        DefaultTimeout = 15.0        % seconds
        DefaultWaitTimeout = 30.0    % seconds
    end
    
    methods
        %% Constructor
        function obj = FrankaGripper(frankaRobotHandle)
            % FrankaGripper constructor
            % Input:
            %   frankaRobotHandle - Handle to the Franka robot instance
            obj.frankaRobotHandle = frankaRobotHandle;
        end
        
        %% Private Methods
        function initializeGripper(obj)
            if ~isempty(obj.frankaRobotHandle) && ~obj.isInitialized
                franka_robot('initialize_gripper', obj.frankaRobotHandle);
                obj.isInitialized = true;
            end
        end
        
        %% Gripper State
        function s = state(obj)
            % Get the current state of the gripper
            % Returns:
            %   s - Struct with fields: width, max_width, is_grasped,
            %       temperature, time_stamp
            obj.initializeGripper();
            s = franka_robot('gripper_state', obj.frankaRobotHandle);
        end
        
        %% Homing
        function result = homing(obj)
            % Perform gripper homing (always blocking)
            % Returns:
            %   result - True if homing was successful
            obj.initializeGripper();
            result = franka_robot('gripper_homing', obj.frankaRobotHandle);
        end
        
        %% Move
        function result = move(obj, width, speed, varargin)
            % Move the gripper to a specific width
            %
            % Syntax:
            %   result = gripper.move(width)
            %   result = gripper.move(width, speed)
            %   result = gripper.move(width, speed, 'Async', true)
            %   result = gripper.move(width, speed, 'Async', true, 'Timeout', 15)
            %
            % Inputs:
            %   width   - Target width in meters
            %   speed   - Speed of motion (default: 0.1 m/s)
            %
            % Name-Value Arguments:
            %   'Async'   - If true, return immediately (default: false)
            %   'Timeout' - Max time for async command in seconds (default: 15)
            %
            % Returns:
            %   result - If sync: true if motion succeeded
            %            If async: true if command was started
            obj.initializeGripper();
            
            if nargin < 3 || isempty(speed), speed = obj.DefaultSpeed; end
            
            p = inputParser;
            addParameter(p, 'Async', false, @islogical);
            addParameter(p, 'Timeout', obj.DefaultTimeout, @isnumeric);
            parse(p, varargin{:});
            
            if p.Results.Async
                result = franka_robot('gripper_move_async', obj.frankaRobotHandle, ...
                    width, speed, p.Results.Timeout);
            else
                result = franka_robot('gripper_move', obj.frankaRobotHandle, width, speed);
            end
        end
        
        %% Grasp
        function result = grasp(obj, width, speed, force, epsilon_inner, epsilon_outer, varargin)
            % Grasp an object with the gripper
            %
            % Syntax:
            %   result = gripper.grasp(width)
            %   result = gripper.grasp(width, speed, force)
            %   result = gripper.grasp(width, speed, force, 'Async', true)
            %   result = gripper.grasp(width, speed, force, eps_in, eps_out, 'Async', true)
            %
            % Inputs:
            %   width         - Target width in meters
            %   speed         - Speed of motion (default: 0.1 m/s)
            %   force         - Grasping force in N (default: 50 N)
            %   epsilon_inner - Inner tolerance (default: 0.005 m)
            %   epsilon_outer - Outer tolerance (default: 0.005 m)
            %
            % Name-Value Arguments:
            %   'Async'   - If true, return immediately (default: false)
            %   'Timeout' - Max time for async command in seconds (default: 15)
            %
            % Returns:
            %   result - If sync: true if grasp succeeded
            %            If async: true if command was started
            obj.initializeGripper();
            
            % Allow name-value usage without specifying epsilons, e.g.:
            %   gripper.grasp(w, v, f, 'Async', true)
            % Without this, MATLAB binds 'Async'/true to epsilon_inner/epsilon_outer and
            % the MEX sends garbage epsilon values.
            if nargin >= 5 && (ischar(epsilon_inner) || isstring(epsilon_inner))
                varargin = [{epsilon_inner, epsilon_outer}, varargin];
                epsilon_inner = [];
                epsilon_outer = [];
            elseif nargin >= 6 && (ischar(epsilon_outer) || isstring(epsilon_outer))
                varargin = [{epsilon_outer}, varargin];
                epsilon_outer = [];
            end

            % Handle flexible argument parsing (positional + name-value)
            if nargin < 3 || isempty(speed), speed = obj.DefaultSpeed; end
            if nargin < 4 || isempty(force), force = obj.DefaultForce; end
            if nargin < 5 || isempty(epsilon_inner), epsilon_inner = obj.DefaultEpsilon; end
            if nargin < 6 || isempty(epsilon_outer), epsilon_outer = obj.DefaultEpsilon; end
            
            p = inputParser;
            addParameter(p, 'Async', false, @islogical);
            addParameter(p, 'Timeout', obj.DefaultTimeout, @isnumeric);
            parse(p, varargin{:});
            
            if p.Results.Async
                result = franka_robot('gripper_grasp_async', obj.frankaRobotHandle, ...
                    width, speed, force, epsilon_inner, epsilon_outer, p.Results.Timeout);
            else
                result = franka_robot('gripper_grasp', obj.frankaRobotHandle, ...
                    width, speed, force, epsilon_inner, epsilon_outer);
            end
        end
        
        %% Stop
        function result = stop(obj)
            % Stop the gripper motion
            % Can interrupt async commands - the status will show 'stopped'
            % Returns:
            %   result - True if stop was successful
            obj.initializeGripper();
            result = franka_robot('gripper_stop', obj.frankaRobotHandle);
        end
        
        %% Async Status
        function s = status(obj)
            % Get the current async command status and gripper state
            % Returns:
            %   s - Struct with fields:
            %       width, max_width, is_grasped, temperature, time_stamp
            %       command_status - 'idle', 'busy', 'success', 'failed',
            %                        'timeout', or 'stopped'
            %         NOTE: for grasp(), 'success' means the command completed without
            %         error; whether an object is held is indicated by is_grasped.
            %       last_command   - Name of last/current command
            %       error_message  - Error message if failed
            obj.initializeGripper();
            s = franka_robot('gripper_async_status', obj.frankaRobotHandle);
        end
        
        %% Wait
        function s = wait(obj, timeout)
            % Wait for the current async command to complete
            % Inputs:
            %   timeout - Maximum wait time in seconds (default: 30)
            % Returns:
            %   s - Same as status() after command completes
            obj.initializeGripper();
            
            if nargin < 2, timeout = obj.DefaultWaitTimeout; end
            s = franka_robot('gripper_wait', obj.frankaRobotHandle, timeout);
        end
        
        %% isBusy
        function busy = isBusy(obj)
            % Check if an async command is currently in progress
            % Returns:
            %   busy - True if a command is running
            s = obj.status();
            busy = strcmp(s.command_status, 'busy');
        end
    end
end
