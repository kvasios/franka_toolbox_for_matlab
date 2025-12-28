classdef FrankaVacuumGripper < handle
    %FRANKAVACUUMGRIPPER Interface to Franka vacuum gripper with async support
    %
    %   The vacuum gripper provides both synchronous and asynchronous execution:
    %
    %   Synchronous (blocking, default):
    %       gripper.vacuum(0, 5000, 0)      % Blocks until complete
    %       gripper.dropOff(5000)           % Blocks until complete
    %
    %   Asynchronous (non-blocking):
    %       gripper.vacuum(0, 5000, 0, 'Async', true)   % Returns immediately
    %       gripper.dropOff(5000, 'Async', true)
    %
    %   Async control methods:
    %       status()    - Get current command status (includes gripper state)
    %       wait()      - Wait for async command completion
    %       isBusy()    - Check if command is in progress
    %       stop()      - Stop/interrupt current operation
    %
    %   Example (async vacuum with polling):
    %       gripper.vacuum(0, 5000, 0, 'Async', true);
    %       while gripper.isBusy()
    %           s = gripper.status();
    %           fprintf('Vacuum: %.1f, Part present: %d\n', s.vacuum, s.part_present);
    %           pause(0.1);
    %       end
    %
    %   Example (async drop off with wait):
    %       gripper.dropOff(5000, 'Async', true);
    %       result = gripper.wait(10);  % Wait up to 10 seconds
    %       if strcmp(result.command_status, 'success')
    %           disp('Part released!');
    %       end
    %
    %   Example (stop during async command):
    %       gripper.vacuum(0, 5000, 0, 'Async', true, 'Timeout', 30);
    %       pause(1);
    %       gripper.stop();  % Interrupt the vacuum
    
    properties (Access = private)
        frankaRobotHandle
        isInitialized = false
    end
    
    properties (Constant, Access = private)
        % Default values
        DefaultControlPoint = 0;
        DefaultTimeout = 5000;       % 5 seconds in milliseconds
        DefaultProfile = 0;          % Default production setup profile
        DefaultCommandTimeout = 15.0;  % seconds for async timeout
        DefaultWaitTimeout = 30.0;   % seconds
    end
    
    methods
        %% Constructor
        function obj = FrankaVacuumGripper(frankaRobotHandle)
            % FrankaVacuumGripper constructor
            % Input:
            %   frankaRobotHandle - Handle to the Franka robot instance
            obj.frankaRobotHandle = frankaRobotHandle;
        end
        
        %% Private Methods
        function initializeVacuumGripper(obj)
            % Initialize the vacuum gripper on the robot
            if ~isempty(obj.frankaRobotHandle) && ~obj.isInitialized
                franka_robot('initialize_vacuum_gripper', obj.frankaRobotHandle);
                obj.isInitialized = true;
            end
        end
        
        %% Vacuum Gripper State
        function s = state(obj)
            % Get the current state of the vacuum gripper
            % Returns:
            %   s - Structure containing:
            %       in_control_range - Whether the vacuum gripper is in control range
            %       part_detached - Whether a part is detached
            %       part_present - Whether a part is present
            %       device_status - Current device status
            %       actual_power - Current actual power
            %       vacuum - Current system vacuum
            %       time - Time stamp
            obj.initializeVacuumGripper();
            s = franka_robot('vacuum_gripper_state', obj.frankaRobotHandle);
        end
        
        %% Vacuum Gripper Vacuum
        function result = vacuum(obj, control_point, timeout, profile, varargin)
            % Apply vacuum to the gripper
            %
            % Syntax:
            %   result = gripper.vacuum()
            %   result = gripper.vacuum(control_point, timeout, profile)
            %   result = gripper.vacuum(control_point, timeout, profile, 'Async', true)
            %   result = gripper.vacuum(control_point, timeout, profile, 'Async', true, 'Timeout', 15)
            %
            % Inputs:
            %   control_point - Vacuum control point (default: 0)
            %   timeout       - Timeout in milliseconds (default: 5000)
            %   profile       - Production setup profile (default: 0)
            %
            % Name-Value Arguments:
            %   'Async'   - If true, return immediately (default: false)
            %   'Timeout' - Max time for async command in seconds (default: 15)
            %
            % Returns:
            %   result - If sync: true if vacuum was successful
            %            If async: true if command was started
            obj.initializeVacuumGripper();
            
            % Handle flexible argument parsing (positional + name-value)
            % Allow name-value usage without specifying all positional args, e.g.:
            %   gripper.vacuum('Async', true)
            %   gripper.vacuum(control_point, 'Async', true)
            if nargin >= 2 && (ischar(control_point) || isstring(control_point))
                varargin = [{control_point}, varargin];
                if nargin >= 3
                    varargin = [{timeout}, varargin];
                end
                if nargin >= 4
                    varargin = [{profile}, varargin];
                end
                control_point = [];
                timeout = [];
                profile = [];
            elseif nargin >= 3 && (ischar(timeout) || isstring(timeout))
                varargin = [{timeout}, varargin];
                if nargin >= 4
                    varargin = [{profile}, varargin];
                end
                timeout = [];
                profile = [];
            elseif nargin >= 4 && (ischar(profile) || isstring(profile))
                varargin = [{profile}, varargin];
                profile = [];
            end
            
            % Set default values if not provided
            if nargin < 4 || isempty(profile), profile = obj.DefaultProfile; end
            if nargin < 3 || isempty(timeout), timeout = obj.DefaultTimeout; end
            if nargin < 2 || isempty(control_point), control_point = obj.DefaultControlPoint; end
            
            p = inputParser;
            addParameter(p, 'Async', false, @islogical);
            addParameter(p, 'Timeout', obj.DefaultCommandTimeout, @isnumeric);
            parse(p, varargin{:});
            
            if p.Results.Async
                result = franka_robot('vacuum_gripper_vacuum_async', obj.frankaRobotHandle, ...
                    control_point, timeout, profile, p.Results.Timeout);
            else
                result = franka_robot('vacuum_gripper_vacuum', obj.frankaRobotHandle, ...
                    control_point, timeout, profile);
            end
        end
        
        %% Vacuum Gripper Drop Off
        function result = dropOff(obj, timeout, varargin)
            % Drop off the currently held object
            %
            % Syntax:
            %   result = gripper.dropOff()
            %   result = gripper.dropOff(timeout)
            %   result = gripper.dropOff(timeout, 'Async', true)
            %   result = gripper.dropOff(timeout, 'Async', true, 'Timeout', 15)
            %
            % Inputs:
            %   timeout - Timeout in milliseconds (default: 5000)
            %
            % Name-Value Arguments:
            %   'Async'   - If true, return immediately (default: false)
            %   'Timeout' - Max time for async command in seconds (default: 15)
            %
            % Returns:
            %   result - If sync: true if drop off was successful
            %            If async: true if command was started
            obj.initializeVacuumGripper();
            
            % Allow name-value usage without specifying timeout, e.g.:
            %   gripper.dropOff('Async', true)
            if nargin >= 2 && (ischar(timeout) || isstring(timeout))
                varargin = [{timeout}, varargin];
                timeout = [];
            end
            
            if nargin < 2 || isempty(timeout)
                timeout = obj.DefaultTimeout;
            end
            
            p = inputParser;
            addParameter(p, 'Async', false, @islogical);
            addParameter(p, 'Timeout', obj.DefaultCommandTimeout, @isnumeric);
            parse(p, varargin{:});
            
            if p.Results.Async
                result = franka_robot('vacuum_gripper_drop_off_async', obj.frankaRobotHandle, ...
                    timeout, p.Results.Timeout);
            else
                result = franka_robot('vacuum_gripper_drop_off', obj.frankaRobotHandle, timeout);
            end
        end
        
        %% Vacuum Gripper Stop
        function result = stop(obj)
            % Stop the vacuum gripper
            % Can interrupt async commands - the status will show 'stopped'
            % Returns:
            %   result - True if stop was successful
            obj.initializeVacuumGripper();
            result = franka_robot('vacuum_gripper_stop', obj.frankaRobotHandle);
        end
        
        %% Async Status
        function s = status(obj)
            % Get the current async command status and vacuum gripper state
            % Returns:
            %   s - Struct with fields:
            %       in_control_range, part_detached, part_present, 
            %       device_status, actual_power, vacuum, time
            %       command_status - 'idle', 'busy', 'success', 'failed',
            %                        'timeout', or 'stopped'
            %       last_command   - Name of last/current command
            %       error_message  - Error message if failed
            obj.initializeVacuumGripper();
            s = franka_robot('vacuum_gripper_async_status', obj.frankaRobotHandle);
        end
        
        %% Wait
        function s = wait(obj, timeout)
            % Wait for the current async command to complete
            % Inputs:
            %   timeout - Maximum wait time in seconds (default: 30)
            % Returns:
            %   s - Same as status() after command completes
            obj.initializeVacuumGripper();
            
            if nargin < 2, timeout = obj.DefaultWaitTimeout; end
            s = franka_robot('vacuum_gripper_wait', obj.frankaRobotHandle, timeout);
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
