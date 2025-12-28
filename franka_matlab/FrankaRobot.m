classdef FrankaRobot < handle
    %FRANKAROBOT High-level interface to Franka Emika robots
    %
    %   The robot provides both synchronous and asynchronous motion execution:
    %
    %   Synchronous (blocking, default):
    %       robot.joint_point_to_point_motion(q_target, 0.5)  % Blocks until complete
    %       robot.joint_trajectory_motion(trajectory)          % Blocks until complete
    %
    %   Asynchronous (non-blocking):
    %       robot.joint_point_to_point_motion(q_target, 0.5, 'Async', true)
    %       robot.joint_trajectory_motion(trajectory, 'Async', true)
    %
    %   Async control methods:
    %       motion_status()  - Get current motion command status
    %       motion_wait()    - Wait for async motion to complete
    %       motion_isBusy()  - Check if motion is in progress
    %       stop()           - Stop current motion
    %
    %   Example (single robot):
    %       robot = FrankaRobot('RobotIP', '172.16.0.2');
    %
    %   Example (multiple robots):
    %       robot1 = FrankaRobot('RobotIP', '172.16.0.2', 'ServerPort', '5001');
    %       robot2 = FrankaRobot('RobotIP', '172.16.0.3', 'ServerPort', '5002');
    
    properties (Constant, Access = private)
        DefaultServerUsername = 'franka';
        DefaultServerIP = '172.16.1.2';
        DefaultSSHPort = '22';
        DefaultServerPort = '5001';
        % Error patterns in server logs that indicate initialization failure
        ErrorPatterns = {'error:', 'exception', 'Connection error', 'Network error'};
    end
    
    properties
        Settings
        Server
        Gripper
        VacuumGripper
        AutoReconnect = true  % Enable automatic reconnection on command failure
    end
    
    properties (SetAccess = private)
        RobotIP
    end
    
    properties (SetAccess = private, Hidden = true)
        frankaRobotHandle
    end
    
    methods
        function obj = FrankaRobot(varargin)
            % FrankaRobot Constructor
            %
            % Parameters:
            %   'RobotIP'    - IP address of the Franka robot
            %   'Settings'   - FrankaRobotSettings object
            %   'ServerPort' - RPC server port (use different ports for multiple robots)
            %   'ServerIP'   - Remote server IP (triggers SSH mode)
            %   'Username'   - SSH username (default: 'franka')
            %   'SSHPort'    - SSH port (default: '22')
            %
            % Notes:
            % - If a server is already running on the requested ServerPort, this constructor
            %   will attach to it instead of starting a new process (and will not block on
            %   remote log reading). To run multiple robots, use different ServerPort values.
            
            p = inputParser;
            addParameter(p, 'RobotIP', '', @(x) ischar(x) || isstring(x));
            addParameter(p, 'Settings', FrankaRobotSettings(), @(x) isa(x, 'FrankaRobotSettings'));
            addParameter(p, 'Username', '', @(x) ischar(x) || isstring(x));
            addParameter(p, 'ServerIP', '', @(x) ischar(x) || isstring(x));
            addParameter(p, 'SSHPort', obj.DefaultSSHPort, @(x) ischar(x) || isstring(x));
            addParameter(p, 'ServerPort', obj.DefaultServerPort, @(x) ischar(x) || isstring(x));
            parse(p, varargin{:});
            params = p.Results;
            
            obj.Settings = params.Settings;
            obj.RobotIP = char(params.RobotIP);
            if isempty(obj.RobotIP)
                obj.RobotIP = obj.Settings.robot_ip;
            end
            
            % Remote mode if ServerIP is provided
            serverIP = char(params.ServerIP);
            if ~isempty(serverIP)
                username = char(params.Username);
                if isempty(username), username = obj.DefaultServerUsername; end
                obj.Server = FrankaRobotServer(username, serverIP, ...
                    char(params.SSHPort), char(params.ServerPort));
            else
                obj.Server = FrankaRobotServer();
                if ~strcmp(params.ServerPort, obj.DefaultServerPort)
                    obj.Server.setServerPort(char(params.ServerPort));
                end
            end

            try
                wasAlreadyRunning = obj.Server.isRunning();
                obj.Server.start();
            catch ME
                error('FrankaRobot:InitError', 'Failed to start server: %s', ME.message);
            end

            obj.frankaRobotHandle = franka_robot('new', obj.Server.getServerIp(), obj.Server.getServerPort());
            
            try
                % If we attached to an already-running server, validate connectivity early.
                if wasAlreadyRunning
                    try
                        obj.ping();
                    catch MEping
                        error('FrankaRobot:InitError', ...
                            'Server is running on port %s but RPC ping failed. Check ServerIP/ServerPort. Details: %s', ...
                            obj.Server.getServerPort(), MEping.message);
                    end
                end
                obj.initialize();
                % Check server logs for initialization errors (server may not throw).
                % If we attached to an already-running server, the log may include stale
                % lines from previous runs (and in remote mode could be expensive to read).
                if ~wasAlreadyRunning
                    pause(0.3);  % Brief pause to allow server to log any errors
                    obj.checkInitializationErrors();
                end
            catch ME
                obj.Server.stop();
                franka_robot('delete', obj.frankaRobotHandle);
                obj.frankaRobotHandle = [];  % Clear so destructor doesn't double-delete
                error('FrankaRobot:InitError', 'Failed to initialize robot: %s', ME.message);
            end

            obj.Gripper = FrankaGripper(obj.frankaRobotHandle);
            obj.VacuumGripper = FrankaVacuumGripper(obj.frankaRobotHandle);
        end
        
        function delete(obj)
            if ~isempty(obj.Server)
                obj.Server.stop();
            end
            if ~isempty(obj.frankaRobotHandle)
                franka_robot('delete', obj.frankaRobotHandle);
            end
        end

        function automatic_error_recovery(obj, varargin)
            obj.checkHandle();
            args = varargin;
            obj.executeWithReconnect(@() ...
                franka_robot('automatic_error_recovery', obj.frankaRobotHandle, args{:}));
        end

        function robot_state = state(obj)
            obj.checkHandle();
            robot_state = obj.executeWithReconnect(@() ...
                franka_robot('robot_state', obj.frankaRobotHandle));
        end

        function joint_poses = joint_poses(obj)
            obj.checkHandle();
            joint_poses = obj.executeWithReconnect(@() ...
                franka_robot('joint_poses', obj.frankaRobotHandle));
        end
        
        function result = joint_point_to_point_motion(obj, joints_target_configuration, speed_factor, varargin)
            % Move robot to target joint configuration
            %
            % Syntax:
            %   result = robot.joint_point_to_point_motion(target, speed_factor)
            %   result = robot.joint_point_to_point_motion(target, speed_factor, 'Async', true)
            %   result = robot.joint_point_to_point_motion(target, speed_factor, 'Async', true, 'Timeout', 60)
            %
            % Inputs:
            %   joints_target_configuration - 7-element target joint positions
            %   speed_factor - Speed factor in (0, 1] (default: 0.5)
            %
            % Name-Value Arguments:
            %   'Async'   - If true, return immediately (default: false)
            %   'Timeout' - Max time for async command in seconds (default: 60)
            %
            % Returns:
            %   result - If sync: true if motion succeeded
            %            If async: true if command was started
            obj.checkHandle();
            if nargin < 3 || isempty(speed_factor), speed_factor = 0.5; end
            
            % Handle name-value pairs starting at position 3
            if nargin >= 4 && (ischar(speed_factor) || isstring(speed_factor))
                varargin = [{speed_factor}, varargin];
                speed_factor = 0.5;
            end
            
            p = inputParser;
            addParameter(p, 'Async', false, @islogical);
            addParameter(p, 'Timeout', 60.0, @isnumeric);
            parse(p, varargin{:});
            
            if p.Results.Async
                result = obj.executeWithReconnect(@() ...
                    franka_robot('joint_point_to_point_motion_async', obj.frankaRobotHandle, ...
                        joints_target_configuration, speed_factor, p.Results.Timeout));
            else
                result = obj.executeWithReconnect(@() ...
                    franka_robot('joint_point_to_point_motion', obj.frankaRobotHandle, ...
                        joints_target_configuration, speed_factor));
            end
        end

        function result = joint_trajectory_motion(obj, positions, varargin)
            % Execute joint trajectory motion
            %
            % Syntax:
            %   result = robot.joint_trajectory_motion(positions)
            %   result = robot.joint_trajectory_motion(positions, 'Async', true)
            %   result = robot.joint_trajectory_motion(positions, 'Async', true, 'Timeout', 120)
            %
            % Inputs:
            %   positions - 7xN array of joint positions (1ms per column)
            %
            % Name-Value Arguments:
            %   'Async'   - If true, return immediately (default: false)
            %   'Timeout' - Max time for async command in seconds (default: auto)
            %
            % Returns:
            %   result - If sync: true if motion succeeded
            %            If async: true if command was started
            obj.checkHandle();
            [m, n] = size(positions);
            if m ~= 7
                error('Positions must be a 7xN array');
            end
            
            p = inputParser;
            addParameter(p, 'Async', false, @islogical);
            addParameter(p, 'Timeout', 0.0, @isnumeric);  % 0 = auto-calculate
            parse(p, varargin{:});
            
            if p.Results.Async
                result = obj.executeWithReconnect(@() ...
                    franka_robot('joint_trajectory_motion_async', obj.frankaRobotHandle, ...
                        positions, p.Results.Timeout));
            else
                result = obj.executeWithReconnect(@() ...
                    franka_robot('joint_trajectory_motion', obj.frankaRobotHandle, positions));
            end
        end
        
        function s = motion_status(obj)
            % Get the current motion command status
            % Returns:
            %   s - Struct with fields:
            %       q              - Current joint positions (1x7)
            %       dq             - Current joint velocities (1x7)
            %       command_status - 'idle', 'busy', 'success', 'failed',
            %                        'timeout', or 'stopped'
            %       last_command   - Name of last/current command
            %       error_message  - Error message if failed
            %       progress       - Motion progress 0.0-1.0
            obj.checkHandle();
            s = obj.executeWithReconnect(@() ...
                franka_robot('motion_async_status', obj.frankaRobotHandle));
        end
        
        function s = motion_wait(obj, timeout)
            % Wait for the current async motion to complete
            % Inputs:
            %   timeout - Maximum wait time in seconds (default: 120)
            % Returns:
            %   s - Same as motion_status() after command completes
            obj.checkHandle();
            if nargin < 2, timeout = 120.0; end
            s = obj.executeWithReconnect(@() ...
                franka_robot('motion_wait', obj.frankaRobotHandle, timeout));
        end
        
        function busy = motion_isBusy(obj)
            % Check if a motion command is currently in progress
            % Returns:
            %   busy - True if a motion is running
            s = obj.motion_status();
            busy = strcmp(s.command_status, 'busy');
        end

        function result = setCollisionThresholds(obj, thresholds)
            obj.Settings.collision_thresholds = thresholds;
            ct = obj.Settings.collision_thresholds;
            result = obj.executeWithReconnect(@() ...
                franka_robot('set_collision_behavior', obj.frankaRobotHandle, ...
                    ct.lower_torque_thresholds_acceleration, ...
                    ct.upper_torque_thresholds_acceleration, ...
                    ct.lower_torque_thresholds_nominal, ...
                    ct.upper_torque_thresholds_nominal, ...
                    ct.lower_force_thresholds_acceleration, ...
                    ct.upper_force_thresholds_acceleration, ...
                    ct.lower_force_thresholds_nominal, ...
                    ct.upper_force_thresholds_nominal));
        end

        function thresholds = getCollisionThresholds(obj)
            thresholds = obj.Settings.collision_thresholds;
        end

        function result = setLoadInertia(obj, loadInertia)
            obj.Settings.load_inertia = loadInertia;
            li = obj.Settings.load_inertia;
            result = obj.executeWithReconnect(@() ...
                franka_robot('set_load', obj.frankaRobotHandle, ...
                    li.mass, li.center_of_mass, li.inertia_matrix));
        end

        function inertia = getLoadInertia(obj)
            inertia = obj.Settings.load_inertia;
        end

        function result = homing(obj)
            obj.checkHandle();
            result = obj.joint_point_to_point_motion(obj.Settings.home_configuration, 0.1);
        end

        function resetSettings(obj)
            obj.Settings = FrankaRobotSettings();
            obj.applySettings();
        end

        function result = setJointImpedance(obj, K_theta)
            obj.checkHandle();
            if nargin < 2
                K_theta = obj.Settings.joint_impedance_stiffness;
            else
                validateattributes(K_theta, {'numeric'}, {'numel', 7, 'positive'});
                obj.Settings.joint_impedance_stiffness = K_theta(:)';
            end
            stiffness = K_theta(:)';
            result = obj.executeWithReconnect(@() ...
                franka_robot('set_joint_impedance', obj.frankaRobotHandle, stiffness));
        end

        function K_theta = getJointImpedance(obj)
            K_theta = obj.Settings.joint_impedance_stiffness;
        end

        function result = setCartesianImpedance(obj, K_x)
            obj.checkHandle();
            if nargin < 2
                K_x = obj.Settings.cartesian_impedance_stiffness;
            else
                validateattributes(K_x, {'numeric'}, {'numel', 6, 'positive'});
                obj.Settings.cartesian_impedance_stiffness = K_x(:)';
            end
            stiffness = K_x(:)';
            result = obj.executeWithReconnect(@() ...
                franka_robot('set_cartesian_impedance', obj.frankaRobotHandle, stiffness));
        end

        function K_x = getCartesianImpedance(obj)
            K_x = obj.Settings.cartesian_impedance_stiffness;
        end

        function result = setGuidingMode(obj, guiding_mode, elbow)
            obj.checkHandle();
            validateattributes(guiding_mode, {'logical'}, {'numel', 6});
            validateattributes(elbow, {'logical'}, {'scalar'});
            gm = guiding_mode(:)';
            result = obj.executeWithReconnect(@() ...
                franka_robot('set_guiding_mode', obj.frankaRobotHandle, gm, elbow));
        end

        function result = setK(obj, EE_T_K)
            obj.checkHandle();
            if nargin < 2
                EE_T_K = obj.Settings.EE_T_K;
            else
                validateattributes(EE_T_K, {'numeric'}, {'size', [4, 4]});
                obj.Settings.EE_T_K = EE_T_K;
            end
            transform = EE_T_K(:)';
            result = obj.executeWithReconnect(@() ...
                franka_robot('set_k', obj.frankaRobotHandle, transform));
        end

        function EE_T_K = getK(obj)
            EE_T_K = obj.Settings.EE_T_K;
        end

        function result = setEE(obj, NE_T_EE)
            obj.checkHandle();
            if nargin < 2
                NE_T_EE = obj.Settings.NE_T_EE;
            else
                validateattributes(NE_T_EE, {'numeric'}, {'size', [4, 4]});
                obj.Settings.NE_T_EE = NE_T_EE;
            end
            transform = NE_T_EE(:)';
            result = obj.executeWithReconnect(@() ...
                franka_robot('set_ee', obj.frankaRobotHandle, transform));
        end

        function NE_T_EE = getEE(obj)
            NE_T_EE = obj.Settings.NE_T_EE;
        end

        function result = stop(obj)
            obj.checkHandle();
            result = obj.executeWithReconnect(@() ...
                franka_robot('stop_robot', obj.frankaRobotHandle));
        end
        
        function result = ping(obj)
            % Ping the server to verify connectivity
            obj.checkHandle();
            result = franka_robot('ping', obj.frankaRobotHandle);
        end
        
        function healthy = isHealthy(obj)
            % Check if server connection is healthy via RPC ping
            try
                result = obj.ping();
                healthy = ~isempty(result) && result.port == str2double(obj.Server.getServerPort());
            catch
                healthy = false;
            end
        end

        function reconnect(obj)
            % Reconnect to server after it was restarted
            %
            % Use this method after calling Server.stop() and Server.start()
            % to re-establish the RPC connection.
            
            % Ensure server is running
            if ~obj.Server.isRunning()
                obj.Server.start();
            end
            
            % Delete old handle if exists
            if ~isempty(obj.frankaRobotHandle)
                try
                    franka_robot('delete', obj.frankaRobotHandle);
                catch
                    % Ignore errors from stale handle
                end
            end
            
            % Create new connection
            obj.frankaRobotHandle = franka_robot('new', obj.Server.getServerIp(), obj.Server.getServerPort());
            
            % Reinitialize robot
            obj.initialize();
            
            % Recreate gripper interfaces with new handle
            obj.Gripper = FrankaGripper(obj.frankaRobotHandle);
            obj.VacuumGripper = FrankaVacuumGripper(obj.frankaRobotHandle);
        end

        function initialize(obj)
            franka_robot('initialize_robot', obj.frankaRobotHandle, obj.RobotIP);
        end
    end

    methods (Access = private)
        function checkHandle(obj)
            if isempty(obj.frankaRobotHandle)
                error('FrankaRobot:NotConnected', 'Server not connected');
            end
        end
        
        function applySettings(obj)
            obj.setCollisionThresholds(obj.Settings.collision_thresholds);
            obj.setLoadInertia(obj.Settings.load_inertia);
            obj.setJointImpedance();
            obj.setCartesianImpedance();
            obj.setEE();
            obj.setK();
        end
        
        function checkInitializationErrors(obj)
            % Check server logs for error messages that indicate failed initialization
            lines = obj.Server.getOutput();
            errorLines = {};
            for i = 1:numel(lines)
                line = lines{i};
                for j = 1:numel(obj.ErrorPatterns)
                    if contains(line, obj.ErrorPatterns{j}, 'IgnoreCase', true)
                        errorLines{end+1} = line; %#ok<AGROW>
                        break;
                    end
                end
            end
            
            if ~isempty(errorLines)
                errorMsg = strjoin(errorLines, '\n');
                error('FrankaRobot:InitError', ...
                    'Robot initialization failed. Server reported:\n%s', errorMsg);
            end
        end
        
        function varargout = executeWithReconnect(obj, cmdFunc)
            % Execute a command with automatic reconnection on failure
            %   cmdFunc - function handle that performs the actual command
            %
            % If AutoReconnect is enabled, attempts to reconnect once on failure
            % and retry the command.
            
            try
                [varargout{1:nargout}] = cmdFunc();
                return;
            catch ME
                if obj.AutoReconnect && obj.isConnectionError(ME)
                    try
                        obj.reconnect();
                        [varargout{1:nargout}] = cmdFunc();
                        return;
                    catch reconnectME
                        throwAsCaller(MException('FrankaRobot:ReconnectFailed', ...
                            'Reconnection failed: %s', reconnectME.message));
                    end
                end
                
                % Keep errors concise (avoid deep internal stack traces).
                throwAsCaller(MException('FrankaRobot:RemoteError', '%s', ME.message));
            end
        end
        
        function isConnErr = isConnectionError(obj, ME)
            % Check if an exception indicates a connection/communication error
            connErrorPatterns = {'connection', 'timeout', 'network', ...
                'communication', 'rpc', 'disconnected', 'socket', 'interrupted'};
            msgLower = lower(ME.message);
            isConnErr = false;
            for i = 1:numel(connErrorPatterns)
                if contains(msgLower, connErrorPatterns{i})
                    isConnErr = true;
                    return;
                end
            end
            % Also check if server is no longer running
            if ~isConnErr
                try
                    isConnErr = ~obj.Server.isRunning();
                catch
                    isConnErr = true;
                end
            end
        end
    end
end
