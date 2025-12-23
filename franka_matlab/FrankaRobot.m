classdef FrankaRobot < handle
    %FRANKAROBOT High-level interface to Franka Emika robots
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
                obj.Server.start();
            catch ME
                error('FrankaRobot:InitError', 'Failed to start server: %s', ME.message);
            end

            obj.frankaRobotHandle = franka_robot('new', obj.Server.getServerIp(), obj.Server.getServerPort());
            
            try
                obj.initialize();
                % Check server logs for initialization errors (server may not throw)
                pause(0.3);  % Brief pause to allow server to log any errors
                obj.checkInitializationErrors();
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

        function robot_state = robot_state(obj)
            obj.checkHandle();
            robot_state = obj.executeWithReconnect(@() ...
                franka_robot('robot_state', obj.frankaRobotHandle));
        end

        function joint_poses = joint_poses(obj)
            obj.checkHandle();
            joint_poses = obj.executeWithReconnect(@() ...
                franka_robot('joint_poses', obj.frankaRobotHandle));
        end
        
        function result = joint_point_to_point_motion(obj, joints_target_configuration, speed_factor)
            obj.checkHandle();
            if nargin < 3, speed_factor = 0.5; end
            result = obj.executeWithReconnect(@() ...
                franka_robot('joint_point_to_point_motion', obj.frankaRobotHandle, ...
                    joints_target_configuration, speed_factor));
        end

        function result = joint_trajectory_motion(obj, positions)
            obj.checkHandle();
            [m, ~] = size(positions);
            if m ~= 7
                error('Positions must be a 7xN array');
            end
            result = obj.executeWithReconnect(@() ...
                franka_robot('joint_trajectory_motion', obj.frankaRobotHandle, positions));
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

        function result = robot_homing(obj)
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
            catch ME
                if obj.AutoReconnect && obj.isConnectionError(ME)
                    warning('FrankaRobot:ConnectionLost', ...
                        'Connection lost, attempting to reconnect...');
                    try
                        obj.reconnect();
                        [varargout{1:nargout}] = cmdFunc();
                        fprintf('Reconnection successful.\n');
                    catch reconnectME
                        error('FrankaRobot:ReconnectFailed', ...
                            'Reconnection failed: %s\nOriginal error: %s', ...
                            reconnectME.message, ME.message);
                    end
                else
                    rethrow(ME);
                end
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
