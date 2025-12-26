function [move_trigger, grasp_trigger, stop_trigger, ...
          move_width, move_speed, grasp_width, grasp_speed, grasp_force, ...
          read_state_trigger, state_out] = ...
    binaryGripperController(gripper_action, current_width, is_grasped, command_status, enable)
%BINARYGRIPPERCONTROLLER Binary open/close gripper control for VLAs
%
%   Simple binary action space controller for Vision-Language-Action models:
%     action < 0.5  → OPEN  (move to max width)
%     action >= 0.5 → CLOSE (grasp with force)
%
%   Uses stop() to immediately switch between open/close states.
%   Grasp uses force control to hold objects securely.
%
%   Inputs:
%       gripper_action - Binary action: <0.5 = open, >=0.5 = close/grasp
%       current_width  - Current gripper width from state [m]
%       is_grasped     - From gripper state (1 if object detected)
%       command_status - From gripper state: 0=idle, 1=busy, etc.
%       enable         - Enable signal
%
%   Outputs:
%       move_trigger   - Triggers move (open) command
%       grasp_trigger  - Triggers grasp (close with force) command
%       stop_trigger   - Triggers stop (for switching)
%       move_width     - Width for open [m]
%       move_speed     - Speed for open [m/s]
%       grasp_width    - Expected object width for grasp [m]
%       grasp_speed    - Speed for closing [m/s]
%       grasp_force    - Grasping force [N]
%       read_state_trigger - Triggers state read
%       state_out      - Debug state
%
%   States:
%       0 = INIT     : Initialize, read state
%       1 = IDLE     : Waiting for action
%       2 = OPENING  : Moving to open position
%       3 = CLOSING  : Grasping with force
%       4 = HOLDING  : Object grasped, maintaining grip
%       5 = STOPPING : Transitioning between open/close
%
%   Copyright (c) 2025 Franka Robotics GmbH

    %#codegen
    
    %% ========================================================================
    %  PARAMETERS
    %  ========================================================================
    
    OPEN_WIDTH = 0.08;       % [m] - fully open
    GRASP_WIDTH = 0.0;       % [m] - close fully (gripper stops on object)
    GRASP_SPEED = 0.05;      % [m/s] - slower for grasping
    GRASP_FORCE = 40.0;      % [N] - holding force
    OPEN_SPEED = 0.1;        % [m/s] - faster for opening
    
    % Epsilon for grasp success detection
    GRASP_EPSILON_INNER = 0.005;  % [m]
    GRASP_EPSILON_OUTER = 0.005;  % [m]
    
    % Action threshold
    ACTION_THRESHOLD = 0.5;
    
    % Timing
    SAMPLES_WAIT = 3;
    SAMPLES_READ = 10;
    
    %% ========================================================================
    %  STATES
    %  ========================================================================
    
    S_INIT = uint8(0);
    S_IDLE = uint8(1);
    S_OPENING = uint8(2);
    S_CLOSING = uint8(3);
    S_HOLDING = uint8(4);
    S_STOPPING = uint8(5);
    
    %% ========================================================================
    %  PERSISTENT STATE
    %  ========================================================================
    
    persistent state;
    persistent last_action;     % 0 = open, 1 = close
    persistent wait_counter;
    persistent was_enabled;
    persistent pending_action;  % Action to execute after stop
    persistent init;
    
    if isempty(init)
        state = S_INIT;
        last_action = 0;  % Start with open
        wait_counter = 0;
        was_enabled = false;
        pending_action = 0;
        init = true;
    end
    
    %% ========================================================================
    %  DEFAULT OUTPUTS
    %  ========================================================================
    
    move_trigger = 0;
    grasp_trigger = 0;
    stop_trigger = 0;
    read_state_trigger = 0;
    move_width = OPEN_WIDTH;
    move_speed = OPEN_SPEED;
    grasp_width = GRASP_WIDTH;
    grasp_speed = GRASP_SPEED;
    grasp_force = GRASP_FORCE;
    state_out = double(state);
    
    %% ========================================================================
    %  INPUT PROCESSING
    %  ========================================================================
    
    % Binary action: 0 = open, 1 = close
    want_close = gripper_action >= ACTION_THRESHOLD;
    current_action = double(want_close);
    
    gripper_busy = (command_status > 0.5) && (command_status < 1.5);
    
    % Handle enable
    if enable < 0.5
        state = S_INIT;
        was_enabled = false;
        return;
    end
    
    if (enable > 0.5) && ~was_enabled
        state = S_INIT;
    end
    was_enabled = true;
    
    % Detect action change
    action_changed = (current_action ~= last_action);
    
    %% ========================================================================
    %  STATE MACHINE
    %  ========================================================================
    
    switch state
        
        case S_INIT
            % Initialize: read state
            read_state_trigger = 1;
            last_action = current_action;
            wait_counter = 0;
            
            if want_close
                state = S_CLOSING;
                grasp_trigger = 1;
            else
                state = S_OPENING;
                move_trigger = 1;
            end
            
        case S_IDLE
            % Periodic read
            wait_counter = wait_counter + 1;
            if wait_counter >= SAMPLES_READ
                read_state_trigger = 1;
                wait_counter = 0;
            end
            
            % Check for action change
            if action_changed
                last_action = current_action;
                if want_close
                    grasp_trigger = 1;
                    state = S_CLOSING;
                else
                    move_trigger = 1;
                    state = S_OPENING;
                end
                wait_counter = 0;
            end
            
        case S_OPENING
            wait_counter = wait_counter + 1;
            if wait_counter >= SAMPLES_READ
                read_state_trigger = 1;
                wait_counter = 0;
            end
            
            if action_changed && want_close
                % Switch to close - stop first!
                stop_trigger = 1;
                pending_action = 1;  % close
                last_action = current_action;
                state = S_STOPPING;
                wait_counter = 0;
                
            elseif ~gripper_busy
                % Open complete
                state = S_IDLE;
            end
            
        case S_CLOSING
            wait_counter = wait_counter + 1;
            if wait_counter >= SAMPLES_READ
                read_state_trigger = 1;
                wait_counter = 0;
            end
            
            if action_changed && ~want_close
                % Switch to open - stop first!
                stop_trigger = 1;
                pending_action = 0;  % open
                last_action = current_action;
                state = S_STOPPING;
                wait_counter = 0;
                
            elseif ~gripper_busy
                % Grasp complete
                if is_grasped > 0.5
                    state = S_HOLDING;
                else
                    % No object - go to idle
                    state = S_IDLE;
                end
            end
            
        case S_HOLDING
            % Object grasped, maintain
            wait_counter = wait_counter + 1;
            if wait_counter >= SAMPLES_READ
                read_state_trigger = 1;
                wait_counter = 0;
            end
            
            if action_changed && ~want_close
                % Release! Stop grasp and open
                stop_trigger = 1;
                pending_action = 0;  % open
                last_action = current_action;
                state = S_STOPPING;
                wait_counter = 0;
            end
            % Otherwise stay holding
            
        case S_STOPPING
            % Wait for stop, then execute pending action
            wait_counter = wait_counter + 1;
            
            if ~gripper_busy || wait_counter > SAMPLES_WAIT * 3
                if pending_action > 0.5
                    % Close/grasp
                    grasp_trigger = 1;
                    state = S_CLOSING;
                else
                    % Open
                    move_trigger = 1;
                    state = S_OPENING;
                end
                wait_counter = 0;
            end
            
        otherwise
            state = S_INIT;
    end
    
    %% ========================================================================
    %  OUTPUT
    %  ========================================================================
    
    state_out = double(state);
    
end

