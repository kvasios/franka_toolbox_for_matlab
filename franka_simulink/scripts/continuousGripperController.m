function [move_trigger, move_width, move_speed, stop_trigger, read_state_trigger, state_out] = ...
    continuousGripperController(target_width, current_width, command_status, enable)
%CONTINUOUSGRIPPERCONTROLLER Smart pseudo-continuous gripper control
%
%   Direction-aware controller that minimizes interruptions:
%   - Only stops if direction of motion needs to REVERSE
%   - Lets moves complete if new target is "further along" same direction
%   - Queues next target and issues move when current completes
%
%   Inputs:
%       target_width   - Desired gripper width [m] (0 to ~0.08 for Franka)
%       current_width  - Current gripper width from gripper state [m]
%       command_status - From gripper state: 0=idle, 1=busy, 2=success, etc.
%       enable         - Enable signal (1 = active, 0 = disabled)
%
%   Outputs:
%       move_trigger       - Rising edge triggers move command
%       move_width         - Target width for move command [m]
%       move_speed         - Speed for move command [m/s]
%       stop_trigger       - Rising edge triggers stop
%       read_state_trigger - Rising edge triggers gripper state read
%       state_out          - Internal state for debugging
%
%   Copyright (c) 2025 Franka Robotics GmbH

    %#codegen
    
    %% ========================================================================
    %  TUNABLE PARAMETERS
    %  ========================================================================
    
    DEADBAND = 0.002;            % [m] - 2mm deadband
    INTERRUPT_THRESHOLD = 0.005; % [m] - 5mm behind = interrupt
    MOVE_SPEED = 0.1;            % [m/s]
    WIDTH_MIN = 0.0;
    WIDTH_MAX = 0.08;
    
    % Timing (samples) - CRITICAL for proper sequencing
    SAMPLES_WAIT_CMD = 5;        % Wait after command before checking status
    SAMPLES_READ_INTERVAL = 8;   % Periodic state read interval
    
    %% ========================================================================
    %  STATES
    %  ========================================================================
    %  0 = INIT           : Read state first
    %  1 = INIT_WAIT      : Wait for state read
    %  2 = INIT_MOVE      : Issue first move
    %  3 = WAIT_CMD       : Wait for command to be acknowledged
    %  4 = MOVING         : Move in progress, monitor
    %  5 = IDLE           : At target, checking if move needed
    %  6 = STOPPING       : Issued stop, waiting
    %  7 = WAIT_STOP      : Wait for stop to complete
    
    S_INIT = uint8(0);
    S_INIT_WAIT = uint8(1);
    S_INIT_MOVE = uint8(2);
    S_WAIT_CMD = uint8(3);
    S_MOVING = uint8(4);
    S_IDLE = uint8(5);
    S_STOPPING = uint8(6);
    S_WAIT_STOP = uint8(7);
    
    %% ========================================================================
    %  PERSISTENT STATE
    %  ========================================================================
    
    persistent state;
    persistent current_target;
    persistent move_direction;      % +1 = opening, -1 = closing, 0 = stationary
    persistent wait_counter;
    persistent was_enabled;
    persistent init;
    
    if isempty(init)
        state = S_INIT;
        current_target = 0.04;
        move_direction = 0;
        wait_counter = 0;
        was_enabled = false;
        init = true;
    end
    
    %% ========================================================================
    %  DEFAULT OUTPUTS
    %  ========================================================================
    
    move_trigger = 0;
    stop_trigger = 0;
    read_state_trigger = 0;
    move_width = current_target;
    move_speed = MOVE_SPEED;
    state_out = double(state);
    
    %% ========================================================================
    %  INPUT PROCESSING
    %  ========================================================================
    
    target_clamped = max(WIDTH_MIN, min(WIDTH_MAX, target_width));
    gripper_busy = (command_status > 0.5) && (command_status < 1.5);
    
    % Handle enable/disable
    if enable < 0.5
        state = S_INIT;
        was_enabled = false;
        return;
    end
    
    if (enable > 0.5) && ~was_enabled
        state = S_INIT;
    end
    was_enabled = true;
    
    %% ========================================================================
    %  COMPUTE ERRORS AND DIRECTION
    %  ========================================================================
    
    error_to_target = abs(target_clamped - current_width);
    
    % Determine if we need to interrupt (direction reversal)
    need_interrupt = false;
    if move_direction > 0  % Currently opening
        % Interrupt if target is significantly BEHIND (need to close)
        if target_clamped < (current_width - INTERRUPT_THRESHOLD)
            need_interrupt = true;
        end
    elseif move_direction < 0  % Currently closing
        % Interrupt if target is significantly BEHIND (need to open)  
        if target_clamped > (current_width + INTERRUPT_THRESHOLD)
            need_interrupt = true;
        end
    end
    
    %% ========================================================================
    %  STATE MACHINE
    %  ========================================================================
    
    switch state
        
        %% ---- INITIALIZATION ----
        
        case S_INIT
            % Trigger state read
            read_state_trigger = 1;
            wait_counter = 0;
            state = S_INIT_WAIT;
            
        case S_INIT_WAIT
            % Wait for state to be valid
            wait_counter = wait_counter + 1;
            if wait_counter >= SAMPLES_WAIT_CMD
                state = S_INIT_MOVE;
            end
            
        case S_INIT_MOVE
            % Issue initial move
            move_trigger = 1;
            move_width = target_clamped;
            current_target = target_clamped;
            if abs(target_clamped - current_width) > 0.001
                move_direction = sign(target_clamped - current_width);
            else
                move_direction = 0;
            end
            wait_counter = 0;
            state = S_WAIT_CMD;
            
        %% ---- WAIT FOR COMMAND ACKNOWLEDGMENT ----
        
        case S_WAIT_CMD
            % Wait before checking busy status
            wait_counter = wait_counter + 1;
            
            % Periodic read while waiting
            if wait_counter == 2
                read_state_trigger = 1;
            end
            
            if wait_counter >= SAMPLES_WAIT_CMD
                state = S_MOVING;
                wait_counter = 0;
            end
            
        %% ---- MOVING ----
        
        case S_MOVING
            wait_counter = wait_counter + 1;
            
            % Periodic state read
            if wait_counter >= SAMPLES_READ_INTERVAL
                read_state_trigger = 1;
                wait_counter = 0;
            end
            
            if ~gripper_busy
                % Move completed
                move_direction = 0;
                
                % Check if we need another move
                if error_to_target > DEADBAND
                    % Chain next move immediately
                    move_trigger = 1;
                    move_width = target_clamped;
                    current_target = target_clamped;
                    if abs(target_clamped - current_width) > 0.001
                        move_direction = sign(target_clamped - current_width);
                    end
                    state = S_WAIT_CMD;
                    wait_counter = 0;
                else
                    state = S_IDLE;
                    wait_counter = 0;
                end
                
            elseif need_interrupt
                % Direction reversal - stop!
                stop_trigger = 1;
                state = S_STOPPING;
                wait_counter = 0;
            end
            % Otherwise keep moving, don't interrupt
            
        %% ---- IDLE ----
        
        case S_IDLE
            wait_counter = wait_counter + 1;
            
            % Periodic state read
            if wait_counter >= SAMPLES_READ_INTERVAL
                read_state_trigger = 1;
                wait_counter = 0;
            end
            
            % Check if target moved enough to warrant new move
            if error_to_target > DEADBAND
                move_trigger = 1;
                move_width = target_clamped;
                current_target = target_clamped;
                if abs(target_clamped - current_width) > 0.001
                    move_direction = sign(target_clamped - current_width);
                end
                state = S_WAIT_CMD;
                wait_counter = 0;
            end
            
        %% ---- STOPPING ----
        
        case S_STOPPING
            % Wait a tick for stop to be issued
            wait_counter = 0;
            state = S_WAIT_STOP;
            
        case S_WAIT_STOP
            wait_counter = wait_counter + 1;
            
            % Read state while waiting
            if wait_counter == 2
                read_state_trigger = 1;
            end
            
            if ~gripper_busy || wait_counter >= SAMPLES_WAIT_CMD * 2
                % Stopped, issue new move
                move_trigger = 1;
                move_width = target_clamped;
                current_target = target_clamped;
                if abs(target_clamped - current_width) > 0.001
                    move_direction = sign(target_clamped - current_width);
                end
                state = S_WAIT_CMD;
                wait_counter = 0;
            end
            
        otherwise
            state = S_INIT;
    end
    
    %% ========================================================================
    %  OUTPUT
    %  ========================================================================
    
    move_width = target_clamped;
    state_out = double(state);
    
end
