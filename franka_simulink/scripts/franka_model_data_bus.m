function bus = franka_model_data_bus()
%FRANKA_MODEL_DATA_BUS Creates Simulink bus for computed dynamics/kinematics
%
%   bus = FRANKA_MODEL_DATA_BUS() returns a Simulink.Bus object containing
%   model-computed signals from libfranka's Model class. These are derived
%   quantities computed from the robot state each control cycle.
%
%   Usage:
%       % Create and register the bus in base workspace
%       FrankaModelDataBus = franka_model_data_bus();
%       assignin('base', 'FrankaModelDataBus', FrankaModelDataBus);
%
%   The bus contains:
%     - Mass matrix M(q) for dynamics control
%     - Coriolis vector c(q,dq) for compensation
%     - Gravity vector g(q) for gravity compensation
%     - Jacobians for Cartesian control
%
%   These signals are commonly used in:
%     - Computed torque control: tau = M*ddq_d + c + g
%     - Gravity compensation: tau = g
%     - Cartesian impedance: tau = J' * F
%
%   See also: franka::Model in libfranka documentation
%
%   Copyright (c) 2025 Franka Robotics GmbH

    elems = Simulink.BusElement.empty;
    idx = 0;
    
    %% ====================================================================
    %% Dynamics Matrices
    %% ====================================================================
    
    % mass: 7x7 mass (inertia) matrix M(q) [kg*m^2]
    % Used in: tau = M(q)*ddq + c(q,dq) + g(q)
    idx = idx + 1;
    elems(idx) = create_element('mass', [7 7], 'double', ...
        'Mass matrix M(q) [kg*m^2]');
    
    % coriolis: 7x1 Coriolis force vector c = C(q,dq)*dq [Nm]
    idx = idx + 1;
    elems(idx) = create_element('coriolis', [7 1], 'double', ...
        'Coriolis force vector c(q,dq) [Nm]');
    
    % gravity: 7x1 gravity compensation vector g(q) [Nm]
    idx = idx + 1;
    elems(idx) = create_element('gravity', [7 1], 'double', ...
        'Gravity vector g(q) [Nm]');
    
    %% ====================================================================
    %% Jacobians (End Effector frame, in base frame coordinates)
    %% ====================================================================
    
    % jacobian: 6x7 geometric Jacobian of EE in base frame (zero Jacobian)
    % dx = J * dq, where dx = [v; omega] is EE twist in base frame
    % Used in: tau = J' * F for Cartesian force control
    idx = idx + 1;
    elems(idx) = create_element('jacobian', [6 7], 'double', ...
        'End effector Jacobian in base frame (6x7)');
    
    % jacobian_body: 6x7 body Jacobian of EE (in EE frame)
    % dx_body = J_body * dq, where dx_body is EE twist in EE frame
    idx = idx + 1;
    elems(idx) = create_element('jacobian_body', [6 7], 'double', ...
        'End effector body Jacobian (6x7)');
    
    %% ====================================================================
    %% Create the Bus
    %% ====================================================================
    
    bus = Simulink.Bus();
    bus.Description = 'Computed dynamics and kinematics from libfranka Model';
    bus.Elements = elems;
    
    % Tell Simulink Coder to use our pre-defined C struct
    bus.HeaderFile = 'franka_simulink_types.h';
    bus.DataScope = 'Imported';
end

function elem = create_element(name, dims, datatype, description)
%CREATE_ELEMENT Helper to create a Simulink.BusElement
    elem = Simulink.BusElement();
    elem.Name = name;
    elem.Dimensions = dims;
    elem.DataType = datatype;
    elem.Description = description;
    elem.Complexity = 'real';
    elem.SamplingMode = 'Sample based';
end
