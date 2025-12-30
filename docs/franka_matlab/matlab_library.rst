.. _matlab-library:

Franka Library for MATLAB - Reference
=====================================

FrankaRobot Class
-----------------

The ``FrankaRobot`` constructor initializes a connection to the Franka robot. It can be configured for two primary scenarios: connecting to a robot on a local network (Host PC) or connecting to a robot via an external AI companion computer like a Jetson.

Multiple ``FrankaRobot`` instances can be created simultaneously, each managing its own server lifecycle independently.

**Local Host PC as Target PC**

.. code-block:: matlab

    fr = FrankaRobot();  % Uses default robot IP (172.16.0.2)

    % Or with custom robot IP
    fr = FrankaRobot('RobotIP', '172.16.0.2');

    % Or with a full settings object for advanced configuration
    settings = FrankaRobotSettings();
    settings.robot_ip = '172.16.0.2';
    settings.home_configuration = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4];
    fr = FrankaRobot('Settings', settings);

**Connecting via AI Companion/NVIDIA Jetson**

When using an external Target PC to control the robot, you must provide connection details for that computer, including its IP address and a username.

.. warning::

    Before attempting to connect to the robot via an external AI companion or NVIDIA Jetson, 
    ensure that you have copied your SSH key to the target PC, e.g with ``ssh-copy-id`` for Linux. 
    This step is crucial for establishing an SSH connection without requiring a password each time.

.. code-block:: matlab

    fr = FrankaRobot('RobotIP', '172.16.0.2', ...
                     'Username', 'jetson_user', ...
                     'ServerIP', '192.168.1.100');

    % Or with custom settings object
    settings = FrankaRobotSettings();
    settings.home_configuration = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4];
    fr = FrankaRobot('RobotIP', '172.16.0.2', ...
                     'Settings', settings, ...
                     'Username', 'jetson_user', ...
                     'ServerIP', '192.168.1.100');

All constructor parameters are optional and have default values.

Parameters:
    - RobotIP: IP address of the Franka robot (default: '172.16.0.2').
      Overrides ``Settings.robot_ip`` if both are provided.
    - Settings: ``FrankaRobotSettings`` object containing robot configuration (optional).
      Other settings like ``collision_thresholds`` and ``load_inertia`` can be modified at runtime.
    - Username: Username for the server on the AI companion (default: 'franka')
    - ServerIP: IP address of the server on the AI companion (default: '172.16.1.2')
    - SSHPort: SSH port for server connection (default: '22')
    - ServerPort: Server port for communication (default: '5001')

Automatic Error Recovery
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    fr.automatic_error_recovery();

Attempts an automatic error recovery of the robot.

Get Joint Poses
^^^^^^^^^^^^^^^

.. code-block:: matlab

    jp = fr.joint_poses();

Returns a 7-element array with the current robot joint poses.

Read Robot State
^^^^^^^^^^^^^^^^

.. code-block:: matlab

    rs = fr.state();

Returns a struct with the current robot state.

Joint Point to Point Motion
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    % Synchronous (blocking)
    fr.joint_point_to_point_motion(joints_target_configuration, speed_factor);

    % Asynchronous (non-blocking)
    fr.joint_point_to_point_motion(joints_target_configuration, speed_factor, 'Async', true);
    fr.joint_point_to_point_motion(joints_target_configuration, speed_factor, 'Async', true, 'Timeout', 60);

    % With motion recording (async only)
    fr.joint_point_to_point_motion(joints_target_configuration, speed_factor, 'Async', true, 'Record', true);

Moves the robot into a desired joint configuration.

Parameters:
    - joints_target_configuration: 7-element double array with target configuration
    - speed_factor: Scalar between 0 and 1 (default: 0.5)

Name-Value Arguments:
    - 'Async': If true, return immediately without waiting (default: false).
    - 'Timeout': Maximum time for async command in seconds (default: 60).
    - 'Record': If true, record robot state at 1kHz during motion (default: false). Retrieve with ``read_recording()``.

Returns:
    - Sync mode: true if motion was successful, false otherwise.
    - Async mode: true if command was started successfully.

Joint Trajectory Motion
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    % Synchronous (blocking)
    fr.joint_trajectory_motion(positions);

    % Asynchronous (non-blocking)
    fr.joint_trajectory_motion(positions, 'Async', true);
    fr.joint_trajectory_motion(positions, 'Async', true, 'Timeout', 120);

    % With motion recording (async only)
    fr.joint_trajectory_motion(positions, 'Async', true, 'Record', true);

Moves the robot based on the given desired joint trajectory.

Parameters:
    - positions: 7xN double array with desired joint trajectory (1ms per column)

Name-Value Arguments:
    - 'Async': If true, return immediately without waiting (default: false).
    - 'Timeout': Maximum time for async command in seconds (default: auto-calculated).
    - 'Record': If true, record robot state at 1kHz during motion (default: false). Retrieve with ``read_recording()``.

Returns:
    - Sync mode: true if motion was successful, false otherwise.
    - Async mode: true if command was started successfully.

.. warning::
    Make sure that the current configuration of the robot matches the initial trajectory element `q(1:7,1)` that is passed in the function! Additionally make sure that
    the given trajectory is sufficiently smooth and continuous.

Get Motion Status
^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    status = fr.motion_status();

Returns a struct with the current motion command status:

- ``q``: Current joint positions (1x7)
- ``dq``: Current joint velocities (1x7)
- ``command_status``: One of 'idle', 'busy', 'success', 'failed', 'timeout', or 'stopped'
- ``last_command``: Name of last/current command
- ``error_message``: Error message if failed
- ``progress``: Motion progress 0.0-1.0

Wait for Motion
^^^^^^^^^^^^^^^

.. code-block:: matlab

    status = fr.motion_wait(timeout);

Blocks until the current async motion completes or timeout is reached.

Parameters:
    - timeout: Maximum wait time in seconds (default: 120).

Returns:
    - Same struct as ``motion_status()`` after command completes.

Check if Motion Busy
^^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    busy = fr.motion_isBusy();

Returns true if a motion command is currently in progress.

Read Motion Recording
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    recording = fr.read_recording();

Retrieves recorded motion data from the last motion executed with ``'Record', true``.

Returns a struct with:
    - ``command_name``: Name of the motion command that was recorded
    - ``duration``: Total motion duration in seconds
    - ``success``: Whether the motion completed successfully
    - ``num_samples``: Number of recorded samples (typically 1000/sec)
    - ``timestamp``: 1xN array of timestamps (seconds from motion start)
    - ``q``: 7xN array of actual joint positions (rad)
    - ``q_d``: 7xN array of desired joint positions (rad)
    - ``dq``: 7xN array of actual joint velocities (rad/s)
    - ``dq_d``: 7xN array of desired joint velocities (rad/s)
    - ``tau_J``: 7xN array of measured joint torques (Nm)
    - ``tau_ext_hat_filtered``: 7xN array of external torques (Nm)
    - ``O_T_EE``: 16xN array of end-effector poses (column-major 4x4 matrices)

**Async Motion Examples**

.. code-block:: matlab

    % Example: Async point-to-point motion with polling
    q_target = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4];
    fr.joint_point_to_point_motion(q_target, 0.3, 'Async', true);
    while fr.motion_isBusy()
        s = fr.motion_status();
        fprintf('Progress: %.1f%%, q1=%.3f\n', s.progress*100, s.q(1));
        pause(0.1);
    end

    % Example: Async trajectory motion with wait
    trajectory = generate_trajectory();  % 7xN array
    fr.joint_trajectory_motion(trajectory, 'Async', true);
    result = fr.motion_wait(60);  % Wait up to 60 seconds
    if strcmp(result.command_status, 'success')
        disp('Trajectory completed!');
    end

    % Example: Stop during async motion
    fr.joint_point_to_point_motion(q_target, 0.1, 'Async', true);
    pause(1);
    fr.stop();  % Interrupt the motion

    % Example: Record motion data for analysis
    fr.joint_point_to_point_motion(q_target, 0.5, 'Async', true, 'Record', true);
    fr.motion_wait();
    rec = fr.read_recording();
    
    % Plot joint positions over time
    figure;
    subplot(2,2,1);
    plot(rec.timestamp, rec.q');
    title('Joint Positions'); xlabel('Time (s)'); ylabel('rad');
    legend('q1','q2','q3','q4','q5','q6','q7');
    
    subplot(2,2,2);
    plot(rec.timestamp, rec.dq');
    title('Joint Velocities'); xlabel('Time (s)'); ylabel('rad/s');
    
    subplot(2,2,3);
    plot(rec.timestamp, rec.tau_J');
    title('Measured Torques'); xlabel('Time (s)'); ylabel('Nm');
    
    subplot(2,2,4);
    plot(rec.timestamp, rec.tau_ext_hat_filtered');
    title('External Torques'); xlabel('Time (s)'); ylabel('Nm');

Collision Thresholds
^^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    fr.setCollisionThresholds(thresholds);
    thresholds = fr.getCollisionThresholds();

Sets or gets the collision thresholds for the robot.

Parameters:
    - thresholds: ``FrankaRobotCollisionThresholds`` object

Load Inertia
^^^^^^^^^^^^

.. code-block:: matlab

    fr.setLoadInertia(loadInertia);
    inertia = fr.getLoadInertia();

Sets or gets the load inertia parameters for the robot.

Parameters:
    - loadInertia: ``FrankaRobotLoadInertia`` object (mass, center_of_mass, inertia_matrix)

Robot Homing
^^^^^^^^^^^^

.. code-block:: matlab

    result = fr.homing();

Moves the robot to its home configuration using point-to-point motion.

Returns:
    - true if the motion was successful, false otherwise

Reset Settings
^^^^^^^^^^^^^^

.. code-block:: matlab

    fr.resetSettings();

Resets all robot settings to their default values and applies them to the robot.

Joint Impedance
^^^^^^^^^^^^^^^

.. code-block:: matlab

    fr.setJointImpedance(K_theta);
    fr.setJointImpedance();  % Uses Settings.joint_impedance_stiffness
    K_theta = fr.getJointImpedance();

Sets or gets the impedance for each joint in the internal controller.
The value is stored in ``Settings.joint_impedance_stiffness``.

Parameters:
    - K_theta: 7-element array of joint stiffness values [Nm/rad]

Default values: ``[3000, 3000, 3000, 2500, 2500, 2000, 2000]``

Cartesian Impedance
^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    fr.setCartesianImpedance(K_x);
    fr.setCartesianImpedance();  % Uses Settings.cartesian_impedance_stiffness
    K_x = fr.getCartesianImpedance();

Sets or gets the Cartesian stiffness/compliance in the internal controller.
The value is stored in ``Settings.cartesian_impedance_stiffness``.

Parameters:
    - K_x: 6-element array for (x, y, z, roll, pitch, yaw) stiffness
      [N/m, N/m, N/m, Nm/rad, Nm/rad, Nm/rad]

Default values: ``[3000, 3000, 3000, 300, 300, 300]``

Guiding Mode
^^^^^^^^^^^^

.. code-block:: matlab

    fr.setGuidingMode(guiding_mode, elbow);

Locks or unlocks guiding mode movement in (x, y, z, roll, pitch, yaw) directions.

Parameters:
    - guiding_mode: 6-element logical array where ``true`` = unlocked, ``false`` = locked
    - elbow: logical scalar, ``true`` = unlock elbow movement

Returns:
    - true if successful, false otherwise

Example:

.. code-block:: matlab

    % Unlock all directions and elbow
    fr.setGuidingMode([true true true true true true], true);
    
    % Lock rotation, allow translation and elbow
    fr.setGuidingMode([true true true false false false], true);

End Effector Frame (NE_T_EE)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    fr.setEE(NE_T_EE);
    fr.setEE();  % Uses Settings.NE_T_EE
    NE_T_EE = fr.getEE();

Sets or gets the transformation from nominal end effector to end effector frame.
The value is stored in ``Settings.NE_T_EE``.

Parameters:
    - NE_T_EE: 4x4 homogeneous transformation matrix

Default: Identity matrix (eye(4))

Stiffness Frame (EE_T_K)
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    fr.setK(EE_T_K);
    fr.setK();  % Uses Settings.EE_T_K
    EE_T_K = fr.getK();

Sets or gets the transformation from end effector frame to stiffness frame.
The value is stored in ``Settings.EE_T_K``.

Parameters:
    - EE_T_K: 4x4 homogeneous transformation matrix

Default: Identity matrix (eye(4))

Stop Robot
^^^^^^^^^^

.. code-block:: matlab

    result = fr.stop();

Stops all currently running motions on the robot.

Returns:
    - true if successful, false otherwise

Gripper Control
^^^^^^^^^^^^^^^

The FrankaRobot class provides access to both standard and vacuum grippers through the following properties:

.. code-block:: matlab

    fr.Gripper        % Standard gripper interface
    fr.VacuumGripper  % Vacuum gripper interface

See the respective gripper class documentation for available methods.

FrankaGripper Class
-------------------

The ``FrankaGripper`` class is accessed through the ``Gripper`` property of a ``FrankaRobot`` instance.
It provides both synchronous (blocking) and asynchronous (non-blocking) execution modes.

**Synchronous vs Asynchronous Execution**

.. code-block:: matlab

    % Synchronous (blocking, default) - waits until complete
    fr.Gripper.move(0.08, 0.1);
    fr.Gripper.grasp(0.02, 0.1, 40);

    % Asynchronous (non-blocking) - returns immediately
    fr.Gripper.move(0.08, 0.1, 'Async', true);
    fr.Gripper.grasp(0.02, 0.1, 40, 'Async', true);

Get Gripper State
^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    state = fr.Gripper.state();

Returns a struct with the current gripper state containing: ``width``, ``max_width``, 
``is_grasped``, ``temperature``, ``time_stamp``.

Gripper Homing
^^^^^^^^^^^^^^

.. code-block:: matlab

    result = fr.Gripper.homing();

Performs gripper homing (always blocking). Returns true if successful.

Move Gripper
^^^^^^^^^^^^

.. code-block:: matlab

    % Synchronous (blocking)
    result = fr.Gripper.move(width, speed);

    % Asynchronous (non-blocking)
    result = fr.Gripper.move(width, speed, 'Async', true);
    result = fr.Gripper.move(width, speed, 'Async', true, 'Timeout', 15);

Moves the gripper to a specific width.

Parameters:
    - width: Target width in meters.
    - speed: Speed of the motion (default: 0.1 m/s).

Name-Value Arguments:
    - 'Async': If true, return immediately without waiting (default: false).
    - 'Timeout': Maximum time for async command in seconds (default: 15).

Returns:
    - Sync mode: true if motion was successful, false otherwise.
    - Async mode: true if command was started successfully.

Grasp Object
^^^^^^^^^^^^

.. code-block:: matlab

    % Synchronous (blocking)
    result = fr.Gripper.grasp(width, speed, force, epsilon_inner, epsilon_outer);

    % Asynchronous (non-blocking)
    result = fr.Gripper.grasp(width, speed, force, 'Async', true);
    result = fr.Gripper.grasp(width, speed, force, eps_in, eps_out, 'Async', true);

Grasps an object with the specified width.

Parameters:
    - width: Target width in meters.
    - speed: Speed of the motion (default: 0.1 m/s).
    - force: Grasping force in N (default: 50 N).
    - epsilon_inner: Inner tolerance in meters (default: 0.005 m).
    - epsilon_outer: Outer tolerance in meters (default: 0.005 m).

Name-Value Arguments:
    - 'Async': If true, return immediately without waiting (default: false).
    - 'Timeout': Maximum time for async command in seconds (default: 15).

Returns:
    - Sync mode: true if grasping was successful, false otherwise.
    - Async mode: true if command was started successfully.

Stop Gripper
^^^^^^^^^^^^

.. code-block:: matlab

    result = fr.Gripper.stop();

Stops the gripper motion. Can interrupt async commands. Returns true if successful.

Get Async Status
^^^^^^^^^^^^^^^^

.. code-block:: matlab

    status = fr.Gripper.status();

Returns a struct with the current async command status and gripper state:

- ``width``, ``max_width``, ``is_grasped``, ``temperature``, ``time_stamp``: Gripper state fields
- ``command_status``: One of 'idle', 'busy', 'success', 'failed', 'timeout', or 'stopped'
- ``last_command``: Name of last/current command
- ``error_message``: Error message if command failed

.. note::
    For ``grasp()``, 'success' means the command completed without error.
    Whether an object is actually held is indicated by ``is_grasped``.

Wait for Async Command
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    status = fr.Gripper.wait(timeout);

Blocks until the current async command completes or timeout is reached.

Parameters:
    - timeout: Maximum wait time in seconds (default: 30).

Returns:
    - Same struct as ``status()`` after command completes.

Check if Busy
^^^^^^^^^^^^^

.. code-block:: matlab

    busy = fr.Gripper.isBusy();

Returns true if an async command is currently in progress.

**Async Usage Examples**

.. code-block:: matlab

    % Example: Async move with polling
    fr.Gripper.move(0.08, 0.1, 'Async', true);
    while fr.Gripper.isBusy()
        s = fr.Gripper.status();
        fprintf('Width: %.3f m\n', s.width);
        pause(0.1);
    end

    % Example: Async grasp with wait
    fr.Gripper.grasp(0.02, 0.1, 40, 'Async', true);
    result = fr.Gripper.wait(10);  % Wait up to 10 seconds
    if strcmp(result.command_status, 'success') && result.is_grasped
        disp('Object grasped!');
    end

    % Example: Stop during async command
    fr.Gripper.move(0.08, 0.02, 'Async', true);
    pause(1);
    fr.Gripper.stop();  % Interrupt the move

FrankaVacuumGripper Class
-------------------------

The ``FrankaVacuumGripper`` class is accessed through the ``VacuumGripper`` property of a ``FrankaRobot`` instance.
It provides both synchronous (blocking) and asynchronous (non-blocking) execution modes.

**Synchronous vs Asynchronous Execution**

.. code-block:: matlab

    % Synchronous (blocking, default) - waits until complete
    fr.VacuumGripper.vacuum(0, 5000, 0);
    fr.VacuumGripper.dropOff(5000);

    % Asynchronous (non-blocking) - returns immediately
    fr.VacuumGripper.vacuum(0, 5000, 0, 'Async', true);
    fr.VacuumGripper.dropOff(5000, 'Async', true);

Get Vacuum Gripper State
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    state = fr.VacuumGripper.state();

Returns a struct with the current vacuum gripper state containing: ``in_control_range``, 
``part_detached``, ``part_present``, ``device_status``, ``actual_power``, ``vacuum``, ``time``.

Apply Vacuum
^^^^^^^^^^^^

.. code-block:: matlab

    % Synchronous (blocking)
    result = fr.VacuumGripper.vacuum(control_point, timeout, profile);

    % Asynchronous (non-blocking)
    result = fr.VacuumGripper.vacuum(control_point, timeout, profile, 'Async', true);
    result = fr.VacuumGripper.vacuum(control_point, timeout, profile, 'Async', true, 'Timeout', 15);

Applies vacuum to the gripper.

Parameters:
    - control_point: Vacuum control point (default: 0).
    - timeout: Timeout in milliseconds (default: 5000).
    - profile: Production setup profile (default: 0).

Name-Value Arguments:
    - 'Async': If true, return immediately without waiting (default: false).
    - 'Timeout': Maximum time for async command in seconds (default: 15).

Returns:
    - Sync mode: true if vacuum was successfully applied, false otherwise.
    - Async mode: true if command was started successfully.

Drop Off
^^^^^^^^

.. code-block:: matlab

    % Synchronous (blocking)
    result = fr.VacuumGripper.dropOff(timeout);

    % Asynchronous (non-blocking)
    result = fr.VacuumGripper.dropOff(timeout, 'Async', true);
    result = fr.VacuumGripper.dropOff(timeout, 'Async', true, 'Timeout', 15);

Drops off the currently held object.

Parameters:
    - timeout: Timeout in milliseconds (default: 5000).

Name-Value Arguments:
    - 'Async': If true, return immediately without waiting (default: false).
    - 'Timeout': Maximum time for async command in seconds (default: 15).

Returns:
    - Sync mode: true if drop off was successful, false otherwise.
    - Async mode: true if command was started successfully.

Stop Vacuum Gripper
^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    result = fr.VacuumGripper.stop();

Stops the vacuum gripper. Can interrupt async commands. Returns true if successful.

Get Async Status
^^^^^^^^^^^^^^^^

.. code-block:: matlab

    status = fr.VacuumGripper.status();

Returns a struct with the current async command status and vacuum gripper state:

- ``in_control_range``, ``part_detached``, ``part_present``, ``device_status``, 
  ``actual_power``, ``vacuum``, ``time``: Vacuum gripper state fields
- ``command_status``: One of 'idle', 'busy', 'success', 'failed', 'timeout', or 'stopped'
- ``last_command``: Name of last/current command
- ``error_message``: Error message if command failed

Wait for Async Command
^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: matlab

    status = fr.VacuumGripper.wait(timeout);

Blocks until the current async command completes or timeout is reached.

Parameters:
    - timeout: Maximum wait time in seconds (default: 30).

Returns:
    - Same struct as ``status()`` after command completes.

Check if Busy
^^^^^^^^^^^^^

.. code-block:: matlab

    busy = fr.VacuumGripper.isBusy();

Returns true if an async command is currently in progress.

**Async Usage Examples**

.. code-block:: matlab

    % Example: Async vacuum with polling
    fr.VacuumGripper.vacuum(0, 5000, 0, 'Async', true);
    while fr.VacuumGripper.isBusy()
        s = fr.VacuumGripper.status();
        fprintf('Vacuum: %.1f, Part present: %d\n', s.vacuum, s.part_present);
        pause(0.1);
    end

    % Example: Async drop off with wait
    fr.VacuumGripper.dropOff(5000, 'Async', true);
    result = fr.VacuumGripper.wait(10);  % Wait up to 10 seconds
    if strcmp(result.command_status, 'success')
        disp('Part released!');
    end

    % Example: Stop during async command
    fr.VacuumGripper.vacuum(0, 5000, 0, 'Async', true, 'Timeout', 30);
    pause(1);
    fr.VacuumGripper.stop();  % Interrupt the vacuum
