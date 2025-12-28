# Changelog:

## 5.0.0 (Unreleased)

   - New auto-build system for franka.mtlbx under github release page. 
   - Default build --> libfranka 0.19.0.
   - FrankaRobot API: Added `FrankaRobotSettings` class for centralized robot configuration.
   - FrankaRobot API: New methods for impedance control (`setJointImpedance`, `setCartesianImpedance`), guiding mode (`setGuidingMode`), frame transformations (`setEE`, `setK`), and motion control (`stop`).
   - FrankaRobot API: Improved server lifecycle handling.
   - FrankaRobot API: Support for multiple `FrankaRobot` instances.
   - FrankaRobot API: Renamed `robot_state()` to `state()` and `robot_homing()` to `homing()`.
   - FrankaGripper API: Added async support for `move()` and `grasp()` with `'Async', true` option. New methods: `status()`, `wait()`, `isBusy()` for non-blocking command execution and monitoring.
   - FrankaVacuumGripper API: Added async support for `vacuum()` and `dropOff()` with `'Async', true` option. New methods: `status()`, `wait()`, `isBusy()` for non-blocking command execution and monitoring.
   - Simulink: Added `franka_gripper_sfunction` (finger gripper) with async command execution (homing/grasp/move/stop) and bus I/O (`FrankaGripperCommandBus`, `FrankaGripperStateBus`).
   - Simulink: Added `franka_vacuum_gripper_sfunction` (vacuum gripper) with async command execution (vacuum/drop_off/stop) and bus I/O (`FrankaVacuumGripperCommandBus`, `FrankaVacuumGripperStateBus`).
   - Simulink: Added MATLAB bus scripts and default command helpers for both grippers; `franka_setup_bus` now registers all gripper buses.
   - Simulink: Restructured and renamed robot model blocks (`*_sfunction.m/tlc`):
     - `robot_ip` changed from parameter to input port (16-char ASCII) for flexible generated code.
     - Removed "robot_state_only" parameter (use `franka_robot_sfunction.model_data` for current state).
     - Model blocks compute from explicit inputs (q, dq, load parameters, transforms).
     - Integrated with `FrankaRobotManager` for lazy connection and shared robot instance.
     - Works both standalone AND under function-called subsystems.
   - Build: Updated `franka_simulink/CMakeLists.txt` to build the new gripper MEX S-Functions; updated codegen `makecfg.m` to compile the new gripper API sources.

## 3.1.0 (20-10-2025)

   - Franka Toolbox for MATLAB open source release.