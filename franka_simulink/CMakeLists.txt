cmake_minimum_required(VERSION 3.22)
project(franka_simulink)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra)

find_package(Matlab REQUIRED)

# ============================================================================
# MEX S-Function (for Simulink simulation)
# ============================================================================
set(Matlab_INCLUDE_DIRS 
     ${Matlab_INCLUDE_DIRS}
     ${Matlab_ROOT_DIR}/simulink/include
)

#
# IMPORTANT: do NOT name this S-function "franka_robot" because the toolbox also
# builds a MATLAB API MEX with that name (franka_matlab/bin/franka_robot.mex*),
# and MATLAB path resolution can cause Simulink to load the wrong binary.
#
matlab_add_mex(
     NAME franka_robot_sfunction
     SRC "blocks/franka_robot_sfunction.cpp"
)

# ============================================================================
# Gripper S-Function (for Simulink simulation)
# ============================================================================
# Async finger gripper control with command worker thread.
# Commands (homing, grasp, move, stop) triggered by rising edges.
#
matlab_add_mex(
     NAME franka_gripper_sfunction
     SRC "blocks/franka_gripper_sfunction.cpp"
)

# ============================================================================
# Vacuum Gripper S-Function (for Simulink simulation)
# ============================================================================
# Async vacuum gripper control with command worker thread.
# Commands (vacuum, drop_off, stop) triggered by rising edges.
#
matlab_add_mex(
     NAME franka_vacuum_gripper_sfunction
     SRC "blocks/franka_vacuum_gripper_sfunction.cpp"
)

# ============================================================================
# Motion Generator S-Function (standalone, no external dependencies)
# ============================================================================
# Generic joint-space trajectory generator using synchronized polynomial profiles.
# Works in Simulink simulation mode and code generation (fully inlined via TLC).
#
matlab_add_mex(
     NAME motion_generator_sfunction
     SRC "blocks/motion_generator_sfunction.cpp"
)

# ============================================================================
# Robot Model S-Functions (for computing dynamics/kinematics from arbitrary q)
# ============================================================================
# These blocks compute model data (mass, coriolis, gravity, jacobian, pose)
# from explicit joint positions, unlike the franka_robot block which uses
# the current robot state.
#
matlab_add_mex(
     NAME franka_robot_model_mass_sfunction
     SRC "blocks/franka_robot_model_mass_sfunction.cpp"
)

matlab_add_mex(
     NAME franka_robot_model_coriolis_sfunction
     SRC "blocks/franka_robot_model_coriolis_sfunction.cpp"
)

matlab_add_mex(
     NAME franka_robot_model_gravity_sfunction
     SRC "blocks/franka_robot_model_gravity_sfunction.cpp"
)

matlab_add_mex(
     NAME franka_robot_model_jacobian_sfunction
     SRC "blocks/franka_robot_model_jacobian_sfunction.cpp"
)

matlab_add_mex(
     NAME franka_robot_model_pose_sfunction
     SRC "blocks/franka_robot_model_pose_sfunction.cpp"
)
