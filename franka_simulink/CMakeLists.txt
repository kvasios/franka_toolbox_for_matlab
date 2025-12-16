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
     SRC "blocks/franka_robot.cpp"
)
