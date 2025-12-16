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
set(franka_simulink_SRC
    "blocks/franka_robot.cpp"
)

set(Matlab_INCLUDE_DIRS 
     ${Matlab_INCLUDE_DIRS}
     ${Matlab_ROOT_DIR}/simulink/include
)

foreach(franka_simulink_function ${franka_simulink_SRC})
     get_filename_component(sfunction ${franka_simulink_function} NAME_WLE [CACHE])
     matlab_add_mex(NAME ${sfunction} 
          SRC ${franka_simulink_function}
     )
endforeach()
