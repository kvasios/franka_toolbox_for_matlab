cmake_minimum_required(VERSION 3.22)
project(franka_toolbox_simulink)
find_package(Matlab REQUIRED)

# Manually specify source files
set(franka_toolbox_simulink_SRC
    "blocks/franka_torque_control.cpp"
)

set(Matlab_INCLUDE_DIRS 
     ${Matlab_INCLUDE_DIRS}
     ${Matlab_ROOT_DIR}/simulink/include
)

foreach(franka_toolbox_simulink_function ${franka_toolbox_simulink_SRC})
     get_filename_component(sfunction ${franka_toolbox_simulink_function} NAME_WLE [CACHE])
     matlab_add_mex(NAME ${sfunction} 
          SRC ${franka_toolbox_simulink_function}
     )
endforeach()
