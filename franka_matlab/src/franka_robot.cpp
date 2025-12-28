#include "mex.h"
#include "class_handle.hpp"
#include "franka_robot.hpp"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{	
    // Get the command string
    char cmd[64];
	if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
		mexErrMsgTxt("First input should be a command string less than 64 characters long.");
        
    // New
    if (!strcmp("new", cmd)) {
        // Check parameters
        if (nlhs != 1 || nrhs != 3)
            mexErrMsgTxt("New: One output and two inputs (IP and port) expected.");
        
        // Get the IP address
        char ip[64];
        if (mxGetString(prhs[1], ip, sizeof(ip)))
            mexErrMsgTxt("Second input should be a valid IP address string.");
        
        // Get the port number as a string
        char portStr[6]; // Assuming port number is at most 5 digits
        if (mxGetString(prhs[2], portStr, sizeof(portStr)))
            mexErrMsgTxt("Third input should be a valid port number string.");
        
        // Convert port string to integer
        int port = std::stoi(portStr);
        
        // Return a handle to a new C++ instance
        plhs[0] = convertPtr2Mat<FrankaRobot>(new FrankaRobot(ip, port));
        return;
    }
    
    // Check there is a second input, which should be the class instance handle
    if (nrhs < 2)
		mexErrMsgTxt("Second input should be a class instance handle.");
    
    // Delete
    if (!strcmp("delete", cmd)) {
        // Destroy the C++ object
        destroyObject<FrankaRobot>(prhs[1]);
        // Warn if other commands were ignored
        if (nlhs != 0 || nrhs != 2)
            mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
        return;
    }
    
    // Get the class instance pointer from the second input
    FrankaRobot* franka_robot_instance = convertMat2Ptr<FrankaRobot>(prhs[1]);
    
    // Call the various class methods
    
    // Initialize Robot
    if (!strcmp("initialize_robot", cmd)) {
        // Check parameters
        if (nlhs != 0 || nrhs != 3)
            mexErrMsgTxt("Initialize Robot: No outputs and two inputs (handle and IP) expected.");
        
        // Get the IP address
        char ip[64];
        if (mxGetString(prhs[2], ip, sizeof(ip)))
            mexErrMsgTxt("Third input should be a valid IP address string.");
        
        // Call the method
        try {
            franka_robot_instance->initializeRobot(ip);
        } catch (...) {
            mexErrMsgTxt("Failed to initialize robot");
        }
        return;
    }

    // Initialize Gripper
    if (!strcmp("initialize_gripper", cmd)) {
        // Check parameters
        if (nlhs != 0 || nrhs != 2)
            mexErrMsgTxt("Initialize Gripper: No outputs and one input (handle) expected.");
        
        // Call the method
        try {
            franka_robot_instance->initializeGripper();
        } catch (...) {
            mexErrMsgTxt("Failed to initialize gripper");
        }
        return;
    }

    // Initialize Vacuum Gripper
    if (!strcmp("initialize_vacuum_gripper", cmd)) {
        // Check parameters
        if (nlhs != 0 || nrhs != 2)
            mexErrMsgTxt("Initialize Vacuum Gripper: No outputs and one input (handle) expected.");
        
        // Call the method
        try {
            franka_robot_instance->initializeVacuumGripper();
        } catch (...) {
            mexErrMsgTxt("Failed to initialize vacuum gripper");
        }
        return;
    }

    // Automatic Error Recovery    
    if (!strcmp("automatic_error_recovery", cmd)) {
        // Check parameters
        if (nlhs != 0 || nrhs != 2)
            mexErrMsgTxt("Automatic Error Recovery: No outputs and two inputs (handle) expected.");
        // Call the method
        try {
            franka_robot_instance->automaticErrorRecovery();
        } catch (const kj::Exception& e) {
            mexErrMsgTxt(e.getDescription().cStr());
        } catch (...) {
            mexErrMsgTxt("Failed to perform automatic error recovery");
        }
        return;
    }

    // Get Robot State
    if (!strcmp("robot_state", cmd)) {
        // Check parameters
        if (nlhs != 1 || nrhs != 2)
            mexErrMsgTxt("Robot State: One output and one input (handle) expected.");
        
        try {
            auto state = franka_robot_instance->getRobotState();
            
            // Create MATLAB struct for output
            const char* fieldnames[] = {
                "O_T_EE", "O_T_EE_d", "F_T_EE", "F_T_NE", "NE_T_EE", "EE_T_K", "m_ee", "I_ee", "F_x_Cee",
                "m_load", "I_load", "F_x_Cload", "m_total", "I_total", "F_x_Ctotal", "elbow", "elbow_d",
                "elbow_c", "delbow_c", "ddelbow_c", "tau_J", "tau_J_d", "dtau_J", "q", "q_d", "dq", "dq_d",
                "ddq_d", "joint_contact", "cartesian_contact", "joint_collision", "cartesian_collision",
                "tau_ext_hat_filtered", "O_F_ext_hat_K", "K_F_ext_hat_K", "O_dP_EE_d", "O_T_EE_c", "O_dP_EE_c",
                "O_ddP_EE_c", "theta", "dtheta", "current_errors", "last_motion_errors", "control_command_success_rate"
            };
            int nfields = sizeof(fieldnames) / sizeof(fieldnames[0]);
            plhs[0] = mxCreateStructMatrix(1, 1, nfields, fieldnames);

            // Helper lambda to set array data
            auto setArrayField = [&](const char* fieldname, const auto& data, size_t size) {
                mxArray* arr = mxCreateDoubleMatrix(1, size, mxREAL);
                double* ptr = mxGetPr(arr);
                for (size_t i = 0; i < size; ++i) {
                    ptr[i] = data[i];
                }
                mxSetField(plhs[0], 0, fieldname, arr);
            };

            // Helper lambda to set matrix data
            auto setMatrixField = [&](const char* fieldname, const auto& data, size_t rows, size_t cols) {
                mxArray* arr = mxCreateDoubleMatrix(rows, cols, mxREAL);
                double* ptr = mxGetPr(arr);
                for (size_t i = 0; i < rows * cols; ++i) {
                    ptr[i] = data[i];
                }
                mxSetField(plhs[0], 0, fieldname, arr);
            };

            // Helper lambda to set boolean array data
            auto setBoolArrayField = [&](const char* fieldname, const auto& data, size_t size) {
                mxArray* arr = mxCreateLogicalMatrix(1, size);
                mxLogical* ptr = mxGetLogicals(arr);
                for (size_t i = 0; i < size; ++i) {
                    ptr[i] = data[i];
                }
                mxSetField(plhs[0], 0, fieldname, arr);
            };

            // Set transformation matrices (4x4)
            setMatrixField("O_T_EE", state.getOTEe(), 4, 4);
            setMatrixField("O_T_EE_d", state.getOTEeD(), 4, 4);
            setMatrixField("F_T_EE", state.getFTEe(), 4, 4);
            setMatrixField("F_T_NE", state.getFTNe(), 4, 4);
            setMatrixField("NE_T_EE", state.getNeTEe(), 4, 4);
            setMatrixField("EE_T_K", state.getEeTK(), 4, 4);
            setMatrixField("O_T_EE_c", state.getOTEeC(), 4, 4);

            // Set scalar values
            mxSetField(plhs[0], 0, "m_ee", mxCreateDoubleScalar(state.getMEe()));
            mxSetField(plhs[0], 0, "m_load", mxCreateDoubleScalar(state.getMLoad()));
            mxSetField(plhs[0], 0, "m_total", mxCreateDoubleScalar(state.getMTotal()));

            // Set inertia matrices (3x3)
            setMatrixField("I_ee", state.getIEe(), 3, 3);
            setMatrixField("I_load", state.getILoad(), 3, 3);
            setMatrixField("I_total", state.getITotal(), 3, 3);

            // Set center of mass positions
            setArrayField("F_x_Cee", state.getFXCee(), 3);
            setArrayField("F_x_Cload", state.getFXCload(), 3);
            setArrayField("F_x_Ctotal", state.getFXCtotal(), 3);

            // Set elbow state
            setArrayField("elbow", state.getElbow(), 2);
            setArrayField("elbow_d", state.getElbowD(), 2);
            setArrayField("elbow_c", state.getElbowC(), 2);
            setArrayField("delbow_c", state.getDelbowC(), 2);
            setArrayField("ddelbow_c", state.getDdelbowC(), 2);

            // Set joint states
            setArrayField("tau_J", state.getTauJ(), 7);
            setArrayField("tau_J_d", state.getTauJD(), 7);
            setArrayField("dtau_J", state.getDtauJ(), 7);
            setArrayField("q", state.getQ(), 7);
            setArrayField("q_d", state.getQD(), 7);
            setArrayField("dq", state.getDq(), 7);
            setArrayField("dq_d", state.getDqD(), 7);
            setArrayField("ddq_d", state.getDdqD(), 7);
            setArrayField("theta", state.getTheta(), 7);
            setArrayField("dtheta", state.getDtheta(), 7);
            setArrayField("tau_ext_hat_filtered", state.getTauExtHatFiltered(), 7);

            // Set contact and collision states
            setBoolArrayField("joint_contact", state.getJointContact(), 7);
            setBoolArrayField("joint_collision", state.getJointCollision(), 7);
            setBoolArrayField("cartesian_contact", state.getCartesianContact(), 6);
            setBoolArrayField("cartesian_collision", state.getCartesianCollision(), 6);

            // Set Cartesian states
            setArrayField("O_F_ext_hat_K", state.getOFExtHatK(), 6);
            setArrayField("K_F_ext_hat_K", state.getKFExtHatK(), 6);
            setArrayField("O_dP_EE_d", state.getODpEeD(), 6);
            setArrayField("O_dP_EE_c", state.getODpEeC(), 6);
            setArrayField("O_ddP_EE_c", state.getODdpEeC(), 6);

            // Set error states and control command success rate
            mxSetField(plhs[0], 0, "current_errors", mxCreateString(state.getCurrentErrors().cStr()));
            mxSetField(plhs[0], 0, "last_motion_errors", mxCreateString(state.getLastMotionErrors().cStr()));
            mxSetField(plhs[0], 0, "control_command_success_rate", 
                      mxCreateDoubleScalar(state.getControlCommandSuccessRate()));

        } catch (const kj::Exception& e) {
            mexErrMsgTxt(e.getDescription().cStr());
        } catch (...) {
            mexErrMsgTxt("Unknown error occurred while getting robot state");
        }
        return;
    }
    
    // Get Joint Poses
    if (!strcmp("joint_poses", cmd)) {
        if (nlhs != 1 || nrhs != 2)
            mexErrMsgTxt("Joint Poses: One output and one input (handle) expected.");
        try {
            auto poses = franka_robot_instance->getJointPoses();
            
            // Create cell array for output (10 matrices)
            plhs[0] = mxCreateCellMatrix(1, 10);
            
            // Fill each cell with a 4x4 transformation matrix
            for (size_t i = 0; i < poses.size(); i++) {
                mxArray* matrix = mxCreateDoubleMatrix(4, 4, mxREAL);
                double* data = mxGetPr(matrix);
                
                // Copy data with correct layout transformation
                for (size_t row = 0; row < 4; row++) {
                    for (size_t col = 0; col < 4; col++) {
                        // Convert from row-major to column-major
                        data[row + col*4] = poses[i][col*4 + row];
                    }
                }
                
                mxSetCell(plhs[0], i, matrix);
            }
        } catch (const kj::Exception& e) {
            mexErrMsgTxt(("Failed to get joint poses: " + std::string(e.getDescription().cStr())).c_str());
        } catch (const std::exception& e) {
            mexErrMsgTxt(("Failed to get joint poses: " + std::string(e.what())).c_str());
        } catch (...) {
            mexErrMsgTxt("Unknown error occurred while getting joint poses");
        }
        return;
    }

    // Joint Point-to-Point Motion
    if (!strcmp("joint_point_to_point_motion", cmd)) {
        if (nlhs != 1 || nrhs != 4)
            mexErrMsgTxt("Joint Point-to-Point Motion: One output and three inputs (handle, target_config, speed_factor) expected.");
        
        if (!mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 7)
            mexErrMsgTxt("Target configuration must be a 7-element double array.");
        
        if (!mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != 1)
            mexErrMsgTxt("Speed factor must be a scalar double.");

        try {
            std::array<double, 7> target_config;
            double* target_ptr = mxGetPr(prhs[2]);
            for (size_t i = 0; i < 7; i++) {
                target_config[i] = target_ptr[i];
            }
            double speed_factor = mxGetScalar(prhs[3]);
            
            bool success = franka_robot_instance->jointPointToPointMotion(target_config, speed_factor);
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to execute joint point-to-point motion");
        }
        return;
    }

    // Joint Trajectory Motion
    if (!strcmp("joint_trajectory_motion", cmd)) {
        if (nlhs != 1 || nrhs != 3)
            mexErrMsgTxt("Joint Trajectory Motion: One output and two inputs (handle, trajectory) expected.");
        
        if (!mxIsDouble(prhs[2]))
            mexErrMsgTxt("Trajectory must be a 7xN double array.");

        try {
            size_t num_points = mxGetN(prhs[2]);
            if (mxGetM(prhs[2]) != 7)
                mexErrMsgTxt("Trajectory must be a 7xN array.");

            std::vector<std::array<double, 7>> positions(num_points);
            double* pos_ptr = mxGetPr(prhs[2]);

            for (size_t i = 0; i < num_points; i++) {
                for (size_t j = 0; j < 7; j++) {
                    positions[i][j] = pos_ptr[i*7 + j];
                }
            }

            bool success = franka_robot_instance->jointTrajectoryMotion(positions);
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to execute joint trajectory motion");
        }
        return;
    }

    // Get Gripper State
    if (!strcmp("gripper_state", cmd)) {
        if (nlhs != 1 || nrhs != 2)
            mexErrMsgTxt("Gripper State: One output and one input (handle) expected.");
        try {
            auto state = franka_robot_instance->getGripperState();
            plhs[0] = mxCreateStructMatrix(1, 1, 0, nullptr);
            
            mxAddField(plhs[0], "width");
            mxSetField(plhs[0], 0, "width", mxCreateDoubleScalar(state.getWidth()));
            
            mxAddField(plhs[0], "max_width");
            mxSetField(plhs[0], 0, "max_width", mxCreateDoubleScalar(state.getMaxWidth()));
            
            mxAddField(plhs[0], "is_grasped");
            mxSetField(plhs[0], 0, "is_grasped", mxCreateLogicalScalar(state.getIsGrasped()));
            
            mxAddField(plhs[0], "temperature");
            mxSetField(plhs[0], 0, "temperature", mxCreateDoubleScalar(state.getTemperature()));
            
            mxAddField(plhs[0], "time_stamp");
            mxSetField(plhs[0], 0, "time_stamp", mxCreateDoubleScalar(state.getTimeStamp()));
        } catch (...) {
            mexErrMsgTxt("Failed to get gripper state");
        }
        return;
    }

    // Gripper Homing
    if (!strcmp("gripper_homing", cmd)) {
        if (nlhs != 1 || nrhs != 2)
            mexErrMsgTxt("Gripper Homing: One output and one input (handle) expected.");
        try {
            bool success = franka_robot_instance->gripperHoming();
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to perform gripper homing");
        }
        return;
    }

    // Gripper Grasp
    if (!strcmp("gripper_grasp", cmd)) {
        if (nlhs != 1 || nrhs != 7)
            mexErrMsgTxt("Gripper Grasp: One output and six inputs (handle, width, speed, force, epsilon_inner, epsilon_outer) expected.");
        try {
            double width = mxGetScalar(prhs[2]);
            double speed = mxGetScalar(prhs[3]);
            double force = mxGetScalar(prhs[4]);
            double epsilon_inner = mxGetScalar(prhs[5]);
            double epsilon_outer = mxGetScalar(prhs[6]);
            
            bool success = franka_robot_instance->gripperGrasp(
                width, speed, force, epsilon_inner, epsilon_outer);
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to execute gripper grasp");
        }
        return;
    }

    // Gripper Move
    if (!strcmp("gripper_move", cmd)) {
        if (nlhs != 1 || nrhs != 4)
            mexErrMsgTxt("Gripper Move: One output and three inputs (handle, width, speed) expected.");
        try {
            double width = mxGetScalar(prhs[2]);
            double speed = mxGetScalar(prhs[3]);
            
            bool success = franka_robot_instance->gripperMove(width, speed);
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to execute gripper move");
        }
        return;
    }

    // Gripper Stop
    if (!strcmp("gripper_stop", cmd)) {
        if (nlhs != 1 || nrhs != 2)
            mexErrMsgTxt("Gripper Stop: One output and one input (handle) expected.");
        try {
            bool success = franka_robot_instance->gripperStop();
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to stop gripper");
        }
        return;
    }

    // Gripper Move Async
    if (!strcmp("gripper_move_async", cmd)) {
        if (nlhs != 1 || nrhs < 4 || nrhs > 5)
            mexErrMsgTxt("Gripper Move Async: One output and 3-4 inputs (handle, width, speed, [timeout]) expected.");
        try {
            double width = mxGetScalar(prhs[2]);
            double speed = mxGetScalar(prhs[3]);
            double timeout = (nrhs >= 5) ? mxGetScalar(prhs[4]) : 15.0;
            
            bool started = franka_robot_instance->gripperMoveAsync(width, speed, timeout);
            plhs[0] = mxCreateLogicalScalar(started);
        } catch (const kj::Exception& e) {
            mexErrMsgTxt(("Gripper move async failed: " + std::string(e.getDescription().cStr())).c_str());
        } catch (const std::exception& e) {
            mexErrMsgTxt(("Gripper move async failed: " + std::string(e.what())).c_str());
        } catch (...) {
            mexErrMsgTxt("Failed to start async gripper move (unknown error)");
        }
        return;
    }

    // Gripper Grasp Async
    if (!strcmp("gripper_grasp_async", cmd)) {
        if (nlhs != 1 || nrhs < 7 || nrhs > 8)
            mexErrMsgTxt("Gripper Grasp Async: One output and 6-7 inputs (handle, width, speed, force, epsilon_inner, epsilon_outer, [timeout]) expected.");
        try {
            double width = mxGetScalar(prhs[2]);
            double speed = mxGetScalar(prhs[3]);
            double force = mxGetScalar(prhs[4]);
            double epsilon_inner = mxGetScalar(prhs[5]);
            double epsilon_outer = mxGetScalar(prhs[6]);
            double timeout = (nrhs >= 8) ? mxGetScalar(prhs[7]) : 15.0;
            
            bool started = franka_robot_instance->gripperGraspAsync(
                width, speed, force, epsilon_inner, epsilon_outer, timeout);
            plhs[0] = mxCreateLogicalScalar(started);
        } catch (const kj::Exception& e) {
            mexErrMsgTxt(("Gripper grasp async failed: " + std::string(e.getDescription().cStr())).c_str());
        } catch (const std::exception& e) {
            mexErrMsgTxt(("Gripper grasp async failed: " + std::string(e.what())).c_str());
        } catch (...) {
            mexErrMsgTxt("Failed to start async gripper grasp (unknown error)");
        }
        return;
    }

    // Gripper Async Status
    if (!strcmp("gripper_async_status", cmd)) {
        if (nlhs != 1 || nrhs != 2)
            mexErrMsgTxt("Gripper Async Status: One output and one input (handle) expected.");
        try {
            auto status = franka_robot_instance->getGripperAsyncStatus();
            plhs[0] = mxCreateStructMatrix(1, 1, 0, nullptr);
            
            // State fields
            mxAddField(plhs[0], "width");
            mxSetField(plhs[0], 0, "width", mxCreateDoubleScalar(status.getState().getWidth()));
            
            mxAddField(plhs[0], "max_width");
            mxSetField(plhs[0], 0, "max_width", mxCreateDoubleScalar(status.getState().getMaxWidth()));
            
            mxAddField(plhs[0], "is_grasped");
            mxSetField(plhs[0], 0, "is_grasped", mxCreateLogicalScalar(status.getState().getIsGrasped()));
            
            mxAddField(plhs[0], "temperature");
            mxSetField(plhs[0], 0, "temperature", mxCreateDoubleScalar(status.getState().getTemperature()));
            
            mxAddField(plhs[0], "time_stamp");
            mxSetField(plhs[0], 0, "time_stamp", mxCreateDoubleScalar(status.getState().getTimeStamp()));
            
            // Command status as string
            mxAddField(plhs[0], "command_status");
            const char* status_str = "unknown";
            switch (status.getCommandStatus()) {
                case GripperCommandStatus::IDLE: status_str = "idle"; break;
                case GripperCommandStatus::BUSY: status_str = "busy"; break;
                case GripperCommandStatus::SUCCESS: status_str = "success"; break;
                case GripperCommandStatus::FAILED: status_str = "failed"; break;
                case GripperCommandStatus::TIMEOUT: status_str = "timeout"; break;
                case GripperCommandStatus::STOPPED: status_str = "stopped"; break;
            }
            mxSetField(plhs[0], 0, "command_status", mxCreateString(status_str));
            
            mxAddField(plhs[0], "last_command");
            mxSetField(plhs[0], 0, "last_command", mxCreateString(status.getLastCommand().cStr()));
            
            mxAddField(plhs[0], "error_message");
            mxSetField(plhs[0], 0, "error_message", mxCreateString(status.getErrorMessage().cStr()));
            
        } catch (const kj::Exception& e) {
            mexErrMsgTxt(("Gripper async status failed: " + std::string(e.getDescription().cStr())).c_str());
        } catch (const std::exception& e) {
            mexErrMsgTxt(("Gripper async status failed: " + std::string(e.what())).c_str());
        } catch (...) {
            mexErrMsgTxt("Failed to get gripper async status (unknown error)");
        }
        return;
    }

    // Gripper Wait For Command
    if (!strcmp("gripper_wait", cmd)) {
        if (nlhs != 1 || nrhs < 2 || nrhs > 3)
            mexErrMsgTxt("Gripper Wait: One output and 1-2 inputs (handle, [timeout]) expected.");
        try {
            double timeout = (nrhs >= 3) ? mxGetScalar(prhs[2]) : 30.0;
            
            auto status = franka_robot_instance->gripperWaitForCommand(timeout);
            plhs[0] = mxCreateStructMatrix(1, 1, 0, nullptr);
            
            // State fields
            mxAddField(plhs[0], "width");
            mxSetField(plhs[0], 0, "width", mxCreateDoubleScalar(status.getState().getWidth()));
            
            mxAddField(plhs[0], "max_width");
            mxSetField(plhs[0], 0, "max_width", mxCreateDoubleScalar(status.getState().getMaxWidth()));
            
            mxAddField(plhs[0], "is_grasped");
            mxSetField(plhs[0], 0, "is_grasped", mxCreateLogicalScalar(status.getState().getIsGrasped()));
            
            mxAddField(plhs[0], "temperature");
            mxSetField(plhs[0], 0, "temperature", mxCreateDoubleScalar(status.getState().getTemperature()));
            
            mxAddField(plhs[0], "time_stamp");
            mxSetField(plhs[0], 0, "time_stamp", mxCreateDoubleScalar(status.getState().getTimeStamp()));
            
            // Command status as string
            mxAddField(plhs[0], "command_status");
            const char* status_str = "unknown";
            switch (status.getCommandStatus()) {
                case GripperCommandStatus::IDLE: status_str = "idle"; break;
                case GripperCommandStatus::BUSY: status_str = "busy"; break;
                case GripperCommandStatus::SUCCESS: status_str = "success"; break;
                case GripperCommandStatus::FAILED: status_str = "failed"; break;
                case GripperCommandStatus::TIMEOUT: status_str = "timeout"; break;
                case GripperCommandStatus::STOPPED: status_str = "stopped"; break;
            }
            mxSetField(plhs[0], 0, "command_status", mxCreateString(status_str));
            
            mxAddField(plhs[0], "last_command");
            mxSetField(plhs[0], 0, "last_command", mxCreateString(status.getLastCommand().cStr()));
            
            mxAddField(plhs[0], "error_message");
            mxSetField(plhs[0], 0, "error_message", mxCreateString(status.getErrorMessage().cStr()));
            
        } catch (const kj::Exception& e) {
            mexErrMsgTxt(("Gripper wait failed: " + std::string(e.getDescription().cStr())).c_str());
        } catch (const std::exception& e) {
            mexErrMsgTxt(("Gripper wait failed: " + std::string(e.what())).c_str());
        } catch (...) {
            mexErrMsgTxt("Failed to wait for gripper command (unknown error)");
        }
        return;
    }

    // Set Collision Behavior
    if (!strcmp("set_collision_behavior", cmd)) {
        if (nlhs != 1 || nrhs != 10)
            mexErrMsgTxt("Set Collision Behavior: One output and nine inputs expected (handle + 8 threshold arrays).");
        
        // Validate torque threshold arrays (7 elements each)
        for (int i = 2; i <= 5; i++) {
            if (!mxIsDouble(prhs[i]) || mxGetNumberOfElements(prhs[i]) != 7)
                mexErrMsgTxt("Torque threshold arrays must be 7-element double arrays.");
        }
        
        // Validate force threshold arrays (6 elements each)
        for (int i = 6; i <= 9; i++) {
            if (!mxIsDouble(prhs[i]) || mxGetNumberOfElements(prhs[i]) != 6)
                mexErrMsgTxt("Force threshold arrays must be 6-element double arrays.");
        }

        try {
            // Convert torque thresholds
            std::array<double, 7> lower_torque_acc{};
            std::array<double, 7> upper_torque_acc{};
            std::array<double, 7> lower_torque_nom{};
            std::array<double, 7> upper_torque_nom{};
            
            // Convert force thresholds
            std::array<double, 6> lower_force_acc{};
            std::array<double, 6> upper_force_acc{};
            std::array<double, 6> lower_force_nom{};
            std::array<double, 6> upper_force_nom{};

            // Get pointers to input arrays
            double* lower_torque_acc_ptr = mxGetPr(prhs[2]);
            double* upper_torque_acc_ptr = mxGetPr(prhs[3]);
            double* lower_torque_nom_ptr = mxGetPr(prhs[4]);
            double* upper_torque_nom_ptr = mxGetPr(prhs[5]);
            double* lower_force_acc_ptr = mxGetPr(prhs[6]);
            double* upper_force_acc_ptr = mxGetPr(prhs[7]);
            double* lower_force_nom_ptr = mxGetPr(prhs[8]);
            double* upper_force_nom_ptr = mxGetPr(prhs[9]);

            // Copy torque thresholds
            for (size_t i = 0; i < 7; i++) {
                lower_torque_acc[i] = lower_torque_acc_ptr[i];
                upper_torque_acc[i] = upper_torque_acc_ptr[i];
                lower_torque_nom[i] = lower_torque_nom_ptr[i];
                upper_torque_nom[i] = upper_torque_nom_ptr[i];
            }

            // Copy force thresholds
            for (size_t i = 0; i < 6; i++) {
                lower_force_acc[i] = lower_force_acc_ptr[i];
                upper_force_acc[i] = upper_force_acc_ptr[i];
                lower_force_nom[i] = lower_force_nom_ptr[i];
                upper_force_nom[i] = upper_force_nom_ptr[i];
            }

            bool success = franka_robot_instance->setCollisionBehavior(
                lower_torque_acc,
                upper_torque_acc,
                lower_torque_nom,
                upper_torque_nom,
                lower_force_acc,
                upper_force_acc,
                lower_force_nom,
                upper_force_nom
            );
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to set collision behavior");
        }
        return;
    }

    // Set Load Inertia
    if (!strcmp("set_load", cmd)) {
        if (nlhs != 1 || nrhs != 5)
            mexErrMsgTxt("Set Load: One output and four inputs expected (handle, mass, center_of_mass, load_inertia).");
        
        // Validate mass is a scalar
        if (!mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1)
            mexErrMsgTxt("Mass must be a scalar double.");
        
        // Validate center of mass is a 3-element array
        if (!mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != 3)
            mexErrMsgTxt("Center of mass must be a 3-element double array.");
        
        // Validate load inertia is a 9-element array (3x3 matrix in row-major format)
        if (!mxIsDouble(prhs[4]) || mxGetNumberOfElements(prhs[4]) != 9)
            mexErrMsgTxt("Load inertia must be a 9-element double array (3x3 matrix in row-major format).");

        try {
            double mass = mxGetScalar(prhs[2]);
            
            std::array<double, 3> center_of_mass{};
            std::array<double, 9> load_inertia{};

            double* com_ptr = mxGetPr(prhs[3]);
            double* inertia_ptr = mxGetPr(prhs[4]);

            // Copy center of mass
            for (size_t i = 0; i < 3; i++) {
                center_of_mass[i] = com_ptr[i];
            }

            // Copy load inertia
            for (size_t i = 0; i < 9; i++) {
                load_inertia[i] = inertia_ptr[i];
            }

            bool success = franka_robot_instance->setLoadInertia(mass, center_of_mass, load_inertia);
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to set load inertia");
        }
        return;
    }

    // Get Vacuum Gripper State
    if (!strcmp("vacuum_gripper_state", cmd)) {
        if (nlhs != 1 || nrhs != 2)
            mexErrMsgTxt("Vacuum Gripper State: One output and one input (handle) expected.");
        try {
            auto state = franka_robot_instance->getVacuumGripperState();
            plhs[0] = mxCreateStructMatrix(1, 1, 0, nullptr);
            
            mxAddField(plhs[0], "in_control_range");
            mxSetField(plhs[0], 0, "in_control_range", mxCreateLogicalScalar(state.getInControlRange()));
            
            mxAddField(plhs[0], "part_detached");
            mxSetField(plhs[0], 0, "part_detached", mxCreateLogicalScalar(state.getPartDetached()));
            
            mxAddField(plhs[0], "part_present");
            mxSetField(plhs[0], 0, "part_present", mxCreateLogicalScalar(state.getPartPresent()));
            
            mxAddField(plhs[0], "device_status");
            mxSetField(plhs[0], 0, "device_status", mxCreateDoubleScalar(state.getDeviceStatus()));
            
            mxAddField(plhs[0], "actual_power");
            mxSetField(plhs[0], 0, "actual_power", mxCreateDoubleScalar(state.getActualPower()));
            
            mxAddField(plhs[0], "vacuum");
            mxSetField(plhs[0], 0, "vacuum", mxCreateDoubleScalar(state.getVacuum()));
            
            mxAddField(plhs[0], "time");
            mxSetField(plhs[0], 0, "time", mxCreateDoubleScalar(state.getTime()));
        } catch (...) {
            mexErrMsgTxt("Failed to get vacuum gripper state");
        }
        return;
    }

    // Vacuum Gripper Vacuum
    if (!strcmp("vacuum_gripper_vacuum", cmd)) {
        if (nlhs != 1 || nrhs != 5)
            mexErrMsgTxt("Vacuum Gripper Vacuum: One output and four inputs (handle, control_point, timeout, profile) expected.");
        try {
            uint8_t control_point = static_cast<uint8_t>(mxGetScalar(prhs[2]));
            uint32_t timeout = static_cast<uint32_t>(mxGetScalar(prhs[3]));
            uint8_t profile = static_cast<uint8_t>(mxGetScalar(prhs[4]));
            
            bool success = franka_robot_instance->vacuumGripperVacuum(control_point, timeout, profile);
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to execute vacuum gripper vacuum");
        }
        return;
    }

    // Vacuum Gripper Drop Off
    if (!strcmp("vacuum_gripper_drop_off", cmd)) {
        if (nlhs != 1 || nrhs != 3)
            mexErrMsgTxt("Vacuum Gripper Drop Off: One output and two inputs (handle, timeout) expected.");
        try {
            uint32_t timeout = static_cast<uint32_t>(mxGetScalar(prhs[2]));
            
            bool success = franka_robot_instance->vacuumGripperDropOff(timeout);
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to execute vacuum gripper drop off");
        }
        return;
    }

    // Vacuum Gripper Stop
    if (!strcmp("vacuum_gripper_stop", cmd)) {
        if (nlhs != 1 || nrhs != 2)
            mexErrMsgTxt("Vacuum Gripper Stop: One output and one input (handle) expected.");
        try {
            bool success = franka_robot_instance->vacuumGripperStop();
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to stop vacuum gripper");
        }
        return;
    }

    // Vacuum Gripper Vacuum Async
    if (!strcmp("vacuum_gripper_vacuum_async", cmd)) {
        if (nlhs != 1 || nrhs < 5 || nrhs > 6)
            mexErrMsgTxt("Vacuum Gripper Vacuum Async: One output and 4-5 inputs (handle, control_point, timeout, profile, [command_timeout]) expected.");
        try {
            uint8_t control_point = static_cast<uint8_t>(mxGetScalar(prhs[2]));
            uint32_t timeout = static_cast<uint32_t>(mxGetScalar(prhs[3]));
            uint8_t profile = static_cast<uint8_t>(mxGetScalar(prhs[4]));
            double command_timeout = (nrhs >= 6) ? mxGetScalar(prhs[5]) : 15.0;
            
            bool started = franka_robot_instance->vacuumGripperVacuumAsync(
                control_point, timeout, profile, command_timeout);
            plhs[0] = mxCreateLogicalScalar(started);
        } catch (const kj::Exception& e) {
            mexErrMsgTxt(("Vacuum gripper vacuum async failed: " + std::string(e.getDescription().cStr())).c_str());
        } catch (const std::exception& e) {
            mexErrMsgTxt(("Vacuum gripper vacuum async failed: " + std::string(e.what())).c_str());
        } catch (...) {
            mexErrMsgTxt("Failed to start async vacuum gripper vacuum (unknown error)");
        }
        return;
    }

    // Vacuum Gripper Drop Off Async
    if (!strcmp("vacuum_gripper_drop_off_async", cmd)) {
        if (nlhs != 1 || nrhs < 3 || nrhs > 4)
            mexErrMsgTxt("Vacuum Gripper Drop Off Async: One output and 2-3 inputs (handle, timeout, [command_timeout]) expected.");
        try {
            uint32_t timeout = static_cast<uint32_t>(mxGetScalar(prhs[2]));
            double command_timeout = (nrhs >= 4) ? mxGetScalar(prhs[3]) : 15.0;
            
            bool started = franka_robot_instance->vacuumGripperDropOffAsync(timeout, command_timeout);
            plhs[0] = mxCreateLogicalScalar(started);
        } catch (const kj::Exception& e) {
            mexErrMsgTxt(("Vacuum gripper drop off async failed: " + std::string(e.getDescription().cStr())).c_str());
        } catch (const std::exception& e) {
            mexErrMsgTxt(("Vacuum gripper drop off async failed: " + std::string(e.what())).c_str());
        } catch (...) {
            mexErrMsgTxt("Failed to start async vacuum gripper drop off (unknown error)");
        }
        return;
    }

    // Vacuum Gripper Async Status
    if (!strcmp("vacuum_gripper_async_status", cmd)) {
        if (nlhs != 1 || nrhs != 2)
            mexErrMsgTxt("Vacuum Gripper Async Status: One output and one input (handle) expected.");
        try {
            auto status = franka_robot_instance->getVacuumGripperAsyncStatus();
            plhs[0] = mxCreateStructMatrix(1, 1, 0, nullptr);
            
            // State fields
            mxAddField(plhs[0], "in_control_range");
            mxSetField(plhs[0], 0, "in_control_range", mxCreateLogicalScalar(status.getState().getInControlRange()));
            
            mxAddField(plhs[0], "part_detached");
            mxSetField(plhs[0], 0, "part_detached", mxCreateLogicalScalar(status.getState().getPartDetached()));
            
            mxAddField(plhs[0], "part_present");
            mxSetField(plhs[0], 0, "part_present", mxCreateLogicalScalar(status.getState().getPartPresent()));
            
            mxAddField(plhs[0], "device_status");
            mxSetField(plhs[0], 0, "device_status", mxCreateDoubleScalar(status.getState().getDeviceStatus()));
            
            mxAddField(plhs[0], "actual_power");
            mxSetField(plhs[0], 0, "actual_power", mxCreateDoubleScalar(status.getState().getActualPower()));
            
            mxAddField(plhs[0], "vacuum");
            mxSetField(plhs[0], 0, "vacuum", mxCreateDoubleScalar(status.getState().getVacuum()));
            
            mxAddField(plhs[0], "time");
            mxSetField(plhs[0], 0, "time", mxCreateDoubleScalar(status.getState().getTime()));
            
            // Command status as string
            mxAddField(plhs[0], "command_status");
            const char* status_str = "unknown";
            switch (status.getCommandStatus()) {
                case VacuumGripperCommandStatus::IDLE: status_str = "idle"; break;
                case VacuumGripperCommandStatus::BUSY: status_str = "busy"; break;
                case VacuumGripperCommandStatus::SUCCESS: status_str = "success"; break;
                case VacuumGripperCommandStatus::FAILED: status_str = "failed"; break;
                case VacuumGripperCommandStatus::TIMEOUT: status_str = "timeout"; break;
                case VacuumGripperCommandStatus::STOPPED: status_str = "stopped"; break;
            }
            mxSetField(plhs[0], 0, "command_status", mxCreateString(status_str));
            
            mxAddField(plhs[0], "last_command");
            mxSetField(plhs[0], 0, "last_command", mxCreateString(status.getLastCommand().cStr()));
            
            mxAddField(plhs[0], "error_message");
            mxSetField(plhs[0], 0, "error_message", mxCreateString(status.getErrorMessage().cStr()));
            
        } catch (const kj::Exception& e) {
            mexErrMsgTxt(("Vacuum gripper async status failed: " + std::string(e.getDescription().cStr())).c_str());
        } catch (const std::exception& e) {
            mexErrMsgTxt(("Vacuum gripper async status failed: " + std::string(e.what())).c_str());
        } catch (...) {
            mexErrMsgTxt("Failed to get vacuum gripper async status (unknown error)");
        }
        return;
    }

    // Vacuum Gripper Wait For Command
    if (!strcmp("vacuum_gripper_wait", cmd)) {
        if (nlhs != 1 || nrhs < 2 || nrhs > 3)
            mexErrMsgTxt("Vacuum Gripper Wait: One output and 1-2 inputs (handle, [timeout]) expected.");
        try {
            double timeout = (nrhs >= 3) ? mxGetScalar(prhs[2]) : 30.0;
            
            auto status = franka_robot_instance->vacuumGripperWaitForCommand(timeout);
            plhs[0] = mxCreateStructMatrix(1, 1, 0, nullptr);
            
            // State fields
            mxAddField(plhs[0], "in_control_range");
            mxSetField(plhs[0], 0, "in_control_range", mxCreateLogicalScalar(status.getState().getInControlRange()));
            
            mxAddField(plhs[0], "part_detached");
            mxSetField(plhs[0], 0, "part_detached", mxCreateLogicalScalar(status.getState().getPartDetached()));
            
            mxAddField(plhs[0], "part_present");
            mxSetField(plhs[0], 0, "part_present", mxCreateLogicalScalar(status.getState().getPartPresent()));
            
            mxAddField(plhs[0], "device_status");
            mxSetField(plhs[0], 0, "device_status", mxCreateDoubleScalar(status.getState().getDeviceStatus()));
            
            mxAddField(plhs[0], "actual_power");
            mxSetField(plhs[0], 0, "actual_power", mxCreateDoubleScalar(status.getState().getActualPower()));
            
            mxAddField(plhs[0], "vacuum");
            mxSetField(plhs[0], 0, "vacuum", mxCreateDoubleScalar(status.getState().getVacuum()));
            
            mxAddField(plhs[0], "time");
            mxSetField(plhs[0], 0, "time", mxCreateDoubleScalar(status.getState().getTime()));
            
            // Command status as string
            mxAddField(plhs[0], "command_status");
            const char* status_str = "unknown";
            switch (status.getCommandStatus()) {
                case VacuumGripperCommandStatus::IDLE: status_str = "idle"; break;
                case VacuumGripperCommandStatus::BUSY: status_str = "busy"; break;
                case VacuumGripperCommandStatus::SUCCESS: status_str = "success"; break;
                case VacuumGripperCommandStatus::FAILED: status_str = "failed"; break;
                case VacuumGripperCommandStatus::TIMEOUT: status_str = "timeout"; break;
                case VacuumGripperCommandStatus::STOPPED: status_str = "stopped"; break;
            }
            mxSetField(plhs[0], 0, "command_status", mxCreateString(status_str));
            
            mxAddField(plhs[0], "last_command");
            mxSetField(plhs[0], 0, "last_command", mxCreateString(status.getLastCommand().cStr()));
            
            mxAddField(plhs[0], "error_message");
            mxSetField(plhs[0], 0, "error_message", mxCreateString(status.getErrorMessage().cStr()));
            
        } catch (const kj::Exception& e) {
            mexErrMsgTxt(("Vacuum gripper wait failed: " + std::string(e.getDescription().cStr())).c_str());
        } catch (const std::exception& e) {
            mexErrMsgTxt(("Vacuum gripper wait failed: " + std::string(e.what())).c_str());
        } catch (...) {
            mexErrMsgTxt("Failed to wait for vacuum gripper command (unknown error)");
        }
        return;
    }

    // Set Joint Impedance
    if (!strcmp("set_joint_impedance", cmd)) {
        if (nlhs != 1 || nrhs != 3)
            mexErrMsgTxt("Set Joint Impedance: One output and two inputs (handle, K_theta) expected.");
        
        if (!mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 7)
            mexErrMsgTxt("Joint impedance must be a 7-element double array.");

        try {
            std::array<double, 7> K_theta{};
            double* k_ptr = mxGetPr(prhs[2]);
            for (size_t i = 0; i < 7; i++) {
                K_theta[i] = k_ptr[i];
            }
            
            bool success = franka_robot_instance->setJointImpedance(K_theta);
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to set joint impedance");
        }
        return;
    }

    // Set Cartesian Impedance
    if (!strcmp("set_cartesian_impedance", cmd)) {
        if (nlhs != 1 || nrhs != 3)
            mexErrMsgTxt("Set Cartesian Impedance: One output and two inputs (handle, K_x) expected.");
        
        if (!mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 6)
            mexErrMsgTxt("Cartesian impedance must be a 6-element double array.");

        try {
            std::array<double, 6> K_x{};
            double* k_ptr = mxGetPr(prhs[2]);
            for (size_t i = 0; i < 6; i++) {
                K_x[i] = k_ptr[i];
            }
            
            bool success = franka_robot_instance->setCartesianImpedance(K_x);
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to set Cartesian impedance");
        }
        return;
    }

    // Set Guiding Mode
    if (!strcmp("set_guiding_mode", cmd)) {
        if (nlhs != 1 || nrhs != 4)
            mexErrMsgTxt("Set Guiding Mode: One output and three inputs (handle, guiding_mode, elbow) expected.");
        
        if (!mxIsLogical(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 6)
            mexErrMsgTxt("Guiding mode must be a 6-element logical array.");
        
        if (!mxIsLogicalScalar(prhs[3]))
            mexErrMsgTxt("Elbow must be a logical scalar.");

        try {
            std::array<bool, 6> guiding_mode{};
            mxLogical* mode_ptr = mxGetLogicals(prhs[2]);
            for (size_t i = 0; i < 6; i++) {
                guiding_mode[i] = mode_ptr[i];
            }
            bool elbow = mxIsLogicalScalarTrue(prhs[3]);
            
            bool success = franka_robot_instance->setGuidingMode(guiding_mode, elbow);
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to set guiding mode");
        }
        return;
    }

    // Set K (EE_T_K transformation)
    if (!strcmp("set_k", cmd)) {
        if (nlhs != 1 || nrhs != 3)
            mexErrMsgTxt("Set K: One output and two inputs (handle, EE_T_K) expected.");
        
        if (!mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 16)
            mexErrMsgTxt("EE_T_K must be a 16-element double array (4x4 matrix in column-major format).");

        try {
            std::array<double, 16> EE_T_K{};
            double* k_ptr = mxGetPr(prhs[2]);
            for (size_t i = 0; i < 16; i++) {
                EE_T_K[i] = k_ptr[i];
            }
            
            bool success = franka_robot_instance->setK(EE_T_K);
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to set EE_T_K transformation");
        }
        return;
    }

    // Set EE (NE_T_EE transformation)
    if (!strcmp("set_ee", cmd)) {
        if (nlhs != 1 || nrhs != 3)
            mexErrMsgTxt("Set EE: One output and two inputs (handle, NE_T_EE) expected.");
        
        if (!mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 16)
            mexErrMsgTxt("NE_T_EE must be a 16-element double array (4x4 matrix in column-major format).");

        try {
            std::array<double, 16> NE_T_EE{};
            double* ee_ptr = mxGetPr(prhs[2]);
            for (size_t i = 0; i < 16; i++) {
                NE_T_EE[i] = ee_ptr[i];
            }
            
            bool success = franka_robot_instance->setEE(NE_T_EE);
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to set NE_T_EE transformation");
        }
        return;
    }

    // Stop Robot
    if (!strcmp("stop_robot", cmd)) {
        if (nlhs != 1 || nrhs != 2)
            mexErrMsgTxt("Stop Robot: One output and one input (handle) expected.");
        try {
            bool success = franka_robot_instance->stopRobot();
            plhs[0] = mxCreateLogicalScalar(success);
        } catch (...) {
            mexErrMsgTxt("Failed to stop robot");
        }
        return;
    }

    // Ping (health check)
    if (!strcmp("ping", cmd)) {
        if (nlhs != 1 || nrhs != 2)
            mexErrMsgTxt("Ping: One output and one input (handle) expected.");
        try {
            auto [timestamp, port] = franka_robot_instance->ping();
            
            // Create struct with timestamp and port
            const char* fieldnames[] = {"timestamp", "port"};
            plhs[0] = mxCreateStructMatrix(1, 1, 2, fieldnames);
            mxSetField(plhs[0], 0, "timestamp", mxCreateDoubleScalar(static_cast<double>(timestamp)));
            mxSetField(plhs[0], 0, "port", mxCreateDoubleScalar(static_cast<double>(port)));
        } catch (...) {
            mexErrMsgTxt("Failed to ping server");
        }
        return;
    }

    // Got here, so command not recognized
    mexErrMsgTxt("Command not recognized.");
}
