classdef FrankaRobotModes < Simulink.IntEnumType
    %FRANKAROBOTMODES Robot mode enumeration for Franka RobotState.robot_mode.
    %
    % Use this in Simulink (e.g., Data Type Conversion block) to cast the
    % int32 'robot_mode' signal to a readable enumerated type.
    %
    % Values match libfranka franka::RobotMode:
    % 0=Other, 1=Idle, 2=Move, 3=Guiding, 4=Reflex, 5=UserStopped, 6=AutomaticErrorRecovery

    enumeration
        Other(0)
        Idle(1)
        Move(2)
        Guiding(3)
        Reflex(4)
        UserStopped(5)
        AutomaticErrorRecovery(6)
    end

    methods (Static)
        function retVal = getDefaultValue()
            retVal = FrankaRobotModes.Other;
        end

        function retVal = getStorageType()
            retVal = 'int32';
        end
    end
end
