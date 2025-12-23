classdef FrankaConnectionStatus < Simulink.IntEnumType
    %FRANKACONNECTIONSTATUS Robot connection lifecycle status enumeration.
    %
    % Use this in Simulink (e.g., Data Type Conversion block) to cast the
    % int32 'connection_status' signal to a readable enumerated type.
    %
    % Values:
    %   0=Disconnected, 1=Connecting, 2=Connected, 3=ControlRunning, 4=Error

    enumeration
        FRANKA_CONNECTION_DISCONNECTED(0)
        FRANKA_CONNECTION_CONNECTING(1)
        FRANKA_CONNECTION_CONNECTED(2)
        FRANKA_CONNECTION_CONTROL_RUNNING(3)
        FRANKA_CONNECTION_ERROR(4)
    end

    methods (Static)
        function retVal = getDefaultValue()
            retVal = FrankaConnectionStatus.FRANKA_CONNECTION_DISCONNECTED;
        end

        function retVal = getDataScope()
            retVal = 'Imported';
        end

        function retVal = getHeaderFile()
            retVal = 'franka_simulink_types.h';
        end
    end
end
