classdef FrankaConnectionErrorCode < Simulink.IntEnumType
    %FRANKACONNECTIONERRORCODE Connection error code enumeration.
    %
    % Use this in Simulink (e.g., Data Type Conversion block) to cast the
    % int32 'last_connection_error_code' signal to a readable enumerated type.
    %
    % Maps libfranka exception types to codes:
    %   0=None, 1=Network, 2=Protocol, 3=IncompatibleVersion, 4=Control,
    %   5=Command, 6=Realtime, 7=InvalidOperation, 8=Model, 9=Unknown

    enumeration
        FRANKA_CONNECTION_ERROR_NONE(0)
        FRANKA_CONNECTION_ERROR_NETWORK(1)
        FRANKA_CONNECTION_ERROR_PROTOCOL(2)
        FRANKA_CONNECTION_ERROR_INCOMPATIBLE_VERSION(3)
        FRANKA_CONNECTION_ERROR_CONTROL(4)
        FRANKA_CONNECTION_ERROR_COMMAND(5)
        FRANKA_CONNECTION_ERROR_REALTIME(6)
        FRANKA_CONNECTION_ERROR_INVALID_OPERATION(7)
        FRANKA_CONNECTION_ERROR_MODEL(8)
        FRANKA_CONNECTION_ERROR_UNKNOWN(9)
    end

    methods (Static)
        function retVal = getDefaultValue()
            retVal = FrankaConnectionErrorCode.FRANKA_CONNECTION_ERROR_NONE;
        end

        function retVal = getDataScope()
            retVal = 'Imported';
        end

        function retVal = getHeaderFile()
            retVal = 'franka_simulink_types.h';
        end
    end
end
