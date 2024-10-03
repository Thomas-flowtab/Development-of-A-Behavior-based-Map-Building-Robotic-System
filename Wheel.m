classdef Wheel
    properties
        connection  %connection Object Handle
        handle      % Wheel handle from CoppeliaSim
        velocity    % Velocity of the wheel
    end
    
    methods
        % Constructor to initialize the wheel with a handle and radius
        function obj = Wheel(connection,handle)
            obj.connection = connection;  % Store the connection object
            obj.handle = handle;          % Store the wheel handle from CoppeliaSim
            obj.velocity = 0;             % Initial velocity set to 0
        end
        
        % Method to set the wheel's velocity
        function setVelocity(obj,velocity)
            obj.velocity = velocity;  % Set the velocity
            % Send the velocity command to CoppeliaSim
            obj.connection.sim.simxSetJointTargetVelocity(obj.connection.clientID, obj.handle, obj.velocity, obj.connection.sim.simx_opmode_streaming);
        end
    end
end
