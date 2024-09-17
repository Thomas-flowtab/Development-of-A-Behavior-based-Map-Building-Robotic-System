classdef Wheel
    properties
        handle      % Wheel handle from CoppeliaSim
        velocity    % Velocity of the wheel
    end
    
    methods
        % Constructor to initialize the wheel with a handle
        function obj = Wheel(handle)
            obj.handle = handle;
            obj.velocity = 0;  % Initial velocity is 0
        end
        
        % Method to set the wheel's velocity
        function setVelocity(obj, sim, clientID, velocity)
            obj.velocity = velocity;  % Set the velocity
            % Send the velocity command to CoppeliaSim
            sim.simxSetJointTargetVelocity(clientID, obj.handle, obj.velocity, sim.simx_opmode_streaming);
        end
    end
end
