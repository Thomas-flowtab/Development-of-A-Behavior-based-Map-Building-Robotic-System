classdef Wheel
    % Wheel Class representing a single wheel in the robot simulation.
    %
    % This class manages the wheel's properties such as its handle, velocity, and radius.
    % It communicates with CoppeliaSim to set the wheel's target velocity for movement control.
    
    properties
        connection  % Connection object to handle communication with CoppeliaSim
        handle      % Handle for the wheel object in CoppeliaSim
        velocity    % Current velocity of the wheel
        radius      % Radius of the wheel (in meters)
    end
    
    methods
        % Constructor to initialize the wheel with a handle and radius
        function obj = Wheel(connection, handle, radius)
            % Initializes the Wheel object with a connection to CoppeliaSim,
            % a specific wheel handle, and the wheel's radius.
            %
            % Parameters:
            %   connection - Object containing the connection to CoppeliaSim.
            %   handle     - Handle representing the wheel in the simulation.
            %   radius     - The radius of the wheel (meters).
            
            obj.handle = handle;     % Set the handle for the wheel
            obj.velocity = 0;        % Set initial velocity to 0 (stationary)
            obj.radius = radius;     % Set the wheel radius
            obj.connection = connection;  % Store the CoppeliaSim connection object
        end
        
        % Method to set the wheel's velocity
        function setVelocity(obj, velocity)
            % Updates the wheel's velocity and sends the command to CoppeliaSim.
            %
            % Parameters:
            %   velocity - The desired velocity for the wheel (radians/second).
            
            obj.velocity = velocity;  % Update the velocity property
            
            % Send the target velocity command to CoppeliaSim to update the wheel's velocity
            obj.connection.sim.simxSetJointTargetVelocity(obj.connection.clientID, ...
                obj.handle, obj.velocity, obj.connection.sim.simx_opmode_streaming);
        end
    end
end
