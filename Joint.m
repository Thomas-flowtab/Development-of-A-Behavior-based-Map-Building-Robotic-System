classdef Joint < handle
   
    
    properties (Constant)
        % Define joint types within the class
        structJointTypes = struct( ...
            'REVOLUTE', 1, ...
            'PRISMATIC', 2 ...
            );
        MAX_REVOLUTE_SPEED = 360.0;   % Max revolute velocity in degrees/sec
        MAX_PRISMATIC_SPEED = 0.753984; % Max prismatic velocity in meters/sec
    end
    
    properties
        handle      % Joint handle retrieved from CoppeliaSim
        cmdPos      % Commanded position for the joint
        currentPos  % Current position of the joint
        velocity    % Velocity for the joint
        maxSpeed    % Maximum speed for the joint
        type        % Joint type: revolute or prismatic
        lowerLimit  % Lower limit of joint movement
        upperLimit  % Upper limit of joint movement
        atPos = true;  % Indicates if the joint is at the target position
        tolerance = 1; % Position tolerance
    end
    
    methods
        % Constructor
        function obj = Joint(handle, jointType, lowerLimit, upperLimit, sim, clientID)
            obj.handle = handle;
            obj.type = jointType;
            obj.lowerLimit = lowerLimit;
            obj.upperLimit = upperLimit;

            if obj.type == obj.structJointTypes.REVOLUTE
                obj.maxSpeed = obj.MAX_REVOLUTE_SPEED;  % Set max speed for revolute joints
            else
                obj.maxSpeed = obj.MAX_PRISMATIC_SPEED; % Set max speed for prismatic joints
            end

            % Retrieve and set the current position during initialization
            %we need this to init the controls at the same position as
            %CoppSim
            obj.getCurrentPosition(sim, clientID);
        end
        
        % Set the position for the joint
        function setPosition(obj, sim, clientID, position)
            % Ensure position is within limits
            if position < obj.lowerLimit
                position = obj.lowerLimit;
            elseif position > obj.upperLimit
                position = obj.upperLimit;
            end

            % Update commanded position
            obj.cmdPos = position;

            % Convert degrees to radians for revolute joints
            if obj.type == obj.structJointTypes.REVOLUTE  % Revolute
                position = deg2rad(position);
            end

            % Send the new position to CoppeliaSim
            sim.simxSetJointTargetPosition(clientID, obj.handle, position, sim.simx_opmode_streaming);
            obj.atPos = false;
        end
      
        % Method to set velocity
        function setVelocity(obj, sim, clientID, speedValue)
            % Scale the speed based argument given
            scaledSpeed = (speedValue * obj.maxSpeed) / 100;

            if obj.type == obj.structJointTypes.REVOLUTE
                % Convert revolute joint speed from degrees/sec to radians/sec
                scaledSpeed = deg2rad(scaledSpeed);
            end

            sim.simxSetObjectFloatParameter(clientID, obj.handle, ...
                           sim.sim_jointfloatparam_upper_limit, ...
                           scaledSpeed, ...
                           sim.simx_opmode_blocking);
        end
        
        % Get the current position from CoppeliaSim
        function getCurrentPosition(obj, sim, clientID)
            [returnCode, position] = sim.simxGetJointPosition(clientID, obj.handle, sim.simx_opmode_blocking);
            if returnCode == sim.simx_return_ok
                if obj.type == obj.structJointTypes.REVOLUTE  % Revolute joint
                    obj.currentPos = double(rad2deg(position)); % Convert radians to degrees
                else
                    obj.currentPos = double(position); % Prismatic joint in meters
                end
                % Check if the joint has reached its commanded position
                obj.isAtPosition();
            else
                disp('Failed to get current position for joint');
            end
        end
        
        % Check if the joint is at the commanded position
        function isAtPosition(obj)
            % If the current position is within tolerance of the commanded position
            if abs(obj.currentPos - obj.cmdPos) <= obj.tolerance
                obj.atPos = true;  % Mark the joint as being at position
            else
                obj.atPos = false;  % Still in motion
            end
        end
    end
end
