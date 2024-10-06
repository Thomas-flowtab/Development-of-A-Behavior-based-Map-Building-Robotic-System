classdef BaseControl
    % BaseControl Class to handle robot base movements in a simulation environment.
    %
    % This class is responsible for controlling the movement of the robot's base, including
    % forward/backward motion, rotation, and stopping. It uses wheel velocities to control the
    % movement of the robot in different directions and adjusts the speed accordingly.
    % The class also includes functionality for detecting if the robot is stuck and reversing it.
    
    properties
        connection      % Coppeliasim connection object to communicate with the simulation
        baseHandle      % Handle for the robot base, used to identify the base in the simulation
        wheels          % Array of Wheel objects representing the robot's wheels
        linearSpeed     % Linear speed for forward and backward movement
        rotationSpeed   % Speed for rotating the robot (turning)
        WheelMovements  % Struct defining movement directions (FORWARD, BACKWARD, LEFT, etc.)
    end
    
    methods
        % Constructor to initialize the robot's wheels and basic movement parameters
        function obj = BaseControl(connection, basehandle, wheelHandles, wheelRadius)
            % Initializes the base control object with a connection to the simulation,
            % wheel handles, and a wheel radius.
            % 
            % Parameters:
            %   connection - Struct containing simulation connection details
            %   basehandle - Handle for the robot's base in the simulation
            %   wheelHandles - Array of handles representing the robot's wheels
            %   wheelRadius - Radius of the robot's wheels

            obj.baseHandle = basehandle;
            obj.wheels = cell(1, 2);  % Initialize the wheel array (for the front-left and front-right wheels)
            obj.connection = connection;
            for i = 1:length(obj.wheels)
                % Initialize each Wheel object using the provided connection, handle, and wheel radius
                obj.wheels{i} = Wheel(connection, wheelHandles(i), wheelRadius);  
            end
            obj.linearSpeed = 1;    % Set the default linear speed
            obj.rotationSpeed = 0.6; % Set the default rotation speed for turning
            % Define movement direction constants for easy reference
            obj.WheelMovements = struct(...
                'FORWARD', 1, 'BACKWARD', 2, 'LEFT', 3, 'RIGHT', 4, 'STOP', 5, 'CLOCKWISE', 6, 'COUNTER_CLOCKWISE', 7);
        end
        
        % Method to move the robot in a specified direction
        function move(obj, direction)
            % Moves the robot in the direction specified by the input.
            %
            % Parameters:
            %   direction - Movement direction based on the WheelMovements struct
            switch direction
                case obj.WheelMovements.FORWARD
                    obj.setWheelVelocities(obj.linearSpeed, obj.linearSpeed);  % Move both wheels forward
                case obj.WheelMovements.BACKWARD
                    obj.setWheelVelocities(-obj.linearSpeed, -obj.linearSpeed);  % Move both wheels backward
                case obj.WheelMovements.LEFT
                    obj.setWheelVelocities(-obj.rotationSpeed, obj.rotationSpeed);  % Rotate the robot to the left
                case obj.WheelMovements.RIGHT
                    obj.setWheelVelocities(obj.rotationSpeed, -obj.rotationSpeed);  % Rotate the robot to the right
                case obj.WheelMovements.STOP
                    obj.setWheelVelocities(0, 0);  % Stop all wheel movement
                case obj.WheelMovements.CLOCKWISE
                    obj.setWheelVelocities(obj.linearSpeed, 0);  % Turn clockwise (left wheel moves, right wheel stops)
                case obj.WheelMovements.COUNTER_CLOCKWISE
                    obj.setWheelVelocities(0, obj.linearSpeed);  % Turn counter-clockwise (right wheel moves, left wheel stops)
            end
        end

        % Method to stop all movement
        function stop(obj)
            % Stops all movement of the robot by setting both wheel velocities to zero.
            obj.setWheelVelocities(0, 0);
        end

        % Method to move the robot using linear (v) and angular (omega) velocities
        function moveRobot(obj, v, omega, minFrontDist)
            % Controls the robot's movement by setting linear (v) and angular (omega) velocities.
            % Includes stuck detection and a speed reduction factor for fine control.
            %
            % Parameters:
            %   v - Linear velocity for forward/backward movement
            %   omega - Angular velocity for turning
            %   minFrontDist - Minimum distance detected in front to avoid obstacles

            % Define the robot's physical parameters (wheel radius and distance between wheels)
            r = 0.0975;  % Radius of the wheels (in meters)
            L = 0.331;   % Distance between the wheels (wheelbase in meters)
        
            % Introduce a speed reduction factor (to slow down movement)
            speedReductionFactor = 0.4;  % Reduce speed by 40%
        
            % Apply the speed reduction factor to both linear and angular velocities
            v = v * speedReductionFactor;
            omega = omega * speedReductionFactor;
        
            % Stuck detection variables (use persistent to maintain state across calls)
            persistent stuckTime lastV  % Variables to store the last velocity and time spent stuck
            if isempty(stuckTime)
                stuckTime = 0;  % Initialize stuck time to zero
                lastV = v;  % Initialize the last velocity value
            end
        
            % Define parameters for stuck detection
            stuckThreshold = 0.1;  % Velocity threshold to consider the robot stuck
            stuckDuration = 2;  % Time threshold to determine if the robot is stuck
        
            % Check if the robot is stuck (low velocity for an extended period)
            if abs(v) < stuckThreshold && abs(lastV) > stuckThreshold
                stuckTime = stuckTime + 0.1;  % Increment stuck time if the robot is stuck
            else
                stuckTime = 0;  % Reset stuck time if the robot is moving
            end
            lastV = v;  % Update the last velocity value
        
            % If the robot is stuck or too close to an obstacle, reverse the movement
            if stuckTime > stuckDuration || minFrontDist < 0.3
                disp('Robot is stuck, moving backward.');
                v = -0.2;  % Reverse the linear velocity to free the robot
                omega = 0;  % Stop rotation while reversing
                stuckTime = 0;  % Reset stuck time after reversing
            end
        
            % Compute the velocities for the left and right wheels based on linear and angular speeds
            vLeft = v - (omega * L / 2);  % Velocity of the left wheel
            vRight = v + (omega * L / 2);  % Velocity of the right wheel
        
            % Convert linear velocities to angular velocities for the wheels
            omegaLeft = vLeft / r;
            omegaRight = vRight / r;
        
            % Send the computed angular velocities to the robot's wheels
            obj.setWheelVelocities(omegaLeft, omegaRight);
        end

        % Method to set the velocities for both wheels
        function setWheelVelocities(obj, speed_left, speed_right)
            % Sets the velocity for each wheel of the robot.
            %
            % Parameters:
            %   speed_left - The velocity for the left wheel
            %   speed_right - The velocity for the right wheel
            obj.wheels{2}.setVelocity(speed_left);  % Set the velocity for the front-right wheel
            obj.wheels{1}.setVelocity(speed_right);  % Set the velocity for the front-left wheel
        end
    end
end
