classdef MovementController
    properties
        connection  % Coppeliasim connection 
        wheels      % Array of Wheel objects
        linearSpeed    % Speed value for all wheels
        rotationSpeed  % 
        WheelMovements  %Struct for movement directions (FORWARD, BACKWARD, etc.)
    end
    
    methods
        % Constructor to initialize wheels
        function obj = MovementController(connection,wheelHandles)
            obj.wheels = cell(1, 2);
            obj.connection = connection;
            for i = 1:length(obj.wheels)
                obj.wheels{i} = Wheel(connection,wheelHandles(i));  % Initialize each wheel with radius
            end
            obj.linearSpeed = 1.8;    % Default speed
            obj.rotationSpeed = 0.7 ; % Default speed
            obj.WheelMovements = struct(...
                'FORWARD', 1, 'BACKWARD', 2, 'LEFT', 3, 'RIGHT', 4, 'STOP', 5, 'CLOCKWISE', 6, 'COUNTER_CLOCKWISE', 7);
        end
        
        % Method to move the base in a specified direction
        function move(obj,direction)
            
            switch direction
                case obj.WheelMovements.FORWARD
                    obj.setWheelVelocities( obj.linearSpeed, obj.linearSpeed)
                case obj.WheelMovements.BACKWARD
                    obj.setWheelVelocities( -obj.linearSpeed, -obj.linearSpeed );
                case obj.WheelMovements.LEFT
                    obj.setWheelVelocities(-obj.rotationSpeed, obj.rotationSpeed) ;
                case obj.WheelMovements.RIGHT
                    obj.setWheelVelocities( obj.rotationSpeed, -obj.rotationSpeed);
                case obj.WheelMovements.STOP
                    obj.setWheelVelocities( 0, 0);  % Stop all wheels
                case obj.WheelMovements.CLOCKWISE
                    obj.setWheelVelocities(obj.linearSpeed, 0);
                case obj.WheelMovements.COUNTER_CLOCKWISE
                    obj.setWheelVelocities( 0, obj.linearSpeed);
            end
        end

        function stop(obj)
            obj.setWheelVelocities(0, 0);  % Stop all wheels
        end


       function moveRobot(obj, v, omega, minFrontDist)
            % Define the parameters of the Pioneer P3-DX robot
            r = 0.0975;  % Radius of the wheels (in meters)
            L = 0.331;   % Distance between the wheels (wheelbase in meters)
        
            % Introduce a speed reduction factor (between 0 and 1)
            speedReductionFactor = 0.4;  % Reduce speed by x %, 
        
            % Apply the reduction factor to both linear and angular velocities
            v = v * speedReductionFactor;
            omega = omega * speedReductionFactor;
        
            % Stuck detection variables (persistent to maintain state across function calls)
            persistent stuckTime lastV
            if isempty(stuckTime)
                stuckTime = 0;
                lastV = v;  % Initialize the lastV value
            end
        
            % Stuck detection parameters
            stuckThreshold = 0.1;  % Minimum velocity threshold to consider as stuck
            stuckDuration = 2;  % Time to consider robot stuck in seconds
        
            % Check if the robot is stuck
            if abs(v) < stuckThreshold && abs(lastV) > stuckThreshold
                % Robot is stuck (low velocity despite higher commands)
                stuckTime = stuckTime + 0.1;  % Increment stuck time by the control loop interval (0.1 seconds)
            else
                stuckTime = 0;  % Reset stuck time if the robot is moving
            end
            lastV = v;  % Update lastV
        
            % If the robot is stuck, command it to move backward for a short period
            if stuckTime > stuckDuration || minFrontDist < 0.3
                disp('Robot is stuck, moving backward.');
                v = -0.2;  % Reverse linear velocity (adjust this value as needed)
                omega = 0;  % No turning while reversing
                stuckTime = 0;  % Reset stuck time after reversing
            end

            % Too close
            if minFrontDist < 0.3
                disp('Robot is too close, moving backward.');
                v = -0.1;  % Reverse linear velocity (adjust this value as needed)
                omega = 0;  % No turning while reversing
                stuckTime = 0;  % Reset stuck time after reversing
            end                
        
            % Compute the wheel velocities
            vLeft = v - (omega * L / 2);
            vRight = v + (omega * L / 2);
        
            % Convert wheel linear velocities to angular velocities
            omegaLeft = vLeft / r;
            omegaRight = vRight / r;
        
            % Send the computed wheel velocities to the Pioneer P3-DX robot
            obj.setWheelVelocities(omegaLeft, omegaRight);
        end



        % Method to set velocities for all wheels
        function setWheelVelocities(obj,speed_left, speed_right)
            obj.wheels{2}.setVelocity(speed_left);  % Front-right wheel
            obj.wheels{1}.setVelocity(speed_right);  % Front-left wheel
        end
    end
end
