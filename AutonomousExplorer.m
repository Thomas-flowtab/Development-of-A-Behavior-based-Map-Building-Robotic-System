classdef AutonomousExplorer
    properties
        baseControl % base control object
        sim % simulation object
        clientID % client ID
        hokuyoSensor % Hokuyo sensor handle
        isWandering% Flag to indicate when to stop
    end
    
    methods
        function obj = AutonomousExplorer(baseControl, sim, clientID, hokuyoSensor)
            obj.baseControl = baseControl;
            obj.sim = sim;
            obj.clientID = clientID;
            obj.hokuyoSensor = hokuyoSensor;
            obj.isWandering = false;
        end
      
            function wanderAndAvoid(obj, speed)
            % Retrieve packed data from the Hokuyo sensor
            [~, packedData] = obj.sim.simxGetStringSignal(obj.clientID, 'hokuyoData', obj.sim.simx_opmode_streaming);
        
            if ~isempty(packedData)
                % Unpack the string data into a float array
                sensorData = obj.sim.simxUnpackFloats(packedData);
                
                % Check for obstacles
                obstacleDetected = obj.detectObstacles(sensorData);
                
                if obstacleDetected
                    % Avoid obstacle by turning
                    disp('Obstacle detected! Turning...');
                    obj.avoidObstacle(speed);
                else
                    % Move forward if no obstacle
                    disp('No obstacle, moving forward...');
                    obj.baseControl.setWheelVelocities(obj.sim, obj.clientID, -speed, -speed, -speed, -speed);
                end
            end
        end
        
        function obstacleDetected = detectObstacles(~, sensorData)
            % Define a threshold distance for obstacle detection (let's randomly say....0.5 meters)
            distanceThreshold = 0.5;
            
            % Initialize arrays for distances and angles
            numPoints = length(sensorData) / 4;  % Each point has [x, y, z, angle]
            distances = zeros(1, numPoints);     % Array to store 2D distances
            anglesDegrees = zeros(1, numPoints); % Array to store angles in degrees
            
            % Iterate over the sensor data and extract [x, y, z, angle]
            for i = 1:numPoints
                x = sensorData((i-1)*4 + 1);     % Extract x value
                y = sensorData((i-1)*4 + 2);     % Extract y value
                z = sensorData((i-1)*4 + 3);     % Extract z value (unused in this case)
                angleRad = sensorData((i-1)*4 + 4); % Extract angle value in radians
                
                % Calculate the 2D distance from the robot's origin (ignoring z)
                distances(i) = sqrt(x^2 + y^2);
                
                % Convert the angle from radians to degrees
                anglesDegrees(i) = rad2deg(angleRad);
            end
            
            % Optionally filter points based on angles in degrees
            % For example, only detect angles between -45 and 45 degrees (front sector)
            frontSector = (anglesDegrees >= -45) & (anglesDegrees <= 45);
            
            % Find points that are within the detection threshold and in the front sector
            obstacleInZone = (distances < distanceThreshold) & frontSector;
            
            % If any obstacles are detected in the zone, return true
            obstacleDetected = any(obstacleInZone);
        end
        
        function avoidObstacle(obj, speed)
            % Logic to turn the robot when an obstacle is detected
            % Example: simple turn (reduce the velocity on one side)
            obj.baseControl.setWheelVelocities(obj.sim, obj.clientID, -speed, speed, -speed, -speed);
        end
        
    end
end
