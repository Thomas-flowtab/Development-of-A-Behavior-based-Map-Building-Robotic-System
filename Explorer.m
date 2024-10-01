classdef Explorer < handle
    properties
        baseControl      % BaseControl class instance for robot movement
        slamHandler      % SLAMHandler class instance for map updates
        sensor           % Hokuyo sensor (LaserScanner)
        robotPose        % RobotPose class instance for robot position
        robotPosition    % Current robot position
        goalPosition     % Frontier goal position
        largestFrontier  % Biggest detected frontiers
        planner          % Path planner based on plannerType
        controller       % Pure Pursuit controller
        explorationTimer % Timer for asynchronous exploration
        isExploring      % Boolean flag to check if exploration is active
        connection       % Connection to Coppelia
        sim              % Simulation remote API (sim)
        clientID         % Client ID for the simulation connection
        robotHandle      % Robot object handle in CoppeliaSim
    end
    
    methods
        function obj = Explorer(connection, baseControl, slamHandler, sensor, robotPose, sim, clientID, robotHandle)
            obj.connection = connection;
            obj.baseControl = baseControl;  % Instance of BaseControl class for movement
            obj.slamHandler = slamHandler;  % Instance of SLAMHandler class for map updates
            obj.sensor = sensor;            % Hokuyo LaserScanner class instance
            obj.robotPose = robotPose;
            obj.sim = sim;                  % Simulation API instance
            obj.clientID = clientID;        % Client ID for the simulation
            obj.robotHandle = robotHandle;  % Robot handle in the simulation
            
            % Initialize the Pure Pursuit controller
            obj.controller = controllerPurePursuit;
            obj.controller.LookaheadDistance = 0.5;  % Look for the next waypoint 0.5 meters ahead
            obj.controller.DesiredLinearVelocity = 0.3;  % Move at 0.3 m/s
            obj.controller.MaxAngularVelocity = 2.0;  % Allow up to 2 rad/s angular velocity


            % Initialize exploration flag
            obj.isExploring = false;
            
            % Initialize asynchronous exploration timer
            obj.explorationTimer = timer('ExecutionMode', 'fixedRate', ...
                                         'Period', 1.0, ...
                                         'TimerFcn', @(~,~) obj.exploreAsync(), ...
                                         'BusyMode', 'queue');
        end
        
        function startExploration(obj)
            % Start the asynchronous exploration process using the timer
            if ~obj.isExploring
                obj.isExploring = true;
                start(obj.explorationTimer);
                disp('Exploration started asynchronously');
            end
        end
        
        function stopExploration(obj)
            % Stop the asynchronous exploration process by stopping the timer
            if obj.isExploring
                obj.isExploring = false;
                stop(obj.explorationTimer);
                obj.baseControl.stop();
                disp('Exploration stopped');
            end
        end
              

        function exploreAsync(obj)
            % Asynchronous exploration loop, triggered by the timer
            if obj.isExploring
                occMap = obj.slamHandler.occupancyMapObject;
                inflateRadious = 0.1;
                inflate(occMap, inflateRadious);
        
                obj.planner = plannerAStarGrid(occMap);
                
                % obj.largestFrontier = obj.findLargestFrontier(occMap);
                % 
                % if isempty(obj.largestFrontier)
                %     disp('Exploration complete. No more frontiers found.');
                %     obj.stopExploration();
                %     return;
                % end
                % 
                % % Validate that the largest frontier is in a free space
                % occStatus = getOccupancy(occMap, obj.largestFrontier);
                % if occStatus > occMap.FreeThreshold
                %     disp('Selected frontier is occupied. Searching for another...');
                %     % Re-select another frontier or handle the situation accordingly
                %     return;
                % end
        
                disp('Move to the selected frontier');
                obj.createPathPlanningToFrontier();
            end
        end
     
        function largestFrontier = findLargestFrontier(obj, occMap)
            % Occupancy thresholds
            freeThreshold = occMap.FreeThreshold;
            occupiedThreshold = occMap.OccupiedThreshold;
        
            % Get the occupancy matrix
            occMatrix = getOccupancy(occMap);
            freeCells = occMatrix < freeThreshold;
            occupiedCells = occMatrix > occupiedThreshold;
            unknownCells = ~(freeCells | occupiedCells);
        
            % Identify frontiers (free cells next to unknown ones)
            kernel = [1 1 1; 1 0 1; 1 1 1];
            unknownNeighborCount = conv2(double(unknownCells), kernel, 'same');
            frontierCells = freeCells & (unknownNeighborCount > 0);
        
            % Label connected components (frontiers)
            [labeledFrontiers, numFrontiers] = bwlabel(frontierCells, 8);
            largestSize = 0;
            largestLabel = 0;
        
            for i = 1:numFrontiers
                currentSize = sum(labeledFrontiers(:) == i);
                if currentSize > largestSize
                    largestSize = currentSize;
                    largestLabel = i;
                end
            end
        
            if largestLabel == 0
                largestFrontier = [];
                disp('No frontiers found in the map.');
                return;
            end
        
            % Extract largest frontier
            largestFrontierMask = (labeledFrontiers == largestLabel);
            [frontierRows, frontierCols] = find(largestFrontierMask);
        
            % Compute resolution (assuming the map has world limits)
            mapLimits = [occMap.XWorldLimits; occMap.YWorldLimits];
            mapSize = size(occMatrix);
            resolution = (mapLimits(1,2) - mapLimits(1,1)) / mapSize(2);
        
            % Convert grid indices to world coordinates
            frontierX = (frontierCols - 1) * resolution + mapLimits(1,1);
            frontierY = (frontierRows - 1) * resolution + mapLimits(2,1);
        
            % Ensure the selected frontier is not in an occupied area
            for i = 1:length(frontierX)
                occStatus = getOccupancy(occMap, [frontierX(i), frontierY(i)]);
                if occStatus <= freeThreshold
                    largestFrontier = [frontierX(i), frontierY(i)];
                    return;
                end
            end
        
            largestFrontier = [];
            disp('No valid frontiers found after filtering occupied areas.');
            obj.stopExploration();
        end

        
        function createPathPlanningToFrontier(obj)
            % disp(obj.largestFrontier);
            % disp('here we go');
            % 
            currentRobotPosition = obj.robotPose.getPose();
            % 
            % 
            % % Select the goal position from the largest frontier
            % obj.goalPosition = [currentRobotPosition(1), currentRobotPosition(2)] - [obj.largestFrontier(1), obj.largestFrontier(2)];
            % disp(obj.goalPosition);
            % 
            % if isempty(obj.goalPosition)
            %     disp('No goal position available');
            %     return;
            % end
            obj.goalPosition = [-4.75, -0.950];
            currPose = obj.goalPosition- [currentRobotPosition(1), currentRobotPosition(2)];
            
            % Perform path planning using the initialized planner
            path = plan(obj.planner, currPose , obj.goalPosition, 'world');
        
            adjustedPath = path - [currentRobotPosition(1), currentRobotPosition(2)];
   

            %  Validate waypoints before moving
            % for i = 1:size(path, 1)
            %     occStatus = getOccupancy(obj.slamHandler.occupancyMapObject, path(i, :));
            %     if occStatus > obj.slamHandler.occupancyMapObject.FreeThreshold
            %         disp(['Waypoint ', num2str(i), ' is in an occupied area. Recomputing path...']);
            %         % Set a new goal or reselect the largest frontier
            %         obj.goalPosition = obj.selectClosestFrontierPoint(obj.largestFrontier, currentRobotPosition);
            %         return;  % Exit this function and restart path planning with the new goal
            %     end
            % end
        
            % If the waypoints are valid, proceed with the adjusted path
            
            disp('path');
            disp(adjustedPath );
        
            if isempty(adjustedPath )
                disp('Path could not be found.');
                return;
            end
        
            % Follow the path using Pure Pursuit
            obj.followPathWithPurePursuit(adjustedPath);
        end


        function followPathWithPurePursuit(obj, path)
            disp('Following the path using Pure Pursuit');
            
            % Ensure the path is of the correct format and type (double Nx2 matrix)
            path = double(path);  % Convert path to double
            if size(path, 2) ~= 2
                error('The path must be an Nx2 matrix where N is the number of waypoints.');
            end
            
            % Set the waypoints from the path
            obj.controller.Waypoints = path;  % Waypoints should be Nx2 matrix of [X, Y] coordinates
            disp('Waypoints passed to the Pure Pursuit controller:');
            disp(obj.controller.Waypoints);
            
            while ~isempty(path) && obj.isExploring
                % Update the robot's position and heading (x, y, theta)
                obj.getRobotPose();
                
                % Update the Pure Pursuit controller with the robot's current position
                currentPose = [obj.robotPosition(1), obj.robotPosition(2), obj.robotPosition(3)];  % x, y, theta
                
                % Compute the control commands (linear velocity, angular velocity)
                [v, omega] = obj.controller(currentPose);  % Compute velocities based on the Pure Pursuit
                
                % Debugging: Display velocity and angular velocity
                disp('Linear velocity (v):');
                disp(v);
                disp('Angular velocity (omega):');
                disp(omega);
                
                % Move the robot using BaseControl (with velocities v and omega)
                obj.baseControl.moveRobot(v, omega);  % Use v and omega to move the robot
                
                % Update SLAM and check if the robot has reached the goal
                obj.slamHandler.updateSLAM(path);
                
                % Check if the robot has reached the last waypoint
                if norm(obj.robotPosition(1:2) - path(end, :)) < obj.controller.LookaheadDistance
                    disp('Goal reached');
                    obj.exploreAsync();
                    break;
                end
            end
        end
        
        function getRobotPose(obj)
            % Retrieve robot pose (position and orientation) from the simulation
            [resPos, position] = obj.sim.simxGetObjectPosition(obj.clientID, obj.robotHandle, -1, obj.sim.simx_opmode_blocking);
            [resOri, orientation] = obj.sim.simxGetObjectOrientation(obj.clientID, obj.robotHandle, -1, obj.sim.simx_opmode_blocking);
            
            % Update robot's position and orientation (ensure theta is in radians)
            if resPos == obj.sim.simx_return_ok && resOri == obj.sim.simx_return_ok
                obj.robotPosition = [position(1), position(2), orientation(3)];  % x, y, theta
                disp('Updated Robot Pose: [x, y, theta]');
                disp(obj.robotPosition);
            else
                disp('Failed to retrieve robot pose from simulation.');
            end
        end

        function updateRobotPose(obj)
            % Update the robot's position based on the movement
             obj.robotPose.setPose(obj.robotPosition);
        end
        
        function goal = selectClosestFrontierPoint(obj, frontierPoints, currentPose)
            % Select the closest frontier point as the goal.
            if isempty(frontierPoints)
                error('Frontier points are empty. Cannot select a goal.');
            end
        
            robotX = currentPose(1);
            robotY = currentPose(2);
        
            distances = sqrt((frontierPoints(:,1) - robotX).^2 + (frontierPoints(:,2) - robotY).^2);
            [~, minIdx] = min(distances);
            goal = frontierPoints(minIdx, :);
            
            % Validate if the goal is in a free space
            occStatus = getOccupancy(obj.slamHandler.occupancyMapObject, goal);
            if occStatus > obj.slamHandler.occupancyMapObject.FreeThreshold
                error('Selected goal location is occupied. Recomputing goal...');
            end
        end
    end
end
