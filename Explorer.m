classdef Explorer < handle
    properties
        baseControl  % BaseControl class instance for robot movement
        slamHandler  % SLAMHandler class instance for map updates
        sensor       % Hokuyo sensor (LaserScanner)
        robotPose    % RobotPose class instance for robot position
        robotPosition % Current robot position
        goalPosition  % Frontier goal position
        largestFrontier % Biggest detected frontiers
        planner       % Path planner based on plannerType
        controller    % Pure Pursuit controller
        explorationTimer % Timer for asynchronous exploration
        isExploring      % Boolean flag to check if exploration is active
        connection % Connection to Coppelia
        MapOccupancy    % UIAxes for displaying the occupancy map

    end
    
    methods
        function obj = Explorer(connection,baseControl, slamHandler, sensor, robotPose,mapOccupancy)
            obj.connection = connection;
            obj.baseControl = baseControl;  % Instance of BaseControl class for movement
            obj.slamHandler = slamHandler;  % Instance of SLAMHandler class for map updates
            obj.sensor = sensor;            % Hokuyo LaserScanner class instance
            obj.robotPose = robotPose; 
            % Initialize the Pure Pursuit controller
            obj.controller = controllerPurePursuit;
            obj.controller.LookaheadDistance = 0.5;  % Adjust this value as needed
            obj.controller.DesiredLinearVelocity = 0.3;  % Adjust for desired speed
            obj.controller.MaxAngularVelocity = 1.0;  % Maximum rotation speed
            obj.MapOccupancy = mapOccupancy;
            % Initialize exploration flag
            obj.isExploring = false;
            
            % Initialize asynchronous exploration timer
            obj.explorationTimer = timer('ExecutionMode', 'fixedRate', ...
                                         'Period', 1.0, ...
                                         'TimerFcn', @(~,~) obj.exploreAsync(), ...
                                         'BusyMode', 'queue');
        end
        
        function updateMap(obj)
            % Update the map using SLAMHandler
            obj.slamHandler.updateSLAM();
        end
        
        function detectAndSetFrontiers(obj)
            % Detect frontiers and set the list of frontiers
            occMap = obj.slamHandler.buildOccupancyMap();
            disp('Occupancy map');
            disp(occMap);
            
            obj.largestFrontier = obj.findLargestFrontier(occMap);  % Call the detectFrontiers method from the object
            
        end
        
     function goal = selectClosestFrontierPoint(~,frontierPoints, currentPose)
            % Function to select the closest frontier point as the goal
            %
            % Inputs:
            %   frontierPoints - Nx2 array of [X, Y] coordinates
            %   currentPose - 1x3 array [x, y, theta]
            %
            % Outputs:
            %   goal - 1x2 array [X, Y] coordinates representing the goal position
        
            if isempty(frontierPoints)
                error('Frontier points are empty. Cannot select a goal.');
            end
        
            % Extract the robot's current position
            robotX = currentPose(1);
            robotY = currentPose(2);
        
            % Compute Euclidean distances from the robot to each frontier point
            distances = sqrt((frontierPoints(:,1) - robotX).^2 + (frontierPoints(:,2) - robotY).^2);
        
            % Find the index of the minimum distance
            [~, minIdx] = min(distances);
        
            % Select the closest frontier point as the goal
            goal = frontierPoints(minIdx, :);
        end


        function largestFrontier = findLargestFrontier(~,map)
            % Function to find the largest frontier on an occupancy map
            %
            % Inputs:
            %   map - occupancyMap object
            %
            % Outputs:
            %   largestFrontier - Nx2 array of [X, Y] coordinates representing the largest frontier
        
            % Define thresholds for free, occupied, and unknown cells
            freeThreshold = map.FreeThreshold;          % Default is 0.2
            occupiedThreshold = map.OccupiedThreshold;  % Default is 0.65
        
            % Get the occupancy matrix from the map
            occMatrix = getOccupancy(map);
            
            % Classify cells based on occupancy probabilities
            freeCells = occMatrix < freeThreshold;
            occupiedCells = occMatrix > occupiedThreshold;
            unknownCells = ~(freeCells | occupiedCells); % Cells with occupancy ~0.5
            
            % Identify frontier cells
            % A frontier cell is a free cell that has at least one unknown neighbor
            % Use convolution to check the 8-connected neighborhood
            kernel = [1 1 1; 1 0 1; 1 1 1];
            unknownNeighborCount = conv2(double(unknownCells), kernel, 'same');
            frontierCells = freeCells & (unknownNeighborCount > 0);
            
            % Label connected components (frontiers)
            % Use 8-connectivity to consider diagonal neighbors
            [labeledFrontiers, numFrontiers] = bwlabel(frontierCells, 8);
            
            % Initialize variables to track the largest frontier
            largestSize = 0;
            largestLabel = 0;
            
            % Iterate through each labeled frontier to find the largest one
            for i = 1:numFrontiers
                % Find the size (number of cells) of the current frontier
                currentSize = sum(labeledFrontiers(:) == i);
                % Update if the current frontier is larger than the previous largest
                if currentSize > largestSize
                    largestSize = currentSize;
                    largestLabel = i;
                end
            end
            
            % Check if any frontiers were found
            if largestLabel == 0
                largestFrontier = [];
                disp('No frontiers found in the map.');
                return;
            end
            
            % Extract the largest frontier based on the label
            largestFrontierMask = (labeledFrontiers == largestLabel);
            
            % Get the grid indices of the largest frontier cells
            [frontierRows, frontierCols] = find(largestFrontierMask);
            
            % Convert grid indices to world coordinates
            if ismethod(map, 'grid2world')
                % Method 1: Use grid2world if available
                frontierWorld = map.grid2world([frontierCols, frontierRows]);
                frontierX = frontierWorld(:,1);
                frontierY = frontierWorld(:,2);
            else
                % Method 2: Manual conversion using map properties
                resolution = map.GridResolution;          % meters per cell
                origin = map.GridOriginInWorld;           % [x_origin, y_origin]
                
                frontierX = (frontierCols - 1) * resolution + origin(1);
                frontierY = (frontierRows - 1) * resolution + origin(2);
            end
            
            % Combine X and Y coordinates into an Nx2 array
            largestFrontier = [frontierX, frontierY];
        end


        
       function followPathToFrontier(obj)
            % Plan a path to the closest frontier and move the robot along it
            obj.getRobotPose();
            currentPose = [round(obj.robotPosition(1)),round(obj.robotPosition(2))];
            
            disp(obj.largestFrontier);

            obj.goalPosition = obj.selectClosestFrontierPoint(obj.largestFrontier, currentPose);


            if isempty(obj.goalPosition)
                disp('No goal position available');
                return;
            end
        
            
            startPose = [round(obj.robotPosition(1)),round(obj.robotPosition(2))];  % Current robot position
            goalPose = [obj.goalPosition(1),obj.goalPosition(2)];    % Goal frontier position
            
            startGrid = startPose(1:2);  % Convert to integer grid indices
            goalGrid = round(goalPose);         % Convert to integer grid indices
        
            disp('Start Grid:');
            disp(startGrid);
            disp('Goal Grid:');
            disp(goalGrid);
            
            % Perform path planning using the initialized planner
            path = plan(obj.planner, startGrid, goalGrid);
            
            disp('path');

            map = obj.slamHandler.getOccupancyMap();

            show(map); % Plot occupancy map
            hold on
                if ~isempty(path)
                    plot(startGrid(1),startGrid(2),"rx")
                    plot(goalGrid(1),goalGrid(2),"go")
                    plot(path(:, 1), path(:, 2), 'r-', 'LineWidth', 2); % Red line connecting the points
                end
            hold off
            
            
            disp(path);
            hold off


            
            % Check if the path is valid
            if isempty(path)
                disp('Path could not be found.');
                return;
            end
            
            disp('Path found:');
            
        
            % Follow the path using Pure Pursuit
            obj.followPathWithPurePursuit(path);
        end


        
        function followPathWithPurePursuit(obj, path)
            % Setup the Pure Pursuit controller waypoints from the planned path
            disp('following path now:');

            obj.controller.Waypoints = path;
            
            
            
            % Continuously move the robot along the path
            while ~isempty(path) && obj.isExploring
                obj.getRobotPose();  % Update the robot position
                
                
                % disp('Compute the control signals');
                % Compute the control signals (linear velocity and angular velocity)
                [v, omega] = obj.controller(obj.robotPosition);
                

                % disp('Move the robot');
                % disp('speed');
                % disp(v);
                % disp('angle');
                % disp(omega);

                % Move the robot using BaseControl (assuming BaseControl has move method)
                obj.baseControl.moveRobot(obj.connection,v, omega);  % Move the robot based on Pure Pursuit output
                
                % Update robot's position and the map (this ensures SLAM is being updated)
                obj.slamHandler.updateSLAM();
                
                % Check if the robot has reached the goal
                if norm(obj.robotPosition(1:2) - path(end, :)) < obj.controller.LookaheadDistance
                    disp('Goal reached');
                    break;
                end
            end
        end

        
        function exploreAsync(obj)
            % Asynchronous exploration loop, triggered by the timer
            if obj.isExploring
                obj.updateMap();             % Update the map
                pause(1); % Give it time for the occupancy map to update
                obj.detectAndSetFrontiers(); % Detect and set frontiers
                
                if isempty(obj.largestFrontier)
                    disp('Exploration complete. No more frontiers found.');
                    obj.stopExploration();   % Stop exploration if no frontiers found
                    return;
                end
                
                disp('Move to the selected frontier ');
                obj.followPathToFrontier();  % Move to the selected frontier
            end
        end
        
        function getRobotPose(obj)
            % Get the current robot position and orientation
            % disp('Get the current robot position and orientation');

            % Convert world coordinates to grid coordinates
            % robotCurrentPosition = obj.robotPose.getPose();
            % [x,y] = obj.ConvertCoordinates(robotCurrentPosition(1),robotCurrentPosition(2));
            % obj.robotPosition = [x,y,robotCurrentPosition(3)];
            
            obj.robotPosition = obj.robotPose.getPose();
        end
        
        function updateRobotPose(obj)
            % Update the robot's position based on the movement
             obj.robotPose.setPose(obj.robotPosition);
        end
        
        function startExploration(obj)
            % Start the asynchronous exploration process using the timer
            if ~obj.isExploring
                obj.isExploring = true;
                
                map = obj.slamHandler.getOccupancyMap();

                % Initialize the path planner based on user input
                obj.planner = plannerAStarGrid(map);
             

                start(obj.explorationTimer);
                disp('Exploration started asynchronously');
            end
        end
        
        function stopExploration(obj)
            % Stop the asynchronous exploration process by stopping the timer
            if obj.isExploring
                obj.isExploring = false;
                stop(obj.explorationTimer);
                disp('Exploration stopped');
            end
        end


      
        function [x_grid, y_grid] = ConvertCoordinates(~, x_world, y_world)
            % worldToGrid - Converts CoppeliaSim world coordinates to occupancy map grid coordinates
            %
            % Syntax: [x_grid, y_grid] = worldToGrid(x_world, y_world, resolution, gridOrigin, mapSize)
            %
            % Inputs:
            %   x_world   - World X coordinate from CoppeliaSim (in meters)
            %   y_world   - World Y coordinate from CoppeliaSim (in meters)
            %   resolution - The resolution of the occupancy grid (size of each grid cell in meters)
            %   gridOrigin - [x_origin, y_origin], the world coordinate origin in CoppeliaSim (e.g., [-20, -20])
            %   mapSize    - Size of the grid (e.g., [400, 400] for a 400x400 grid)
            %
            % Outputs:
            %   x_grid    - The corresponding X grid index in the occupancy map
            %   y_grid    - The corresponding Y grid index in the occupancy map
            
            % Define parameters for the world-to-grid mapping
            resolution = 20;     % Each grid cell represents 
            gridOrigin = [-10, -10];  % CoppeliaSim world coordinates origin (in meters)
            mapSize = [420,420];  % Size of the occupancy map (10x5 grid cells)
            
            % Example CoppeliaSim world coordinates (in meters)
            % x_world = 2.5;  % X coordinate in CoppeliaSim world
            % y_world = -1.5; % Y coordinate in CoppeliaSim world

            % Convert world coordinates to grid indices using resolution and grid origin
            
            x_grid = round((x_world - gridOrigin(1)) / resolution);
            y_grid = round((y_world - gridOrigin(2)) / resolution);
            
            % Clamp the grid coordinates to ensure they are within the valid map size
            x_grid = max(1, min(mapSize(1), x_grid));  % Clamp x_grid between 1 and mapSize(1)
            y_grid = max(1, min(mapSize(2), y_grid));  % Clamp y_grid between 1 and mapSize(2)
            
            % Display the result for debugging
            fprintf('World coordinates (%.2f, %.2f) map to grid coordinates (%d, %d)\n', x_world, y_world, x_grid, y_grid);
        end


    end
end
