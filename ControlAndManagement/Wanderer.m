classdef Wanderer < handle

    properties
        floorXMin = -4.500  % Minimum X boundary of the floor
        floorXMax =  4.500 % Maximum X boundary of the floor
        floorYMin = -4.500 % Minimum Y boundary of the floor
        floorYMax =  0
        purePursuitController
        movementController
        optimisedPath
        plannedPath
        goalRadius = 0.1;
        targetReached;
        frontObstacleDistance
    end

    methods

        function obj = Wanderer(movementController)

            obj.purePursuitController = controllerPurePursuit;
            obj.purePursuitController.LookaheadDistance = 0.7;  % Look for the next waypoint 0.7 meters ahead
            obj.purePursuitController.DesiredLinearVelocity = 0.2;  % Move at 0.2 m/s
            obj.purePursuitController.MaxAngularVelocity = 0.6;  % Allow up to 0.6 rad/s angular velocity
            
            obj.movementController = movementController;
            obj.targetReached = false;

        end


        function obj = planRobotTrajectory(obj,worldMap,currentPositionOfTheRobot,frontObstacleDistance)
            
            if isempty(currentPositionOfTheRobot) || isempty(worldMap) || isempty(frontObstacleDistance)
                return;
            end
            [x,y] = obj.findNearestFrontier(worldMap,currentPositionOfTheRobot);

            robotX = currentPositionOfTheRobot(1);
            robotY = currentPositionOfTheRobot(2);

            goalPosition = [robotX, robotY] - [x,y];
            
            
            if ~isempty(goalPosition) && ~obj.targetReached
               
                inflateRadious = 0.4;
                inflatedMap = copy(worldMap);
                                
                inflate(inflatedMap , inflateRadious);

                startPosition = [robotX, robotY];

                pathPlanner = plannerAStarGrid(inflatedMap);
                
                disp(startPosition);
                disp(goalPosition);

                % goalPosition = [-1.9150, -1.975];
                % startPosition = [-0.09447, -3.700];

                isStartValid = checkOccupancy(inflatedMap,startPosition) < 0.5;
                isGoalValid = checkOccupancy(inflatedMap,goalPosition) < 0.5;

                goalOccupied = getOccupancy(inflatedMap,goalPosition);
                
                if goalOccupied < 0.2
                    return;
                end 
                

                if isStartValid && isGoalValid
                    try
                        firstPath = plan(pathPlanner, startPosition , goalPosition,'world');
                        if ~isempty(firstPath)

                            obj.optimisedPath = optimizePath(firstPath,inflatedMap);
                            obj.plannedPath = firstPath;
                            obj.purePursuitController.Waypoints = obj.plannedPath; %[obj.optimisedPath(1),obj.optimisedPath(2)];
                        else
                            disp('No plan was found');
                            obj.movementController.moveRobot(0, 0,frontObstacleDistance);
                        end
                    catch exception
                        disp(exception.message);
                        obj.movementController.moveRobot(0, 0,frontObstacleDistance);
                    end
                else
                    disp(isStartValid);
                    disp(isGoalValid);
                end
            end
        end

     function [x, y] = findNearestFrontier(obj, map, currentPositionOfTheRobot)
            % Get the occupancy grid matrix
            occMatrix = getOccupancy(map);  % Get occupancy matrix
            
            % Define the unexplored threshold (typically 0.5 represents unknown)
            unexploredValue = 0.5;
        
            % Get the robot's position in grid coordinates
            robotX = currentPositionOfTheRobot(1);
            robotY = currentPositionOfTheRobot(2);
        
            % Initialize the minimum distance and coordinates for closest unexplored area
            minDist = inf;
            x = NaN;
            y = NaN;
            
            % Get the map size
            [mapHeight, mapWidth] = size(occMatrix);
            
            % Get world limits from the map
            xWorldLimits = map.XWorldLimits; % X limits in world coordinates
            yWorldLimits = map.YWorldLimits; % Y limits in world coordinates
        
            % Loop through the grid cells of the occupancy matrix
            for i = 1:mapHeight
                for j = 1:mapWidth
                    % Check if the cell is unexplored (close to 0.5)
                    occVal = occMatrix(i, j);
                    if abs(occVal - unexploredValue) < 0.01
                        % Convert grid coordinates to world coordinates
                        gridCoords = grid2world(map, [i, j]);
                        worldX = gridCoords(1);
                        worldY = gridCoords(2);
        
                        % Check if the world coordinates are within the X and Y limits of the occupancy map
                        if worldX >= xWorldLimits(1) && worldX <= xWorldLimits(2) && ...
                           worldY >= yWorldLimits(1) && worldY <= yWorldLimits(2)
                       
                            % Check if the point is within the floor boundaries (if needed)
                            if worldX >= obj.floorXMin && worldX <= obj.floorXMax && ...
                               worldY >= obj.floorYMin && worldY <= obj.floorYMax
                               
                                % Compute the Euclidean distance from the robot to the cell (i, j)
                                dist = sqrt((i - robotX)^2 + (j - robotY)^2);
                                
                                % Update the minimum distance and corresponding coordinates
                                if dist < minDist
                                    minDist = dist;
                                    x = round(worldX, 4);  % Extract the X coordinate
                                    y = round(worldY, 4);  % Extract the Y coordinate
        
                                    % Display a debug message with the coordinates and occupancy value
                                    obj.targetReached = false;
                                end
                            end
                        end
                    end
                end
            end
        end

    
        function targetReached = executeRobotMovement(obj,currentPositionOfTheRobot,frontObstacleDistance)
            
            targetReached = false;
            
            % Get the robot's current pose
            robotPosition = currentPositionOfTheRobot;
    
            % Compute control inputs using the pure pursuit controller
            [v, omega] = obj.purePursuitController(robotPosition);
    
            % Move the robot using the control inputs
            obj.movementController.moveRobot(v, omega, frontObstacleDistance);
    
            % Recompute the distance to the goal
            distanceToGoal = norm(robotPosition(1:2) - obj.purePursuitController.Waypoints(end, 1:2));
    
            % Check if the robot has reached the goal
            if distanceToGoal <= obj.goalRadius
                obj.targetReached = true;
                targetReached = obj.targetReached;
                % Stop the robot
                obj.movementController.moveRobot(0, 0, frontObstacleDistance);
            end
        end
    end
end