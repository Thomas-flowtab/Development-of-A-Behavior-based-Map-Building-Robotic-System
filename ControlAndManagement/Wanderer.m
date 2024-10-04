classdef Wanderer < handle

    properties
        floorXMin = -4.500  % Minimum X boundary of the floor
        floorXMax =  4.500 % Maximum X boundary of the floor
        floorYMin = -4.500 % Minimum Y boundary of the floor
        floorYMax =  0
        purePursuitController
        movementController

        goalRadius = 0.1;
        targetReached = false;

        frontObstacleDistance
    end

    methods
        function obj = Wanderer(movementController)

            obj.purePursuitController = controllerPurePursuit;
            obj.purePursuitController.LookaheadDistance = 0.7;  % Look for the next waypoint 0.5 meters ahead
            obj.purePursuitController.DesiredLinearVelocity = 0.2;  % Move at 0.3 m/s
            obj.purePursuitController.MaxAngularVelocity = 0.6;  % Allow up to 2 rad/s angular velocity
            
            obj.movementController = movementController;

        end

        function [x, y] = findNearestFrontier(~,map,currentPositionOfTheRobot)
                
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
            
                % Loop through the grid cells of the occupancy matrix
                for i = 1:mapHeight
                    for j = 1:mapWidth
                        % Check if the cell is unexplored (close to 0.5)
                        occVal = occMatrix(i,j);
                        if abs(occVal - unexploredValue) < 0.01
                            % Convert grid coordinates to world coordinates
                            gridCoords = grid2world(map, [i, j]);
                            worldX = gridCoords(1);
                            worldY = gridCoords(2);

                            % Check if the point is within the floor boundaries
                            if worldX >= app.floorXMin && worldX <= app.floorXMax && worldY >= app.floorYMin && worldY <= app.floorYMax
                                % Compute the Euclidean distance from the robot to the cell (i, j)
                                dist = sqrt((i - robotX)^2 + (j - robotY)^2);
                                
                                % Update the minimum distance and corresponding coordinates
                                if dist < minDist
                                    minDist = dist;
                                    x = round(worldX, 4);  % Extract the X coordinate
                                    y = round(worldY, 4);  % Extract the Y coordinate

                                    % Display a debug message with the coordinates and occupancy value
                                    app.targetReached = false;
                                end
                            end
                        end
                    end
                end
            end
    
        function executeRobotMovement(~,currentPositionOfTheRobot)
            % Get the robot's current pose
            robotPosition = currentPositionOfTheRobot;  % Assuming you have a method to get the current robot pose
    
            % Compute control inputs using the pure pursuit controller
            [v, omega] = obj.purePursuitController(robotPosition);
    
            % Move the robot using the control inputs
            app.movementController.moveRobot(v, omega,app.frontObstacleDistance);
    
            % Recompute the distance to the goal
            distanceToGoal = norm(robotPosition(1:2) - app.purePursuitController.Waypoints(end, 1:2));
    
            % Check if the robot has reached the goal
            if distanceToGoal <= app.goalRadius
                app.targetReached = true;
    
                % Stop the robot
                app.movementController.moveRobot(0, 0,app.frontObstacleDistance);
    
                % Stop the timer
                stop(app.timerMoveTheRobot);
            end
        end


        
       

        function planRobotTrajectory(obj,currentPositionOfTheRobot)
            [x,y] = obj.findNearestFrontier();

            robotX = currentPositionOfTheRobot(1);
            robotY = currentPositionOfTheRobot(2);

            goalPosition = [robotX, robotY] - [x,y];
            disp('ai got one');
            disp(goalPosition);

            if ~isempty(goalPosition) && ~app.targetReached
               
                inflateRadious = 0.3;
                inflatedMap = copy(app.worldMap);
                                
                inflate(inflatedMap , inflateRadious);

                startPosition = [robotX, robotY];

                pathPlanner = plannerAStarGrid(inflatedMap);
                % goalPosition = [-1.9150, -1.975];

                isStartValid = checkOccupancy(inflatedMap,startPosition) < 0.5;
                isGoalValid = checkOccupancy(inflatedMap,goalPosition) < 0.5;

                goalOccupied = getOccupancy(inflatedMap,goalPosition);
                disp('goalOccupied');
                disp(goalPosition);
                disp(goalOccupied);
                disp('startPosition');
                disp(startPosition);

                if isStartValid && isGoalValid
                    try
                        firstPath = plan(pathPlanner, startPosition , goalPosition,'world');
                        if ~isempty(firstPath)

                            app.optimisedPath = optimizePath(firstPath,inflatedMap);
                            app.plannedPath = firstPath;
                            app.purePursuitController.Waypoints = app.plannedPath; %[app.optimisedPath(1),app.optimisedPath(2)];
                            
                            if strcmp(app.timerMoveTheRobot.Running, 'off')
                                start(app.timerMoveTheRobot);
                            end
                        else
                            disp('No plan was found');
                            app.movementController.moveRobot(0, 0,app.frontObstacleDistance);
                        end
                    catch exception
                        disp(exception.message);
                        app.movementController.moveRobot(0, 0,app.frontObstacleDistance);
                        app.stopAllProcesses();
                    end
                else
                    disp(isStartValid);
                    disp(isGoalValid);
                end
            end
        end

    end
end