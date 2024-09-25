classdef Explorer < handle
    properties
        slamHandler           % Reference to SLAMHandler object
        movementControl       % Reference to BaseControl object for movement
        safeDistance          % Safe distance threshold for obstacle avoidance
        exploreTimer          % Timer for periodic exploration
        connection            % Connection to the simulation
        speed                 % Speed of the robot movement
        isExploring           % Flag to check if it's exploring
        switchHandler         % SwitchHandler to control switch state
        slowTurnSpeed         % Speed for slow turns
        reverseSpeed          % Speed for reversing
        lastLeftDistance      % Store previous left distance for gap detection
        lastRightDistance     % Store previous right distance for gap detection
        gapThreshold          % Threshold for detecting gaps (e.g., sudden increase in distance)
        laserHandler          % Reference to LaserScanner object
    end
    
    methods
        function obj = Explorer(slamHandler, baseControl, safeDistance, speed, connection, switchHandler, gapThreshold,laserHandler)
            % Constructor to initialize Explorer class
            obj.slamHandler = slamHandler;
            obj.movementControl = baseControl;
            obj.safeDistance = safeDistance;
            obj.connection = connection;
            obj.speed = speed;
            obj.slowTurnSpeed = speed * 0.5; % Reduce speed for smoother turns
            obj.reverseSpeed = speed * 0.4;  % Speed for reversing
            obj.switchHandler = switchHandler;
            obj.lastLeftDistance = inf;   % Initialize to a large value
            obj.lastRightDistance = inf;  % Initialize to a large value
            obj.gapThreshold = gapThreshold;  % Set gap detection threshold
            obj.laserHandler = laserHandler;

            % Initialize a timer for exploration
            obj.exploreTimer = timer('ExecutionMode', 'fixedRate', ...
                                     'Period', 1, ...
                                     'TimerFcn', @(~,~) obj.exploreStep());
            obj.isExploring = false;
        end
        
        function startExploration(obj)
            % Start exploration by starting the timer
            if ~obj.isExploring
                start(obj.exploreTimer);
                obj.isExploring = true;
                obj.switchHandler.Value = "On";
            end
        end
        
        function stopExploration(obj)
            % Stop exploration by stopping the timer
            if obj.isExploring
                stop(obj.exploreTimer);
                obj.isExploring = false;
                obj.switchHandler.Value = "Off";
                obj.movementControl.stop(obj.connection.sim, obj.connection.clientID);
            end
        end
        
        function exploreStep(obj)
            if obj.isExploring
                % A single step in the exploration process, executed periodically by the timer
                [~, ~, zones] = obj.laserHandler.GetLaserDecodedData();

                % Zones array: 1 (Front), 2 (Front-left), 3 (Left), 4 (Right), 5 (Front-right)
                if ~zones(1)  % Front zone is not free (obstacle detected)
                    % Stop the robot if the front is blocked
                    obj.movementControl.stop(obj.connection.sim, obj.connection.clientID);

                    % Turn based on the left or right zone availability
                    if zones(4)  % Right zone is free
                        obj.turn('clockwise');
                    elseif zones(3)  % Left zone is free
                        obj.turn('counter_clockwise');
                    else
                        % If both sides are blocked, reverse
                        obj.movementControl.move(obj.connection.sim, obj.connection.clientID, obj.movementControl.WheelMovements.BACKWARD, obj.reverseSpeed);
                    end
                else
                    % If the front zone is free, move forward
                    obj.movementControl.move(obj.connection.sim, obj.connection.clientID, obj.movementControl.WheelMovements.FORWARD, obj.speed);
                end
            end
        end

        function [minFrontDist, minLeftDist, minRightDist, minFrontAngle, minLeftAngle, minRightAngle] = detectClosestPoints(~, scan)
            % Extract distance data from lidarScan and detect closest points in front, left, and right
            distances = scan.Ranges;
            angles = scan.Angles;

            % Define angle ranges for front, left, and right
            frontIndices = angles >= deg2rad(30) & angles <= deg2rad(150);
            leftIndices = angles >= deg2rad(150) & angles <= deg2rad(180);
            rightIndices = angles >= deg2rad(0) & angles <= deg2rad(30);

            % Handle cases where no distance is available
            [minFrontDist, idxFront] = min(distances(frontIndices), [], 'omitnan');
            [minLeftDist, idxLeft] = min(distances(leftIndices), [], 'omitnan');
            [minRightDist, idxRight] = min(distances(rightIndices), [], 'omitnan');

            % Extract the corresponding angles (converted to degrees for debug output)
            minFrontAngle = rad2deg(angles(frontIndices));
            minLeftAngle = rad2deg(angles(leftIndices));
            minRightAngle = rad2deg(angles(rightIndices));

            % Get the specific angle for the minimum distance
            minFrontAngle = minFrontAngle(idxFront);
            minLeftAngle = minLeftAngle(idxLeft);
            minRightAngle = minRightAngle(idxRight);
            
            % Debug: Print the distances and angles
            fprintf('Front: Distance = %.2f, Angle = %.2f°\n', minFrontDist, minFrontAngle);
            fprintf('Left: Distance = %.2f, Angle = %.2f°\n', minLeftDist, minLeftAngle);
            fprintf('Right: Distance = %.2f, Angle = %.2f°\n', minRightDist, minRightAngle);
        end

        function isObstacle = detectObstacle(obj, scan)
            % Detect if there's an obstacle in front, left, or right
            [minFrontDist, minLeftDist, minRightDist, minFrontAngle, minLeftAngle, minRightAngle] = obj.detectClosestPoints(scan);

            % Check for obstacles in the safe distance range
            isObstacle = any([minFrontDist, minLeftDist, minRightDist] < obj.safeDistance);

            % Debug: Print a message if an obstacle is detected
            if isObstacle
                fprintf('Obstacle detected!\n');
                fprintf('Closest obstacle detected at:\n');
                fprintf('Front: Distance = %.2f at %.2f°\n', minFrontDist, minFrontAngle);
                fprintf('Left: Distance = %.2f at %.2f°\n', minLeftDist, minLeftAngle);
                fprintf('Right: Distance = %.2f at %.2f°\n', minRightDist, minRightAngle);
            end
        end

        function avoidObstacle(obj, scan)
            % Avoid obstacle based on distance to the nearest point in front, left, and right
            [minFrontDist, minLeftDist, minRightDist, ~, ~, ~] = obj.detectClosestPoints(scan);

            % Check if the robot is trapped in a corner
            if obj.isCornerTrap(minFrontDist, minLeftDist, minRightDist)
                obj.escapeCorner();
                return;
            end

            % Handle front obstacle
            if minFrontDist < obj.safeDistance
                obj.movementControl.stop(obj.connection.sim, obj.connection.clientID);
            end

            % Detect and explore gaps if available
            if obj.detectGap(minLeftDist, minRightDist)
                return;
            end

            % Determine the turn direction based on available space
            if minLeftDist > minRightDist
                obj.turn('counter_clockwise', scan);
            else
                obj.turn('clockwise', scan);
            end
        end

        function isGapDetected = detectGap(obj, minLeftDist, minRightDist)
            % Detect gaps based on changes in left and right distances
            isGapDetected = false;

            if (minLeftDist > obj.lastLeftDistance + obj.gapThreshold)
                obj.turn('counter_clockwise');
                isGapDetected = true;
            elseif (minRightDist > obj.lastRightDistance + obj.gapThreshold)
                obj.turn('clockwise');
                isGapDetected = true;
            end

            % Update previous distances for gap detection
            obj.lastLeftDistance = minLeftDist;
            obj.lastRightDistance = minRightDist;
        end

        function turn(obj, direction, ~)
            % Turn the robot based on direction ('clockwise' or 'counter_clockwise') until clear
            movement = obj.movementControl.WheelMovements.(upper(direction));  % Get movement based on direction
            minFrontDist = obj.safeDistance - 1;
            turnDuration = 0;
            maxTurnDuration = 5;  % Limit the turn duration to prevent infinite loops
            while minFrontDist < obj.safeDistance && turnDuration < maxTurnDuration
                obj.movementControl.move(obj.connection.sim, obj.connection.clientID, movement, obj.slowTurnSpeed);
                pause(0.2);
                scan = obj.slamHandler.updateSLAM();
                [minFrontDist, ~, ~] = obj.detectClosestPoints(scan);
                turnDuration = turnDuration + 1;  % Increment the turn duration
            end
            obj.explore();
        end

        function explore(obj)
            % Move forward when no obstacles are detected
            obj.movementControl.move(obj.connection.sim, obj.connection.clientID, obj.movementControl.WheelMovements.FORWARD, obj.speed);
        end
        
        % --- Corner Trap Handling ---
        
        function isTrapped = isCornerTrap(obj, frontDist, leftDist, rightDist)
            % Detects if the robot is trapped in a corner (too close to walls on all sides)
            isTrapped = (frontDist < obj.safeDistance) && ...
                        (leftDist < obj.safeDistance) && ...
                        (rightDist < obj.safeDistance);
            
            if isTrapped
                fprintf('Corner trap detected! Front = %.2f, Left = %.2f, Right = %.2f\n', ...
                    frontDist, leftDist, rightDist);
            end
        end

        function escapeCorner(obj)
            % Escape from a corner by reversing and turning in a random direction
            fprintf('Escaping corner trap...\n');
            
            % Step 1: Reverse to create distance
            obj.movementControl.move(obj.connection.sim, obj.connection.clientID, obj.movementControl.WheelMovements.BACKWARD, obj.reverseSpeed);
            pause(2); % Reverse for 2 seconds
            obj.movementControl.stop(obj.connection.sim, obj.connection.clientID);
            
            % Step 2: Turn randomly either clockwise or counterclockwise
            turnDirection = randi([0, 1]); % 0 for clockwise, 1 for counterclockwise
            if turnDirection == 0
                obj.turn('clockwise');
            else
                obj.turn('counter_clockwise');
            end
        end
    end
end
