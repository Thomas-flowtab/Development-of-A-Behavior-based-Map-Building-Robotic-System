classdef Explorer < handle
    properties
        slamHandler     % Reference to SLAMHandler object
        baseControl     % Reference to BaseControl object for movement
        safeDistance    % Safe distance threshold for obstacle avoidance
        exploreTimer    % Timer for periodic exploration
        connection      % Connection to the simulation
        speed           % Speed of the robot movement
        isExploring     % Flag to check if it's exploring
        switchHandler   % SwitchHandler make sure it's set to the right position
    end
    
    methods
        function obj = Explorer(slamHandler, baseControl, safeDistance, speed, connection,switchHandler)
            % Constructor to initialize Explorer class
            obj.slamHandler = slamHandler;
            obj.baseControl = baseControl;
            obj.safeDistance = safeDistance;
            obj.connection = connection;
            obj.speed = speed;
            obj.switchHandler = switchHandler;

            % Initialize a timer for exploration
            obj.exploreTimer = timer('ExecutionMode', 'fixedRate', ...
                                     'Period', 1, ...
                                     'TimerFcn', @(~,~) obj.exploreStep());
            obj.isExploring = false;
        end
        
        function obj = startExploration(obj)
            % Start exploration by starting the timer
            if ~obj.isExploring
                start(obj.exploreTimer);
                obj.isExploring = true;
                obj.switchHandler.Value = "On";
            end
        end
        
        function obj = stopExploration(obj)
            % Stop exploration by stopping the timer
            if obj.isExploring
                stop(obj.exploreTimer);
                obj.isExploring = false;
                obj.switchHandler.Value = "Off";
                obj.baseControl.stop(obj.connection.sim, obj.connection.clientID);
            end
        end
        
        function exploreStep(obj)
            if obj.isExploring
                % A single step in the exploration process, executed periodically by the timer
                % Update SLAM to get the latest map and pose and get the laser
                % scan
                [scan] = obj.slamHandler.updateSLAM();
                
                % Detect obstacles and decide movement
                if obj.detectObstacle(scan)
                    obj.avoidObstacle(scan); % Move left or right based on space
                else
                    obj.moveForward();
                end
            end
        end
        
        
        function isObstacle = detectObstacle(obj, scan)
            % Check if any object is within the safe distance
            minDistance = min(scan.Ranges);
            if minDistance < obj.safeDistance
                isObstacle = true;
            else
                isObstacle = false;
            end
        end
        
        function avoidObstacle(obj, scan)
            % Method to avoid obstacles by turning left or right based on the space
            leftScan = scan.Ranges(scan.Angles > 0);  % Data from left side
            rightScan = scan.Ranges(scan.Angles < 0); % Data from right side
            
            % Calculate the available space on each side
            leftSpace = sum(leftScan > obj.safeDistance);
            rightSpace = sum(rightScan > obj.safeDistance);
            
            % Turn in the direction with more space
            if leftSpace > rightSpace
                disp('Turning left');
                obj.baseControl.move(obj.connection.sim, obj.connection.clientID, obj.baseControl.WheelMovements.COUNTER_CLOCKWISE, obj.speed);
            else
                disp('Turning right');
                obj.baseControl.move(obj.connection.sim, obj.connection.clientID, obj.baseControl.WheelMovements.CLOCKWISE, obj.speed);
            end
            pause(0.5);  % Short pause for the robot to turn
            obj.baseControl.stop(obj.connection.sim, obj.connection.clientID);
        end
        
        function moveForward(obj)
            % Method to move the robot forward
            obj.baseControl.move(obj.connection.sim, obj.connection.clientID, obj.baseControl.WheelMovements.FORWARD, obj.speed);
        end
    end
end
