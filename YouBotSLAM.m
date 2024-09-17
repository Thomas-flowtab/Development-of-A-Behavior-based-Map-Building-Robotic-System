classdef YouBotSLAM
    properties
        map                  % Occupancy grid map
        mcl                  % Monte Carlo Localization object
        mapResolution        % Map resolution (cells per meter)
        gridSize             % Grid size of the environment
        initialPose          % Initial robot pose
        clientID             % Connection to CoppeliaSim
        youbotHandle         % Handle for the YouBot in CoppeliaSim
        lidarHandle          % Handle for the LiDAR sensor in CoppeliaSim
    end
    
    methods
        % Constructor
        function obj = YouBotSLAM(clientID, youbotHandle, lidarHandle)
            obj.mapResolution = 10;  % 10 cells per meter (resolution)
            obj.gridSize = [50, 100];  % 5x10 meters
            obj.initialPose = [2.5, 2.5, 0];  % Initial position (x, y, theta)
            
            % Create occupancy grid map
            obj.map = occupancyMap(obj.gridSize(1), obj.gridSize(2), obj.mapResolution);
            
            % Create Monte Carlo Localization object
            obj.mcl = monteCarloLocalization;
            obj.mcl.UseLidarScan = true;
            obj.mcl.Pose = obj.initialPose;
            
            % Assign handles and client ID for CoppeliaSim
            obj.clientID = clientID;
            obj.youbotHandle = youbotHandle;
            obj.lidarHandle = lidarHandle;
        end
        
        % Method to connect to CoppeliaSim and get robot pose
        function [robotPos, robotOri] = getRobotPose(obj)
            % Get robot position and orientation from CoppeliaSim
            [~, robotPos] = simxGetObjectPosition(obj.clientID, obj.youbotHandle, -1, simx_opmode_blocking);
            [~, robotOri] = simxGetObjectOrientation(obj.clientID, obj.youbotHandle, -1, simx_opmode_blocking);
        end
        
        % Method to get sensor data from LiDAR in CoppeliaSim
        function scan = getLidarScan(obj)
            % Placeholder for LiDAR data processing
            % You will need to modify this based on your specific LiDAR setup
            [~, detectionState, detectedPoint] = simxReadProximitySensor(obj.clientID, obj.lidarHandle, simx_opmode_blocking);
            
            if detectionState
                % Process detected points and create a lidarScan object
                ranges = norm(detectedPoint(1:2));  % Get distance (range)
                angles = atan2(detectedPoint(2), detectedPoint(1));  % Get angle
                scan = lidarScan(ranges, angles);
            else
                % If no obstacle is detected, return an empty scan
                scan = lidarScan([], []);
            end
        end
        
        % Method to update the robot's localization and map
        function obj = updateSLAM(obj)
            % Get current robot pose and sensor data
            [robotPos, robotOri] = obj.getRobotPose();
            scan = obj.getLidarScan();
            
            % Update Monte Carlo Localization
            obj.mcl.update(robotPos, scan);
            
            % Update the occupancy map with new sensor data
            insertRay(obj.map, obj.mcl.Pose, scan.Ranges, scan.Angles, 12);
        end
        
        % Method to visualize the occupancy grid map
        function visualizeMap(obj)
            show(obj.map);  % Display the occupancy map
            axis equal;
            drawnow;  % Force MATLAB to update the plot
        end
        
        % Main loop to run SLAM
        function runSLAM(obj)
            while true
                obj = obj.updateSLAM();  % Update localization and map
                obj.visualizeMap();  % Visualize the map
                pause(0.1);  % Simulate real-time operation
            end
        end
    end
end
