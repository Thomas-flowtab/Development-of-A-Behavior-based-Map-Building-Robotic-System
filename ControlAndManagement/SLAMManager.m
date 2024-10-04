classdef SLAMManager < handle
    properties
        lidarScanner          % Reference to LaserScanner object for retrieving laser data
        slamObj               % SLAM object for handling SLAM operations
        worldMap              % Occupancy map to store the SLAM result
        minValidRange = 0.1;  % Minimum valid range for laser data
        maxValidRange = 6;    % Maximum valid range for laser data
        currentPosition       % Current robot pose
        robotStartPosition    % Robot's initial position
        frontObstacleDistance % Distance to the closest obstacle in front
    end
    
    methods
        % Constructor to initialize SLAMManager with a LaserScanner
        function obj = SLAMManager(connection, mapResolution, maxRange)
            % Initialize the lidarSLAM object
            obj.slamObj = lidarSLAM(mapResolution, maxRange);
            obj.slamObj.LoopClosureThreshold = 210;
            obj.slamObj.LoopClosureSearchRadius = 8;
            
            % Initialize the laser scanner reference
            obj.lidarScanner = LidarScanner(connection);
        end

        % Method to update the SLAM system
        function obj = updateSLAM(obj)
            % Retrieve the latest laser scan and robot pose from LaserScanner
            [cartesianData, currentPose, currentPoseLidar, minFrontDist] = obj.lidarScanner.getScannerData();

            if ~isempty(cartesianData) && ~isempty(currentPoseLidar)
                % Create a lidarScan object using Cartesian coordinates
                lidarScanObject = lidarScan(cartesianData);

                % Filter out invalid data based on range
                validData = removeInvalidData(lidarScanObject, 'RangeLimits', [obj.minValidRange obj.maxValidRange]);
                
                % Add the scan to the SLAM object
                addScan(obj.slamObj, validData , currentPoseLidar);

                % Retrieve SLAM scans and poses and build the map
                insertRay(obj.worldMap,currentPose,validData,obj.slamObj.MaxLidarRange);
                
                % Update the current robot pose and front obstacle distance
                obj.currentPosition = currentPose;
                obj.frontObstacleDistance = minFrontDist;
                
                % Set the start position if it hasn't been initialized
                if isempty(obj.robotStartPosition)
                    obj.robotStartPosition = currentPose;
                end
               
            else
                disp('Failed to retrieve laser data or current pose from the laser scanner.');
            end
        end

        function obj = initiateOccupancyMap(obj)
            % Initialize a flag to control the loop
            dataRetrieved = false;

            % Keep retrying until cartesianData is not empty
            while ~dataRetrieved
                [cartesianData, currentPose, currentPoseLidar, minFrontDist] = obj.lidarScanner.getScannerData();
                
                % Check if both cartesianData and currentPoseLidar are not empty
                if ~isempty(cartesianData) && ~isempty(currentPose)
                    % Process the LIDAR scan data and build the occupancy map
                    lidarScanObject = lidarScan(cartesianData);

                    % Filter out invalid data based on range
                    validData = removeInvalidData(lidarScanObject, 'RangeLimits', [obj.minValidRange obj.maxValidRange]);
  
                    addScan(obj.slamObj, validData, currentPoseLidar);

                    [scansSLAM, poses] = scansAndPoses(obj.slamObj);
                    obj.worldMap = buildMap(scansSLAM, poses, obj.slamObj.MapResolution, obj.slamObj.MaxLidarRange);
                    
                    obj.frontObstacleDistance = minFrontDist;
                    % Set the start position if it hasn't been initialized
                    if isempty(obj.robotStartPosition)
                        obj.robotStartPosition = currentPose;
                    end
                    obj.currentPosition = currentPoseLidar;

                    % Set flag to true once data is successfully retrieved
                    dataRetrieved = true;
                end
                pause(0.2);
            end
        end
    end
end
