classdef SLAMManager < handle
    properties
        laserScanner          % Reference to LaserScanner object for retrieving laser data
        slamObj               % SLAM object for handling SLAM operations
        worldMap              % Occupancy map to store the SLAM result
        minValidRange = 0.1;  % Minimum valid range for laser data
        maxValidRange = 9;    % Maximum valid range for laser data
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
            obj.laserScanner = LaserScanner(connection);
           
        end

        % Method to update the SLAM system
        function obj = updateSLAM(obj)
            % Retrieve the latest laser scan and robot pose from LaserScanner
            [cartesianData, currentPose, currentPoseLidar, minFrontDist] = obj.laserScanner.getScannerData();

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
            % Collect multiple scans before building the map
            numScans = 10;  % Default number of scans if not provided
            
            % Preallocate arrays for scans and poses
            collectedScans = cell(1, numScans);  % Preallocate cell array for scans
            collectedPoses = zeros(numScans, 3); % Preallocate matrix for poses (assuming 2D [x, y, theta])
            
            % Loop to collect scans
            for i = 1:numScans
                % Get the laser scan data and robot pose
                [cartesianData, currentPose ,~,~] = obj.laserScanner.getScannerData();
                
                % Create lidarScan object using the collected cartesian data
                lidarScanObject = lidarScan(cartesianData);
                
                % Add the scan to the SLAM object
                addScan(obj.slamObj, lidarScanObject, currentPose);
                
                % Save the scan and pose in preallocated arrays
                collectedScans{i} = lidarScanObject;
                collectedPoses(i, :) = currentPose;
                
                % Set the robot's start position
                obj.robotStartPosition = currentPose;

                % Optionally, you can add a pause or check for real-time data collection
                pause(0.1);  % If you want to pause between scans
            end
        
            % Once the scans are collected, retrieve all the scans and poses from SLAM object
            [scansSLAM, poses] = scansAndPoses(obj.slamObj);
            
            % Build the occupancy map using the collected scans and poses
            obj.worldMap = buildMap(scansSLAM, poses, obj.slamObj.MapResolution, obj.slamObj.MaxLidarRange);

        end

    end
end
