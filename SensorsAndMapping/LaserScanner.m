classdef LaserScanner < handle
    % LaserScanner Class to interface with a laser scanner in a simulation environment.
    %
    % This class handles the connection to the laser scanner, streaming of laser data,
    % extraction of robot pose, and conversion of laser scan data to Cartesian coordinates
    % suitable for SLAM (Simultaneous Localization and Mapping).

    properties
        clientID         % Identifier for the client connection to the simulation
        sim              % Simulation interface object
        isStreamingData  % Flag indicating if laser data streaming is active
    end

    methods
        function obj = LaserScanner(connection)
            % Constructor for the LaserScanner class.
            %   
            % Initializes the connection to the simulation, sets up data streaming,
            % and retrieves the initial robot position.
            %
            % Parameters:
            %   connection - Struct containing simulation connection details

            obj.sim = connection.sim;
            obj.clientID = connection.clientID;
            
            % Attempt to start streaming laser data from the simulation
            [returnCode, ~] = connection.sim.simxGetStringSignal(connection.clientID, ...
                'laserData', connection.sim.simx_opmode_streaming);
            
            % Check if streaming was successfully initialized
            if returnCode == connection.sim.simx_return_ok || ...
               returnCode == connection.sim.simx_return_novalue_flag
                obj.isStreamingData = true;
                disp('Laser data streaming initialized successfully.');
            else
                obj.isStreamingData = false;
                disp('Failed to initialize laser data streaming.');
            end
        end

        function [cartesianData, currentPose, ranges, angles, minFrontDist, currentPoseLidar] = getScannerData(obj)
            % Retrieves the latest laser scan data and the current robot pose.
            %
            % Returns:
            %   cartesianData    - Nx2 matrix of (x, y) points in Cartesian coordinates
            %   currentPose      - 1x3 vector representing the robot's current pose [x, y, beta]
            %   ranges           - Array of ranges (distance values from the laser)
            %   angles           - Array of angles corresponding to the ranges
            %   minFrontDist     - Closest detected object in front of the robot
            %   currentPoseLidar - Pose of the robot relative to the LIDAR sensor

            try
                cartesianData = [];
                currentPose = [];
                currentPoseLidar = [];
                ranges = [];
                angles = [];
                minFrontDist = 2;  % Default maximum distance for front detection
                
                % Retrieve the latest packed laser data signal from the simulation buffer
                [res, data] = obj.sim.simxGetStringSignal(obj.clientID, ...
                    'laserData', obj.sim.simx_opmode_buffer);
                
                % Check if the data was retrieved successfully
                if res == obj.sim.simx_return_ok
                    % Unpack the data from the float table
                    unpackedData = obj.sim.simxUnpackFloats(data);
                    
                    % Validate the unpacked data to avoid processing empty signals
                    if isempty(unpackedData)
                        disp('Received empty laser data.');
                        return;
                    end
                    
                    % The data structure:
                    % [range1, angle1, range2, angle2, ..., rangeN, angleN, x, y, beta, gamma]
                    numRanges = (length(unpackedData) - 3) / 2;  % Subtract 3 for robot pose
                    
                    % Extract ranges and angles using vectorized indexing
                    ranges = double(unpackedData(1:2:(2*numRanges-1)));  % Convert to double
                    angles = double(unpackedData(2:2:(2*numRanges)));    % Convert to double

                    % Extract the robot's current pose
                    [robotX, robotY, robotBeta, robotGamma] = obj.extractRobotPosition(unpackedData);
                    currentPose = [robotX, robotY, robotBeta];
                    currentPoseLidar = [robotX, robotY, robotGamma];  % Pose relative to the LIDAR sensor

                    % Find the closest obstacle in front of the robot
                    minFrontDist = obj.detectClosestPoints(ranges, angles);

                    % Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)
                    x_cartesian = ranges .* cos(angles);
                    y_cartesian = ranges .* sin(angles);
                    
                    % Combine x and y coordinates into an Nx2 matrix
                    cartesianData = [x_cartesian', y_cartesian'];
                else
                    disp('Failed to retrieve laser data from the simulation.');
                end
            catch ME
                % Handle any errors that occur during data retrieval or processing
                disp(['SLAM update error: ', ME.message]);
            end
        end

        function [robotX, robotY, robotBeta, robotGamma] = extractRobotPosition(~, unpackedData)
            % Extracts the robot's pose from the unpacked laser data.
            %
            % Parameters:
            %   unpackedData - Array of floats containing laser data and robot pose
            %
            % Returns:
            %   robotX   - X-coordinate of the robot
            %   robotY   - Y-coordinate of the robot
            %   robotBeta - Orientation (angle) of the robot
            %   robotGamma - Additional orientation data (for LIDAR alignment)

            % The last three elements of unpackedData represent the robot's pose
            robotX = unpackedData(end-3);
            robotY = unpackedData(end-2);
            robotBeta = unpackedData(end-1);
            robotGamma = unpackedData(end);  % Orientation related to the LIDAR sensor
        end

        function minFrontDist = detectClosestPoints(~, ranges, angles)
            % Detects the closest object in front of the robot based on the laser scan data.
            %
            % Parameters:
            %   ranges - Array of distances from the laser scanner
            %   angles - Array of angles corresponding to the distances
            %
            % Returns:
            %   minFrontDist - The minimum distance detected in the forward direction

            % Define angle ranges for the front of the robot (±45 degrees)
            frontIndices = angles >= deg2rad(-45) & angles <= deg2rad(45);

            % Find the closest point in front
            minFrontDist = min(ranges(frontIndices));
        end
    end
end
