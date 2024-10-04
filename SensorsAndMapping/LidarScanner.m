classdef LidarScanner < handle
    % LaserScanner Class to interface with a laser scanner in a simulation environment.
    %
    % This class handles the connection to the laser scanner, streaming of laser data,
    % extraction of robot pose, and conversion of laser scan data to Cartesian coordinates
    % suitable for SLAM (Simultaneous Localization and Mapping).

    properties
        clientID         % Identifier for the client connection to the simulation
        sim              % Simulation interface object
        isStreamingData  % Flag indicating if laser data streaming is active
        robotPose        % Object representing the robot's pose (position and orientation)
    end

    methods
        function obj = LidarScanner(connection)
            % Constructor for the LaserScanner class.
            %   
            % Initializes the connection to the simulation, sets up data streaming,
            % and retrieves the initial robot position.
            %
            % Parameters:
            %   connection - Struct containing simulation connection details
            %   robotPose  - Object to store and update the robot's pose

            obj.sim = connection.sim;
            obj.clientID = connection.clientID;
            
            
            % Attempt to start streaming laser data from the simulation
            [returnCode, ~] = connection.sim.simxGetStringSignal(connection.clientID,'laserData', connection.sim.simx_opmode_streaming);
            
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

        function [cartesianData,currentPose,currentPoseLidar,minFrontDist] = getScannerData(obj)
            % Retrieves the latest laser scan data and the current robot pose.
            %
            % Returns:
            %   cartesianData - Nx2 matrix of (x, y) points in Cartesian coordinates
            %   currentPose   - 1x3 vector representing the robot's current pose [x, y, beta]

            try 
                cartesianData = [];
                currentPose = [];
                currentPoseLidar = [];
                minFrontDist = 2;
                % Retrieve the latest packed laser data signal from the simulation buffer
                [res, data] = obj.sim.simxGetStringSignal(obj.clientID, ...
                    'laserData', obj.sim.simx_opmode_buffer);
                
                if res == obj.sim.simx_return_ok
                    % Unpack the data from the float table
                    unpackedData = obj.sim.simxUnpackFloats(data);
                    
                    % Validate the unpacked data
                    if isempty(unpackedData)
                        disp('Received empty laser data.');
                        return;
                    end
                    
                    % The data structure:
                    % [range1, angle1, range2, angle2, ..., rangeN, angleN, x, y, beta]
                    numRanges = (length(unpackedData) - 3) / 2;  % Subtract 3 for robot pose
                    
                    % Extract ranges and angles using vectorized indexing
                    ranges = double(unpackedData(1:2:(2*numRanges-1)));  % Convert to double
                    angles = double(unpackedData(2:2:(2*numRanges)));    % Convert to double

                    % Extract the robot's current pose
                    [robotX, robotY, robotBeta, robotGamma] = obj.extractRobotPosition(unpackedData);
                    currentPose = [robotX, robotY, robotBeta];
                    currentPoseLidar = [robotX, robotY, robotGamma];

                    minFrontDist = obj.detectClosestPoints(ranges,angles);

                    % Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)
                    x_cartesian = ranges .* cos(angles);
                    y_cartesian = ranges .* sin(angles);
                    
                    % Combine x and y coordinates into an Nx2 matrix
                    cartesianData = [x_cartesian', y_cartesian'];
                else
                    disp(['Failed to retrieve laser data from the simulation.',res]);
                end
            catch ME
                % Handle any errors that occur during data retrieval or processing
                disp(['SLAM update error: ', ME.message]);
            end
        end

        function [robotX, robotY, robotBeta,robotGamma] = extractRobotPosition(~, unpackedData)
            % Extracts the robot's pose from the unpacked laser data.
            %
            % Parameters:
            %   unpackedData - Array of floats containing laser data and robot pose
            %
            % Returns:
            %   robotX   - X-coordinate of the robot
            %   robotY   - Y-coordinate of the robot
            %   robotBeta - Orientation (angle) of the robot

            % The last three elements of unpackedData represent the robot's pose
            robotX = unpackedData(end-3);
            robotY = unpackedData(end-2);
            robotBeta = unpackedData(end-1);
            robotGamma = unpackedData(end);
        end


        function minFrontDist = detectClosestPoints(~, ranges,angles)
            % Extract distance data from lidarScan and detect closest points in front

            % Define angle ranges for front, left, and right
            frontIndices = angles >= deg2rad(-45) & angles <= deg2rad(45);

            % Closest point in front
            minFrontDist = min(ranges(frontIndices));
        end
        
    end
end
