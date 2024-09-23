classdef SLAMHandler
    properties
        slamAlg         % SLAM algorithm object
        MapTrajectory   % UIAxes for displaying the SLAM map and trajectory
        MapOccupancy    % UIAxes for displaying the occupancy map
        LaserData       % Property to store laser scan data
        robotPose       % Robot pose estimation object
    end
    
    methods
        function obj = SLAMHandler(mapResolution, maxRange, loopClosureThreshold, loopClosureSearchRadius, robotPose, mapTrajectoryAxes, mapOccupancyAxes)
            % Constructor to initialize the SLAM algorithm and UIAxes
            obj.slamAlg = lidarSLAM(mapResolution, maxRange);
            obj.slamAlg.LoopClosureThreshold = loopClosureThreshold;
            obj.slamAlg.LoopClosureSearchRadius = loopClosureSearchRadius;
            obj.robotPose = robotPose;
            obj.MapTrajectory = mapTrajectoryAxes; % Assign the UIAxes for SLAM map
            obj.MapOccupancy = mapOccupancyAxes;   % Assign the UIAxes for occupancy map
        end
        
        function updateSLAM(obj, connection)
    try
        % Retrieve the packed laser data signal
        [res, data] = connection.sim.simxGetStringSignal(connection.clientID, 'laserData', connection.sim.simx_opmode_buffer);

        if res == connection.sim.simx_return_ok
            % Unpack the data from the float table
            unpackedData = connection.sim.simxUnpackFloats(data);

            % Check if unpackedData is valid
            if isempty(unpackedData)
                disp('Received empty data.');
                return;
            end

            % The data is structured as [range1, angle1, range2, angle2, ..., rangeN, angleN, x, y, beta]
            numRanges = (length(unpackedData) - 3) / 2;  % Minus 3 for x, y, beta
            ranges = unpackedData(1:2:(2*numRanges-1));  % Extract ranges
            angles = unpackedData(2:2:(2*numRanges));    % Extract angles

            % Extract the robot pose (x, y, beta)
            robotX = unpackedData(end-2);
            robotY = unpackedData(end-1);
            robotBeta = unpackedData(end);
            disp(robotBeta)

            % Convert ranges and angles to Cartesian coordinates for SLAM
            x_cartesian = ranges .* cos(angles);
            y_cartesian = ranges .* sin(angles);
            cartesianData = [x_cartesian', y_cartesian'];  % Transpose to match expected format


            % Create a lidarScan object using Cartesian coordinates
            lidarScanObject = lidarScan(cartesianData);

            % Update the robot's current pose with the x, y, and beta (yaw)
            currentPose = [robotX, robotY, robotBeta];
            obj.robotPose.setPose(currentPose);  % Assuming robotPose is a class with setPose() method

            % Add the scan to the SLAM algorithm
            addScan(obj.slamAlg, lidarScanObject, currentPose);

            % Plot SLAM map on MapTrajectory UIAxes
            axes(obj.MapTrajectory); % Set MapTrajectory as the current axes
            cla(obj.MapTrajectory);  % Clear previous content
            show(obj.slamAlg, 'Parent', obj.MapTrajectory); % Plot SLAM map
            title(obj.MapTrajectory, 'SLAM Map and Trajectory');
            drawnow;

            % Optionally, update occupancy map every 1000 scans
            if mod(length(obj.robotPose.getLoggedPoses()), 1000) == 0
                obj.buildOccupancyMap();  % Update occupancy map
            end
        else
            disp('Failed to retrieve laser data.');
        end
    catch ME
        disp(['SLAM update error: ', ME.message]);
    end
end


        
        function buildOccupancyMap(obj)
            % Create the occupancy map from the SLAM data
            [scans, optimizedPoses] = scansAndPoses(obj.slamAlg);
            map = buildMap(scans, optimizedPoses, obj.slamAlg.MapResolution, obj.slamAlg.MaxLidarRange);
            
            % Plot the occupancy map on MapOccupancy UIAxes
            axes(obj.MapOccupancy); % Set MapOccupancy as the current axes
            cla(obj.MapOccupancy);  % Clear previous content
            show(map, 'Parent', obj.MapOccupancy); % Plot occupancy map
            hold(obj.MapOccupancy, 'on');
            show(obj.slamAlg.PoseGraph, 'IDs', 'off', 'Parent', obj.MapOccupancy); % Show the PoseGraph on the same axes
            hold(obj.MapOccupancy, 'off');
            title(obj.MapOccupancy, 'Occupancy Grid Map Built Using Lidar SLAM');
            drawnow;
        end
    end
end
