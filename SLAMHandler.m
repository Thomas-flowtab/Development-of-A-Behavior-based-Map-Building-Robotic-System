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
            % Retrieve laser data and robot pose
            try
                [res, data] = connection.sim.simxGetStringSignal(connection.clientID, 'laserData', connection.sim.simx_opmode_buffer);
                
                if res == connection.sim.simx_return_ok
                    % Decode and process the laser data
                    obj.LaserData = obj.decode(data);
                    
                    % Process the laser data and create a lidarScan object
                    reshapedData = reshape(obj.LaserData, 4, [])';
                    x = reshapedData(:, 1);
                    y = reshapedData(:, 2);
                    theta = reshapedData(:, 4);
                    validThetaRange = [-pi, pi];
                    validThetaIndices = (theta >= validThetaRange(1)) & (theta <= validThetaRange(2));
                    theta = theta(validThetaIndices);
                    x = x(validThetaIndices);
                    y = y(validThetaIndices);
                    r = sqrt(x.^2 + y.^2);
                    minRange = 0.1;
                    maxRange = 8;
                    validIndices = (r > minRange) & (r < maxRange);
                    r(~validIndices) = NaN;
                    validCartesianIndices = ~isnan(r);
                    x_cartesian = r(validCartesianIndices) .* cos(theta(validCartesianIndices));
                    y_cartesian = r(validCartesianIndices) .* sin(theta(validCartesianIndices));
                    cartesianData = [x_cartesian, y_cartesian];
                    lidarScanObject = lidarScan(cartesianData);
                    currentPose = obj.robotPose.getPose();
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
        
        function laserData = decode(~, data)
            % Decode the raw laser data from CoppeliaSim
            rawBytes = uint8(data);
            laserData = typecast(rawBytes, 'single');
        end
    end
end
