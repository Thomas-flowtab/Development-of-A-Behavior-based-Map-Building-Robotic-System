classdef SLAMHandler < handle
    properties
        slamAlg         % SLAM algorithm object
        MapTrajectory   % UIAxes for displaying the SLAM map and trajectory
        MapOccupancy    % UIAxes for displaying the occupancy map
        LaserData       % Property to store laser scan data
        robotPose       % Robot pose estimation object
        laserScanner    % Reference to LaserScanner object for retrieving laser data
        slamTimer       % Timer for periodic update
        isUpdating      % Flag to check if it's updating
 
    end
    
    methods
        function obj = SLAMHandler(mapResolution, maxRange, loopClosureThreshold, loopClosureSearchRadius, robotPose, mapTrajectoryAxes, mapOccupancyAxes,laserScanner)
            % Constructor to initialize the SLAM algorithm and UIAxes
            obj.slamAlg = lidarSLAM(mapResolution, maxRange);
            obj.slamAlg.LoopClosureThreshold = loopClosureThreshold;
            obj.slamAlg.LoopClosureSearchRadius = loopClosureSearchRadius;
            obj.robotPose = robotPose;
            obj.MapTrajectory = mapTrajectoryAxes; % Assign the UIAxes for SLAM map
            obj.MapOccupancy = mapOccupancyAxes;   % Assign the UIAxes for occupancy map
            obj.laserScanner = laserScanner;
            obj.slamTimer = timer('ExecutionMode', 'fixedRate', ...
                                     'Period', 1, ...
                                     'TimerFcn', @(~,~) obj.updateSLAM());
            obj.isUpdating = false;
        end
        
        function obj = startUpdating(obj)
            % Start updating by starting the timer
            if ~obj.isUpdating 
                start(obj.slamTimer);
                obj.isUpdating = true;
            end
        end
        
        function obj = stopUpdating(obj)
            % Stop updating by stopping the timer
            if obj.isUpdating 
                stop(obj.slamTimer);
                obj.isUpdating = false;
            end
        end

        function [lidarScanObject] = updateSLAM(obj)

            try
                % Retrieve the packed laser data signal
                [cartesianData,currentPose] = obj.laserScanner.GetLaserDecodedData();
                if ~isempty(cartesianData)
                    
                    % Create a lidarScan object using Cartesian coordinates
                    lidarScanObject = lidarScan(cartesianData);
                            
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
                    disp('Failed to retrieve laser data from the laser scanner.');
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
