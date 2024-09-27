classdef SLAMHandler < handle
    % SLAMHandler Manages SLAM operations, including maintaining and updating the occupancy map.
    %
    % This class integrates laser scan data with robot pose estimates to perform
    % Simultaneous Localization and Mapping (SLAM). It maintains an occupancy
    % grid map by building it from a subset of scans and poses to optimize performance.
    %
    % Properties:
    %   lidarSlam         - SLAM object managing scan and pose data.
    %   MapTrajectory     - UIAxes for displaying the SLAM trajectory.
    %   MapOccupancy      - UIAxes for displaying the occupancy map.
    %   LaserData         - Stores the latest laser scan data.
    %   robotPose         - Robot pose estimation object.
    %   laserScanner      - Reference to LaserScanner object for retrieving laser data.
    %   slamTimer         - Timer for periodic SLAM updates.
    %   isUpdating        - Flag indicating if SLAM updates are active.
    
    properties
        lidarSlam         % SLAM object
        mapTrajectoryAxes     % UIAxes for displaying the SLAM map and trajectory
        mapOccupancyAxes      % UIAxes for displaying the occupancy map
        LaserData         % Property to store laser scan data
        robotPose         % Robot pose estimation object
        laserScanner      % Reference to LaserScanner object for retrieving laser data
        slamTimer         % Timer for periodic update
        isUpdating        % Flag to check if it's updating
    end
    
    methods
        function obj = SLAMHandler(laserScanner,robotPose,mapResolution, maxRange, loopClosureThreshold, loopClosureSearchRadius, mapTrajectoryAxes, mapOccupancyAxes)
            % Constructor to initialize the SLAM algorithm and UIAxes
            %
            % Parameters:
            %   mapResolution         - Resolution of the occupancy map (meters per cell)
            %   maxRange              - Maximum range of the Lidar sensor (meters)
            %   loopClosureThreshold  - Threshold for loop closure detection
            %   loopClosureSearchRadius - Search radius for loop closure
            %   robotPose             - Robot pose estimation object
            %   mapTrajectoryAxes     - UIAxes handle for SLAM trajectory
            %   mapOccupancyAxes      - UIAxes handle for occupancy map
            %   laserScanner          - LaserScanner object for retrieving laser data
            %   maxBufferSize         - Maximum number of scans and poses to retain
            
            % Initialize SLAM object
            obj.lidarSlam = lidarSLAM(mapResolution, maxRange);
            obj.lidarSlam.LoopClosureThreshold = loopClosureThreshold;
            obj.lidarSlam.LoopClosureSearchRadius = loopClosureSearchRadius;
            
            % Assign properties
            obj.robotPose = robotPose;
            obj.mapTrajectoryAxes = mapTrajectoryAxes; % Assign the UIAxes for SLAM map
            obj.mapOccupancyAxes = mapOccupancyAxes;   % Assign the UIAxes for occupancy map
            obj.laserScanner = laserScanner;
            
            
            % Initialize timer for periodic updates
            obj.slamTimer = timer('ExecutionMode', 'fixedRate', ...
                                   'Period', 1, ... % Update every second; adjust as needed
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

        function occupancyMap = updateSLAM(obj)
            % updateSLAM Retrieves new laser data, updates SLAM, and updates maps.
            %
            % This method is called periodically by the slamTimer. It retrieves the latest
            % laser scan data and robot pose, updates the SLAM algorithm, and updates both
            % the trajectory and occupancy maps.
            
            
                % Retrieve the latest laser scan and robot pose from the LaserScanner
                [cartesianData, currentPose] = obj.laserScanner.getScannerData();
                
                if ~isempty(cartesianData) && ~isempty(currentPose)
                    % Create a lidarScan object using Cartesian coordinates
                    lidarScanObject = lidarScan(cartesianData);
                                        

                    % Update the robot's current pose
                    obj.robotPose.setPose(currentPose);  
                    
                    
                    % Add the scan to the SLAM object
                    addScan(obj.lidarSlam, lidarScanObject, currentPose);
                    
                    % Update the trajectory map
                    obj.updateTrajectoryMap();

                    
                    % Update occupancy map
                    map = obj.updateOccupancyMap();  
                    occupancyMap = getOccupancy(map);
                    
                    % Update occupancy map based loggs
                else
                    disp('Failed to retrieve laser data or current pose from the laser scanner.');
                end
            
        end
       
        function map = updateOccupancyMap(obj)

            % Create the occupancy map from the SLAM data
            [scans, optimizedPoses] = scansAndPoses(obj.lidarSlam);
            map = buildMap(scans, optimizedPoses,  40, 15);
            
            % Plot the occupancy map on MapOccupancy UIAxes
            axes(obj.mapOccupancyAxes); % Set MapOccupancy as the current axes
            cla(obj.mapOccupancyAxes);  % Clear previous content
            show(map, 'Parent', obj.mapOccupancyAxes); % Plot occupancy map
            hold(obj.mapOccupancyAxes, 'on');
             % Plot the robot's current position
            currentPose = obj.robotPose.getPose();

            if ~isempty(currentPose)
                plot(obj.mapOccupancyAxes, currentPose(1), currentPose(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Robot');
            end
            % show(obj.lidarSlam.PoseGraph, 'IDs', 'off', 'Parent', obj.mapOccupancyAxes); % Show the PoseGraph on the same axes
            
            hold(obj.mapOccupancyAxes, 'off');
            title(obj.mapOccupancyAxes, 'Occupancy Grid Map Built Using Lidar SLAM');
            drawnow;
        end

        function updateTrajectoryMap(obj)
            % updateTrajectoryMap Visualizes the SLAM trajectory on the MapTrajectory UIAxes.
            %
            % This method updates the trajectory visualization based on the current state
            % of the SLAM algorithm.
            
            % Set the current axes to MapTrajectory
            axes(obj.mapTrajectoryAxes);
            
            % Clear previous plots
            cla(obj.mapTrajectoryAxes);
            
            % Display the SLAM trajectory map
            show(obj.lidarSlam, 'Parent', obj.mapTrajectoryAxes);
            
            % Add a title to the plot
            title(obj.mapTrajectoryAxes, 'SLAM Map and Trajectory');
            drawnow;
        end
    end
end
