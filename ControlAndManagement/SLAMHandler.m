classdef SLAMHandler < handle
    % SLAMHandler Manages SLAM operations, including maintaining and updating the occupancy map.
    %
    % This class integrates laser scan data with robot pose estimates to perform
    % Simultaneous Localization and Mapping (SLAM). It maintains an occupancy
    % grid map by building it from a subset of scans and poses to optimize performance.
    %
    % Properties:
    %   lidarSlam         - SLAM object managing scan and pose data.
    %   mapTrajectoryAxes - UIAxes for displaying the SLAM trajectory.
    %   mapOccupancyAxes  - UIAxes for displaying the occupancy map.
    %   LaserData         - Stores the latest laser scan data.
    %   laserScanner      - Reference to LaserScanner object for retrieving laser data.
    %   slamTimer         - Timer for periodic SLAM updates.
    %   isUpdating        - Flag indicating if SLAM updates are active.
    %   occupancyMapObject - Property to store the occupancy map.
    
    properties
        lidarSlam         % SLAM object for managing scans and pose estimates
        mapTrajectoryAxes % UIAxes for displaying the SLAM map and trajectory
        mapOccupancyAxes  % UIAxes for displaying the occupancy map
        LaserData         % Property to store the latest laser scan data
        laserScanner      % Reference to LaserScanner object to get laser scan data
        slamTimer         % Timer object for controlling periodic SLAM updates
        isUpdating        % Flag to check if SLAM updates are active
        occupancyMapObject % Property to store and update the occupancy map
    end
    
    methods
        function obj = SLAMHandler(laserScanner, mapResolution, maxRange, loopClosureThreshold, loopClosureSearchRadius, mapTrajectoryAxes, mapOccupancyAxes)
            % Constructor to initialize the SLAM algorithm and UIAxes for visualizations.
            %
            % Parameters:
            %   laserScanner           - LaserScanner object for retrieving laser data.
            %   mapResolution          - Resolution of the occupancy map (meters per cell).
            %   maxRange               - Maximum range of the Lidar sensor (meters).
            %   loopClosureThreshold   - Threshold for detecting loop closures in SLAM.
            %   loopClosureSearchRadius - Search radius for loop closure detection.
            %   mapTrajectoryAxes      - UIAxes handle for displaying SLAM trajectory.
            %   mapOccupancyAxes       - UIAxes handle for displaying the occupancy map.
            
            % Initialize the lidarSLAM object with the provided resolution and range
            obj.lidarSlam = lidarSLAM(mapResolution, maxRange);
            obj.lidarSlam.LoopClosureThreshold = loopClosureThreshold;
            obj.lidarSlam.LoopClosureSearchRadius = loopClosureSearchRadius;
            
            % Assign properties for visualization and laser data
            obj.mapTrajectoryAxes = mapTrajectoryAxes; % UIAxes for SLAM trajectory visualization
            obj.mapOccupancyAxes = mapOccupancyAxes;   % UIAxes for occupancy map visualization
            obj.laserScanner = laserScanner;           % Reference to the laser scanner object
            
            % Initialize a timer for periodic SLAM updates
            obj.slamTimer = timer('ExecutionMode', 'fixedRate', ...
                                   'Period', 1, ... % Update every second
                                   'TimerFcn', @(~,~) obj.updateSLAM([]));
            obj.isUpdating = false; % Initially, SLAM updates are inactive
        end
        
        function obj = startUpdating(obj)
            % Start updating SLAM by starting the timer for periodic updates.
            if ~obj.isUpdating 
                start(obj.slamTimer);
                obj.isUpdating = true; % Set flag to indicate updates are running
            end
        end
        
        function obj = stopUpdating(obj)
            % Stop updating SLAM by stopping the timer.
            if obj.isUpdating 
                stop(obj.slamTimer);
                obj.isUpdating = false; % Set flag to indicate updates have stopped
            end
        end

        function updateSLAM(obj, path)
            % updateSLAM Retrieves new laser data, updates SLAM, and updates maps.
            %
            % This method is called periodically by the slamTimer. It retrieves the latest
            % laser scan data and robot pose, updates the SLAM algorithm, and updates both
            % the trajectory and occupancy maps.
            
            % Retrieve the latest laser scan and robot pose from the LaserScanner object
            [cartesianData, ~, ~, ~, ~, currentPoseLidar] = obj.laserScanner.getScannerData();
            
            % Check if valid data was retrieved
            if ~isempty(cartesianData) && ~isempty(currentPoseLidar)
                % Create a lidarScan object using the Cartesian coordinates
                lidarScanObject = lidarScan(cartesianData);
                
                % Add the new scan and pose to the SLAM object
                addScan(obj.lidarSlam, lidarScanObject, currentPoseLidar);
                
                % Update the SLAM trajectory map visualization
                obj.updateTrajectoryMap(path);

                % Update occupancy map (if required, add code here)
                % map = obj.updateOccupancyMap(path); 
            else
                disp('Failed to retrieve laser data or current pose from the laser scanner.');
            end
        end

        function updateTrajectoryMap(obj, path)
            % updateTrajectoryMap Visualizes the SLAM trajectory on the MapTrajectory UIAxes.
            %
            % This method updates the trajectory visualization based on the current state
            % of the SLAM algorithm and overlays any additional paths if provided.
            
            % Set the current axes to the mapTrajectoryAxes for plotting
            axes(obj.mapTrajectoryAxes);
            
            % Clear previous plots if necessary
            % cla(obj.mapTrajectoryAxes);
            
            % Display the SLAM map and trajectory
            show(obj.lidarSlam, 'Parent', obj.mapTrajectoryAxes);

            % Hold the plot to overlay additional elements
            hold(obj.mapTrajectoryAxes, 'on');

            % If a path is provided, plot the path from plannerAStarGrid
            if ~isempty(path)
                plot(obj.mapTrajectoryAxes, path(:, 1), path(:, 2), 'r-', 'LineWidth', 2); % Red line for path
            end
            
            % Add a title to the plot
            title(obj.mapTrajectoryAxes, 'SLAM Map and Trajectory');

            % Release the hold on the current axes
            hold(obj.mapTrajectoryAxes, 'off');

            % Redraw the plot to reflect updates
            drawnow;
        end
    end
end
