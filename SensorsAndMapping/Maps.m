classdef Maps
    properties
        mapOccupancyAxes      % Axes for occupancy map
        mapTrajectoryAxes     % Axes for trajectory map
    end

    methods
        % Constructor to initialize the class with UI components and SLAM object
        function obj = Maps(mapOccupancyAxes, mapTrajectoryAxes)
            obj.mapOccupancyAxes = mapOccupancyAxes;
            obj.mapTrajectoryAxes = mapTrajectoryAxes;
        end

       % Method to update the occupancy map in UIAxes
        function updateOccupancyMap(obj, slamObj, plannedPath, optimisedPath, initialPosition)
        
            [scansSLAM, poses] = scansAndPoses(slamObj);
            occMap = buildMap(scansSLAM, poses, slamObj.MapResolution, slamObj.MaxLidarRange);
        
            firstPose = poses(1, :); % Get the first pose from poses
            lastPose = poses(end, :); % Get the last row (pose) from poses
        
            % Plot the occupancy map on MapOccupancy UIAxes
            axes(obj.mapOccupancyAxes); % Set MapOccupancy as the current axes
            cla(obj.mapOccupancyAxes);  % Clear previous content
            
            show(occMap, 'Parent', obj.mapOccupancyAxes); % Plot occupancy map
            hold(obj.mapOccupancyAxes, 'on');
            
            % Plot the robot's current position
            if ~isempty(lastPose)
                plot(obj.mapOccupancyAxes, lastPose(1), lastPose(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Robot');
            end
        
            % Offset the planned path by the firstPose
            if ~isempty(plannedPath) && ~isempty(firstPose)
                offset = firstPose(1:2) - initialPosition(1:2); % Compute the offset
                offsetPlannedPath = plannedPath(:, 1:2) + offset; % Apply the offset to the plannedPath
        
                % Plot the offset planned path
                plot(obj.mapOccupancyAxes, offsetPlannedPath(:, 1), offsetPlannedPath(:, 2), 'r-', 'LineWidth', 2);
            end
        
            % Offset the optimised path by the firstPose
            if ~isempty(optimisedPath) && ~isempty(firstPose)
                offset = firstPose(1:2) - initialPosition(1:2); % Compute the offset
                offsetOptimisedPath = optimisedPath(:, 1:2) + offset; % Apply the offset to the optimisedPath
        
                % Plot the offset optimised path
                plot(obj.mapOccupancyAxes, offsetOptimisedPath(:, 1), offsetOptimisedPath(:, 2), 'g:', 'LineWidth', 2);
            end
            
            % Define zoom parameters (e.g., centered around the robot position)
            if ~isempty(firstPose)
                zoomCenterX = firstPose(1); % Center X at robot's X position
                zoomCenterY = firstPose(2); % Center Y at robot's Y position
                zoomWidth = 11;   % Width of the zoomed-in area 
                zoomHeight = 11;  % Height of the zoomed-in area 
        
                % Set the x and y limits to zoom in around the robot
                xlim(obj.mapOccupancyAxes, [zoomCenterX - zoomWidth/2, zoomCenterX + zoomWidth/2]);
                ylim(obj.mapOccupancyAxes, [zoomCenterY - zoomHeight/2, zoomCenterY + zoomHeight/2]);
            end
        
            hold(obj.mapOccupancyAxes, 'off');
            title(obj.mapOccupancyAxes, 'Occupancy Grid Map Built Using Lidar SLAM');
            drawnow;
        end


        % Method to update the trajectory map in UIAxes
        function updateTrajectoryMap(obj,slamObj)
            % Clear previous plots
            cla(obj.mapTrajectoryAxes);

            % Display the SLAM trajectory map
            show(slamObj, 'Parent', obj.mapTrajectoryAxes);

            hold(obj.mapTrajectoryAxes, 'on');

            % Add a title to the plot
            title(obj.mapTrajectoryAxes, 'SLAM Map and Trajectory2');

            hold(obj.mapTrajectoryAxes, 'off');
            drawnow;
        end
    end
end
