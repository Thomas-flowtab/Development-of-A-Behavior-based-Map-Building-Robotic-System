classdef RobotPose < handle
    % RobotPose Class to manage and log the robot's pose in a simulation.
    %
    % This class maintains the current pose of the robot, provides methods to update
    % and retrieve the pose, and logs all pose updates over time for tracking or analysis purposes.
    
    properties
        pose          % Current robot pose represented as a 1x3 vector [x, y, beta]
        loggedPoses   % Nx3 array storing all logged poses over time
    end
    
    methods
        function obj = RobotPose()
            % Constructor for the RobotPose class.
            %
            % Initializes the robot's pose and sets up the logging mechanism.
            obj.loggedPoses = [];  % Initialize logged poses as an empty array
        end
        
        function setPose(obj, newPose)
            % Updates the robot's current pose and logs the new pose.
            %
            % Parameters:
            %   newPose - 1x3 vector representing the new pose [x, y, beta]
            %
            % This method updates the `pose` property with `newPose` and appends the
            % new pose to the `loggedPoses` array for historical tracking.
            
            % Validate the newPose input
            if isempty(newPose) || length(newPose) ~= 3
                error('newPose must be a 1x3 vector [x, y, beta].');
            end
            
            obj.pose = newPose;  % Update the current pose
            obj.loggedPoses = [obj.loggedPoses; newPose];  % Log the new pose
        end
        
        function pose = getPose(obj)
            % Retrieves the current pose of the robot.
            %
            % Returns:
            %   pose - 1x3 vector representing the current pose [x, y, beta]
            [matlabX, matlabY, matlabTheta]  = obj.transformPose(obj.pose);
            pose = [matlabX, matlabY, matlabTheta];
            % pose = obj.pose;
        end
        
        function poses = getLoggedPoses(obj)
            % Retrieves all logged poses of the robot.
            %
            % Returns:
            %   poses - Nx3 array where each row represents a logged pose [x, y, beta]
            
            poses = obj.loggedPoses;
        end

        function [matlabX, matlabY, matlabTheta] = transformPose(~,coppeliaPose)
                        
            % Extract CoppeliaSim pose
            coppeliaX = coppeliaPose(1);
            coppeliaY = coppeliaPose(2);
            coppeliaTheta = coppeliaPose(3);  % Assuming theta is in radians
            
            % Define rotation angle between CoppeliaSim and MATLAB
            % Example: 90 degrees rotation if CoppeliaSim's X maps to MATLAB's Y
            rotationAngle = pi/3;  % 90 degrees in radians
            
            % Create rotation matrix
            R = [cos(rotationAngle), -sin(rotationAngle);
                 sin(rotationAngle),  cos(rotationAngle)];
            
            % Apply rotation to position
            rotatedPos = R * [coppeliaX; coppeliaY];
            
            % Assign to MATLAB coordinates
            matlabX = rotatedPos(1);
            matlabY = rotatedPos(2);
            
            % Adjust orientation
            matlabTheta = coppeliaTheta + rotationAngle;
            
            % Normalize theta to [0, 2*pi)
            matlabTheta = mod(matlabTheta, 3*pi);
        end

    end
end
