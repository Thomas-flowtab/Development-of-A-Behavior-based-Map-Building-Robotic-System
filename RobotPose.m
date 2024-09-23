classdef RobotPose
    properties
        pose          % Current robot pose [x, y, beta]
        loggedPoses   % Array to store logged poses over time
    end
    
    methods
        function obj = RobotPose(initialPose)
            % Constructor: Initialize the robot pose
            if nargin == 0
                obj.pose = [0, 0, 0];  % Default to origin [0, 0, 0]
            else
                obj.pose = initialPose;
            end
            obj.loggedPoses = [];  % Initialize logged poses as an empty array
        end
        
        function setPose(obj, newPose)
            % Update the robot pose and log the new pose
            obj.pose = newPose;
            obj.loggedPoses = [obj.loggedPoses; newPose];  % Log the new pose
        end
        
        function pose = getPose(obj)
            % Return the current robot pose
            pose = obj.pose;
        end
        
        function poses = getLoggedPoses(obj)
            % Return all logged poses
            poses = obj.loggedPoses;
        end
    end
end
