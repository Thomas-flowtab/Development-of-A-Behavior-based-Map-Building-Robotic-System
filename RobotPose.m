classdef RobotPose < handle
    properties
        pose          % Current robot pose represented as a 1x3 vector [x, y, theta]
        loggedPoses   % Nx3 array storing all logged poses over time
        sim           % Simulation API object
        clientID      % Client ID for the connection
        robotHandle   % Handle to the robot object in CoppeliaSim
    end
    
    methods
        function obj = RobotPose(sim, clientID, robotHandle)
            % Constructor for the RobotPose class.
            % Initialize the robot's pose and set up logging.
            obj.sim = sim;                 % Assign the remote API instance
            obj.clientID = clientID;       % Assign the client ID for communication
            obj.robotHandle = robotHandle; % Assign the robot handle

            obj.loggedPoses = [];  % Initialize logged poses as an empty array
        end
        
        % Method to retrieve the robot's pose (position and orientation) from CoppeliaSim
        function getRobotPose(obj)
            % Retrieve the robot's position (x, y, z) from CoppeliaSim
            [resPos, position] = obj.sim.simxGetObjectPosition(obj.clientID, obj.robotHandle, -1, obj.sim.simx_opmode_blocking);
            % Retrieve the robot's orientation (yaw, pitch, roll) from CoppeliaSim
            [resOrient, orientation] = obj.sim.simxGetObjectOrientation(obj.clientID, obj.robotHandle, -1, obj.sim.simx_opmode_blocking);
            
            % If both position and orientation retrievals were successful, update the pose
            if resPos == obj.sim.simx_return_ok && resOrient == obj.sim.simx_return_ok
                % Store the robot's current pose as [x, y, theta]
                obj.pose = [position(1), position(2), orientation(3)];  % x, y, theta (z-axis rotation)
                disp('Robot Pose (x, y, theta):');
                disp(obj.pose);  % Print the pose for debugging
            else
                % If retrieval failed, display an error message
                disp('Failed to retrieve robot pose from simulation.');
                disp(['Position return code: ', num2str(resPos)]);       % Debug: position return code
                disp(['Orientation return code: ', num2str(resOrient)]); % Debug: orientation return code
            end
        end
        
        % Method to return the current pose of the robot
        function pose = getPose(obj)
            pose = obj.pose;  % Return the stored pose
        end
        
        % Method to set a new pose and log it
        function setPose(obj, newPose)
            obj.pose = newPose;                           % Update the current pose
            obj.loggedPoses = [obj.loggedPoses; newPose]; % Log the new pose
        end
        
        % Method to return all logged poses
        function poses = getLoggedPoses(obj)
            poses = obj.loggedPoses;  % Return the logged poses
        end
    end
end
