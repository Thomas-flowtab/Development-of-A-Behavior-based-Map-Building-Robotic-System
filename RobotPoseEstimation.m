classdef RobotPoseEstimation
    properties
        sim
        clientID
        baseHandle  % Handle to the robot's base in CoppeliaSim
    end
    
    methods
        % Constructor
        function obj = RobotPoseEstimation(sim, clientID, baseHandle)
            obj.sim = sim;
            obj.clientID = clientID;
            obj.baseHandle = baseHandle;
        end
        
        % Get the current pose of the robot [x, y, theta]
        function pose = getPose(obj)
            % Get the position of the robot in the simulation
            [returnCodePos, position] = obj.sim.simxGetObjectPosition(obj.clientID, obj.baseHandle, -1, obj.sim.simx_opmode_blocking);
            % Get the orientation of the robot in the simulation
            [returnCodeOrient, orientation] = obj.sim.simxGetObjectOrientation(obj.clientID, obj.baseHandle, -1, obj.sim.simx_opmode_blocking);
            
            if returnCodePos == obj.sim.simx_return_ok && returnCodeOrient == obj.sim.simx_return_ok
                % Pose consists of [x, y, theta], where theta is the yaw angle (rotation around z-axis)
                pose = [position(1), position(2) orientation(3)];  % x, y, and yaw (theta)
            else
                pose = [];  % Return empty if retrieval fails
                disp('Failed to retrieve robot pose.');
            end
        end
    end
end
