classdef RobotPoseEstimation
    properties
        sim
        clientID
        baseHandle  % Handle to the robot's base in CoppeliaSim
        lastTheta = 0; % Store the last known theta to ensure continuity
    end
    
    methods
        % Constructor
        function obj = RobotPoseEstimation(sim, clientID, baseHandle)
            obj.sim = sim;
            obj.clientID = clientID;
            obj.baseHandle = baseHandle;
        end
        
        function pose = getPose(obj)
            % Get the position of the robot in the simulation
            [returnCodePos, position] = obj.sim.simxGetObjectPosition(obj.clientID, obj.baseHandle, -1, obj.sim.simx_opmode_blocking);
            % Get the orientation of the robot in the simulation
            [returnCodeOrient, orientation] = obj.sim.simxGetObjectOrientation(obj.clientID, obj.baseHandle, -1, obj.sim.simx_opmode_blocking);
        
            if returnCodePos == obj.sim.simx_return_ok && returnCodeOrient == obj.sim.simx_return_ok
                % Extract the yaw (theta) angle (orientation(3) for yaw in CoppeliaSim)
                theta = orientation(2);
        
                % Unwrap the theta to maintain continuity over multiple turns
                theta = atan2(sin(theta), cos(theta));  % This ensures theta is between -pi and pi
        
                % Smoothing out rapid fluctuations (optional step)
                smoothFactor = 0.9;
                theta = smoothFactor * obj.lastTheta + (1 - smoothFactor) * theta;
        
                % Update the last known theta value
                obj.lastTheta = theta;
        
                % Return the updated pose with continuous theta
                pose = [position(1), position(2), theta];  % x, y, and yaw (theta)
                disp(['Position: X=', num2str(position(1)), ', Y=', num2str(position(2)), ', Theta=', num2str(theta)]);
            else
                pose = [];  % Return empty if retrieval fails
                disp('Failed to retrieve robot pose.');
            end
        end

    end
end
