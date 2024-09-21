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
            disp(['Fetched Orientation: ', num2str(orientation)]);

            
            if returnCodePos == obj.sim.simx_return_ok && returnCodeOrient == obj.sim.simx_return_ok
                % Extract the yaw (theta) angle
                theta = orientation(2);  % yaw is the third component of orientation
                
                % Calculate the difference from the last known theta
                deltaTheta = theta - obj.lastTheta;
                
                % Correct for wraparound by adjusting theta if it jumps across the -pi/pi boundary
                if deltaTheta > pi
                    theta = theta - 2 * pi;
                elseif deltaTheta < -pi
                    theta = theta + 2 * pi;
                end
                
                % Update the last known theta value
                obj.lastTheta = theta;
                
                % Return the updated pose with continuous theta
                pose = [position(1), position(2), theta];  % x, y, and yaw (theta)
                disp(pose)
            else
                pose = [];  % Return empty if retrieval fails
                disp('Failed to retrieve robot pose.');
            end
        end
    end
end
