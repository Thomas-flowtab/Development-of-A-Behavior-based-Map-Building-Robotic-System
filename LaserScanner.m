classdef LaserScanner
    properties
        laserHandle
        clientID
        sim
    end

    methods
        function obj = LaserScanner(sim, clientID, laserHandle)
            obj.sim = sim;
            obj.clientID = clientID;
            obj.laserHandle = laserHandle;
        end

        function laserData = getLaserData(obj)
            % First, request streaming of laser data
            [res, ~] = obj.sim.simxGetStringSignal(obj.clientID, 'laserData', obj.sim.simx_opmode_streaming);
            
            % Retrieve laser data
            [res, data] = obj.sim.simxGetStringSignal(obj.clientID, 'laserData', obj.sim.simx_opmode_buffer);
            if res == obj.sim.simx_return_ok
                laserData = obj.sim.simxUnpackFloats(data);
                disp('Laser data retrieved');
             
            else
                laserData = [];
                disp('Failed to retrieve laser data.');
            end
        end

        function processedData = processLaserData(obj, laserData)
            % Process laser data by converting from polar to Cartesian coordinates
            if isempty(laserData)
                processedData = [];
                return;
            end
            
            % Assume the laser data is in polar coordinates, convert to Cartesian
            numDataPoints = length(laserData);
            angles = linspace(-180, 180, numDataPoints); %  360-degree field of view
            processedData = zeros(numDataPoints, 2);    % Store (x, y) positions

            for i = 1:numDataPoints
                r = laserData(i);  % Distance measurement
                theta = deg2rad(angles(i));  % Angle in radians
                % Convert polar to Cartesian coordinates
                processedData(i, 1) = r * cos(theta);  % X-coordinate
                processedData(i, 2) = r * sin(theta);  % Y-coordinate
            end
        end
    end
end
