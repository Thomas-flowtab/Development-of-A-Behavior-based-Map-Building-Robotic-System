classdef LaserScanner < handle
    properties
        clientID
        sim
        isStreamingData
        zones %Array of struct for zones
    end

    methods
        function obj = LaserScanner(connection)
            obj.sim = connection.sim;
            obj.clientID = connection.clientID;
            response = obj.sim.simxGetStringSignal(obj.clientID, 'laserData', obj.sim.simx_opmode_streaming);
            if response == obj.sim.simx_return_ok || response == obj.sim.simx_return_novalue_flag
                obj.isStreamingData = true;
            else
                obj.isStreamingData = false;
            end

            % Initialize 5 zones with minAngle, maxAngle, and threshold
            obj.zones = struct('minAngle', {}, 'maxAngle', {}, 'threshold', {}, 'isFree', {});
            % Define each zone with its angular range and threshold distance
            %Zone1 = front of robot min = 30° and max = 150°
            %Zone2 = front-left of robot min = 150° and max = 180°
            %Zone3 = left of robot min = -180° and max = -130°
            %Zone4 = right of robot min = -50° and max = 0°
            %Zone5 = front-right of robot min = 0° and max = 30°
            obj.zones(1) = struct('minAngle', deg2rad(30), 'maxAngle', deg2rad(150), 'threshold', 0.3, 'isFree', true);
            obj.zones(2) = struct('minAngle', deg2rad(150), 'maxAngle', deg2rad(180), 'threshold', 0.3, 'isFree', true);
            obj.zones(3) = struct('minAngle', deg2rad(-180), 'maxAngle', deg2rad(-130), 'threshold', 0.3, 'isFree', true);
            obj.zones(4) = struct('minAngle', deg2rad(-50), 'maxAngle', deg2rad(0), 'threshold', 0.3, 'isFree', true);
            obj.zones(5) = struct('minAngle', deg2rad(0), 'maxAngle', deg2rad(30), 'threshold', 0.3, 'isFree', true);


        end

        function [cartesianData,currentPose,zones] = GetLaserDecodedData(obj)
            try 
                cartesianData = [];
                currentPose = [];
                % Retrieve the packed laser data signal
                [res, data] = obj.sim.simxGetStringSignal(obj.clientID, 'laserData', obj.sim.simx_opmode_buffer);

                if res == obj.sim.simx_return_ok
                    % Unpack the data from the float table
                    unpackedData = obj.sim.simxUnpackFloats(data);

                    % Check if unpackedData is valid
                    if isempty(unpackedData)
                        disp('Received empty data.');
                        return;
                    end

                    % The data is structured as [range1, angle1, range2, angle2, ..., rangeN, angleN, x, y, beta]
                    numRanges = (length(unpackedData) - 3) / 2;  % Minus 3 for x, y, beta
                    ranges = unpackedData(1:2:(2*numRanges-1));  % Extract ranges
                    angles = unpackedData(2:2:(2*numRanges));    % Extract angles
                    % Extract the robot pose (x, y, beta)
                    robotX = unpackedData(end-2);
                    robotY = unpackedData(end-1);
                    robotBeta = unpackedData(end);
                    
                    currentPose = [robotX, robotY, robotBeta];

                    % Convert ranges and angles to Cartesian coordinates for SLAM
                    x_cartesian = ranges .* cos(angles);
                    y_cartesian = ranges .* sin(angles);
                    cartesianData = [x_cartesian', y_cartesian'];  % Transpose to match expected format

                    % First, assume all zones are free (initialize isFree to true)
                    zones = zeros(1,length(obj.zones)); %array to return 
                    for j = 1:length(obj.zones)
                        obj.zones(j).isFree = true;  % Start by assuming the zone is free
                        zones(j) = true;
                    end
                    
                    % Check each point's angle to determine which zone it falls into
                    for i = 1:numRanges
                        angle = angles(i);
                        distance = ranges(i);
                        % Check each zone
                        for j = 1:length(obj.zones)
                            if angle >= obj.zones(j).minAngle && angle <= obj.zones(j).maxAngle
                                if distance <= obj.zones(j).threshold
                                    obj.zones(j).isFree = false;  % Mark the zone as not free if within threshold
                                    zones(j) = false;
                                end
                            end
                        end
                    end
                else
                    
                    disp('Failed to retrieve laser data from Coppelia.');
                end
             catch ME
                disp(['SLAM update error: ', ME.message]);
            end
        end

         % Method to decode laser data
        function laserData = Decode(~, data)
            % Convert the raw string data from CoppeliaSim into usable laser distance data.
            % Assuming data is a char array (string data from CoppeliaSim)
        
            % Convert the string to uint8 (bytes)
            rawBytes = uint8(data);
            
            % Typecast the bytes to single precision floats
            laserData = typecast(rawBytes, 'single');
            disp('Data was decoded.');
        end
    end
end
