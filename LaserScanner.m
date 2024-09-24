classdef LaserScanner < handle
    properties
        clientID
        sim
        isStreamingData
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
        end

        function [cartesianData,currentPose] = GetLaserDecodedData(obj)
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
