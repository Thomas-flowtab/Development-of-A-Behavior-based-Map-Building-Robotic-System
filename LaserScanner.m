classdef LaserScanner < handle
    properties
        laserHandle
        clientID
        sim
        isStreamingData
    end

    methods
        function obj = LaserScanner(connection, laserHandle)
            obj.sim = connection.sim;
            obj.clientID = connection.clientID;
            obj.laserHandle = laserHandle;
            response = obj.sim.simxGetStringSignal(obj.clientID, 'laserData', obj.sim.simx_opmode_streaming);
            if response == obj.sim.simx_return_ok || response == obj.sim.simx_return_novalue_flag
                obj.isStreamingData = true;
            else
                obj.isStreamingData = false;
            end
        end

        function cartesianData = GetLaserDecodedData(obj)
            try 
            % Retrieve laser data
            [res, data] = obj.sim.simxGetStringSignal(obj.clientID, 'laserData', obj.sim.simx_opmode_buffer);
                        
                if res == obj.sim.simx_return_ok
                
                    tempLaserData = obj.Decode(data);
                    

                    % Reshape the data and restrict to valid theta ranges
                    reshapedData = reshape(tempLaserData, 4, [])';
                    
                    x = reshapedData(:, 1);
                    y = reshapedData(:, 2);
                    theta = reshapedData(:, 4);
                    validThetaRange = [-pi, pi];
                    validThetaIndices = theta >= validThetaRange(1) & theta <= validThetaRange(2);
                    theta = theta(validThetaIndices);
                    x = x(validThetaIndices);
                    y = y(validThetaIndices);
                    r = sqrt(x.^2 + y.^2);
                    minRange = 0.1;
                    maxRange = 8;
                    validIndices = r > minRange & r < maxRange;
                    r(~validIndices) = NaN;
                    validCartesianIndices = ~isnan(r);
                    x_cartesian = r(validCartesianIndices) .* cos(theta(validCartesianIndices));
                    y_cartesian = r(validCartesianIndices) .* sin(theta(validCartesianIndices));
                    cartesianData = [x_cartesian, y_cartesian];
                    disp('Laser data retrieved');
                else
                    cartesianData = [];
                    disp('Failed to retrieve laser data.');
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
