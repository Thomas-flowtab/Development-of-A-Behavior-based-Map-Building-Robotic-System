classdef VisionSensor
    properties
        handle          % Handle for the vision sensor in CoppeliaSim
        visionModes     % Struct to define capture modes (SNAP, STREAM)
        isStreaming     % Boolean to track if streaming is active
    end
    
    methods
        % Constructor to initialize vision sensor path and modes
        function obj = VisionSensor(handle)
            obj.handle = handle;  % handle of the vision sensor
            obj.isStreaming = false;  % Streaming starts as inactive
            obj.visionModes = struct('SNAP', 1, 'STREAM', 2);  % Define the modes
        end      
        
        % Method to capture an image based on the mode (SNAP or STREAM)
        function [image, returnCode] = captureImage(obj, sim, clientID, mode)
            if mode == obj.visionModes.STREAM
                operationMode = sim.simx_opmode_streaming;
            else
                operationMode = sim.simx_opmode_blocking;
            end
            
            % Capture the image from the vision sensor
            [returnCode, resolution, image] = sim.simxGetVisionSensorImage2(clientID, obj.handle, 0, operationMode);
            
            if returnCode ~= sim.simx_return_ok
                disp('Failed to capture image.');
            end
        end
        
        % Method to capture and display the image in the UIAxes
        function captureAndDisplayImage(obj, sim, clientID, app)
            [image, returnCode] = obj.captureImage(sim, clientID, obj.visionModes.SNAP);
            if returnCode == sim.simx_return_ok
                % Display the image in the UIAxes of the app
                imshow(image, 'Parent', app.UIAxes);
            end
        end
    end
end
