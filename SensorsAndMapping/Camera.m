classdef Camera
    properties
        camHandle      % Handle for the camera sensor in CoppeliaSim
        sim
        clientID
        cameraAxes
        hImage 
    end
    
    methods
        % Constructor to initialize camera sensor handle and modes
        function obj = Camera(connection,camHandle,cameraAxes)
            obj.camHandle = camHandle;  % Assign the handle of the camera sensor
            obj.sim = connection.sim;
            obj.clientID = connection.clientID;
            obj.cameraAxes = cameraAxes;

            % Start image streaming
            [~, resolution, ~] = obj.sim.simxGetVisionSensorImage2(obj.clientID, obj.camHandle, 0, obj.sim.simx_opmode_streaming);
            % Initialize image display on UIAxes
            obj.hImage = imshow(zeros(resolution(2), resolution(1), 3), 'Parent', obj.cameraAxes);
        end      
       
        % Method to capture and show the image in the application interface
        function streamImage(obj)
            % Get the current camera frame from CoppeliaSim
            [res, ~, image] = obj.sim.simxGetVisionSensorImage2(obj.clientID, obj.camHandle, 0, obj.sim.simx_opmode_buffer);
            
            if res == obj.sim.simx_return_ok
                set(obj.hImage, 'CData', image);  % Update the image in UIAxes
                drawnow;  % Refresh display
            end
        end
    end
end
