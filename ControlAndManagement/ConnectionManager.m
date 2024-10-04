classdef ConnectionManager < handle
    properties
        clientID                     % Client ID for the connection to CoppeliaSim
        sim                          % Instance of the remote API (sim)
        isSimulationRunning = false; % Flag to check if the simulation is running
    end

    methods
        % Constructor
        function obj = ConnectionManager()
            % Initialize the remote API object
            obj.sim = remApi('remoteApi');
            obj.clientID = -1; % Initialize clientID to an invalid state to indicate no connection
        end

        % Connect to CoppeliaSim
        function success = connect(obj)
            % Close any previous connections to avoid conflicts
            obj.sim.simxFinish(-1);
            
            % Attempt to connect to CoppeliaSim on localhost at the default port 19997
            obj.clientID = obj.sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
            
            % Check if the connection was successful
            if obj.clientID > -1
                disp('Connected to CoppeliaSim.');
                
                % Start the simulation after successful connection
                obj.startSimulation();
                
                % Return success status based on whether the simulation started successfully
                success = obj.isSimulationRunning;
            else
                % Failed to connect to CoppeliaSim
                disp('Failed to connect to CoppeliaSim.');
                success = false;
            end
        end

        % Disconnect from CoppeliaSim
        function disconnect(obj)
            % Check if there is an active connection
            if obj.clientID > -1
                % Stop the simulation before disconnecting
                obj.stopSimulation();
                
                % Close the connection to CoppeliaSim
                obj.sim.simxFinish(obj.clientID);
                disp('Disconnected from CoppeliaSim.');
                
                % Reset the client ID to indicate no connection
                obj.clientID = -1;
            else
                % No active connection to disconnect
                disp('No active connection to disconnect.');
            end
        end

        % Start the simulation in CoppeliaSim
        function startSimulation(obj)
            % Ensure there is an active connection before starting the simulation
            if obj.clientID > -1
                % Start the simulation in blocking mode and get the return code
                returnCode = obj.sim.simxStartSimulation(obj.clientID, obj.sim.simx_opmode_blocking);
                
                % Check if the simulation started successfully
                if returnCode == obj.sim.simx_return_ok
                    disp('Simulation started successfully.');
                    obj.isSimulationRunning = true;
                else
                    % Handle failure to start simulation with the return code
                    disp(['Failed to start simulation. Return code: ', num2str(returnCode)]);
                end
            else
                % No connection available to start the simulation
                disp('No active connection to start the simulation.');
            end
        end

        % Stop the simulation in CoppeliaSim
        function stopSimulation(obj)
            % Ensure there is an active connection before stopping the simulation
            if obj.clientID > -1
                % Stop the simulation in blocking mode and get the return code
                returnCode = obj.sim.simxStopSimulation(obj.clientID, obj.sim.simx_opmode_blocking);
                
                % Check if the simulation stopped successfully
                if returnCode == obj.sim.simx_return_ok
                    disp('Simulation stopped successfully.');
                    obj.isSimulationRunning = false;
                else
                    % Handle failure to stop simulation with the return code
                    disp(['Failed to stop simulation. Return code: ', num2str(returnCode)]);
                end
            else
                % No connection available to stop the simulation
                disp('No active connection to stop the simulation.');
            end
        end

        % Retrieve handle for an object in CoppeliaSim
        function [returnCode,handle] = getHandle(obj, objectName,uiFigure)
            % Get the handle for the specified object in blocking mode
            [returnCode, handle] = obj.sim.simxGetObjectHandle(obj.clientID, objectName, obj.sim.simx_opmode_blocking);
            
            % Check if the handle was retrieved successfully
            if returnCode == obj.sim.simx_return_ok
                disp(['Handle for ', objectName, ' retrieved successfully.']);
            else
                errorMessage = sprintf('Failed to retrieve handle for %s. Return code: %d', objectName, returnCode);
                uialert(uiFigure, errorMessage, 'Handle Retrieval Error', 'Icon', 'error');
            end
        end
    end
end
