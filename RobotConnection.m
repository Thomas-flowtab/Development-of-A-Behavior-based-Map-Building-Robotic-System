classdef RobotConnection < handle
    properties
        clientID      % Client ID for the connection to CoppeliaSim
        sim          % Instance of the remote API (sim)
        isSimulationRunning = false; % Flag to check if the simulation is running
         
    end

    methods
        % Constructor
        function obj = RobotConnection()
            % Initialize the remote API
            obj.sim = remApi('remoteApi');
            obj.clientID = -1; % Initialize clientID to an invalid state
        end

        % Connect to CoppeliaSim
        function success = connect(obj)
            % Close any previous connections
            obj.sim.simxFinish(-1);
            
            % Attempt to connect to CoppeliaSim on localhost and port 19997
            obj.clientID = obj.sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
            
            if obj.clientID > -1
                disp('Connected to CoppeliaSim.');
                obj.startSimulation();
                success = obj.isSimulationRunning;
            else
                disp('Failed to connect to CoppeliaSim.');
                success = false;
            end
        end

        % Disconnect from CoppeliaSim
        function disconnect(obj)
            if obj.clientID > -1
                obj.stopSimulation();
                obj.sim.simxFinish(obj.clientID);  % Close the connection
                disp('Disconnected from CoppeliaSim.');
                obj.clientID = -1; % Reset client ID
            else
                disp('No active connection to disconnect.');
            end
        end

        % Start the simulation
        function startSimulation(obj)
            if obj.clientID > -1
                returnCode = obj.sim.simxStartSimulation(obj.clientID, obj.sim.simx_opmode_blocking);
                if returnCode == obj.sim.simx_return_ok
                    disp('Simulation started successfully.');
                    obj.isSimulationRunning = true;
                else
                    disp(['Failed to start simulation. Return code: ', num2str(returnCode)]);
                end
            else
                disp('No active connection to start the simulation.');
            end
        end

        % Stop the simulation
        function stopSimulation(obj)
            if obj.clientID > -1
                returnCode = obj.sim.simxStopSimulation(obj.clientID, obj.sim.simx_opmode_blocking);
                if returnCode == obj.sim.simx_return_ok
                    disp('Simulation stopped successfully.');
                    obj.isSimulationRunning = false;
                else
                    disp(['Failed to stop simulation. Return code: ', num2str(returnCode)]);
                end
            else
                disp('No active connection to stop the simulation.');
            end
        end

        % Retrieve handle for an object in CoppeliaSim
        function [handle, returnCode] = getHandle(obj, objectName)
            [returnCode, handle] = obj.sim.simxGetObjectHandle(obj.clientID, objectName, obj.sim.simx_opmode_blocking);
            if returnCode == obj.sim.simx_return_ok
                disp(['Handle for ', objectName, ' retrieved successfully.']);
            else
                disp(['Failed to retrieve handle for ', objectName, '.']);
            end
        end
    end
end
