classdef RobotConnection < handle
    % RobotConnection Class to manage the connection between MATLAB and CoppeliaSim.
    %
    % This class handles establishing and managing a remote connection to CoppeliaSim,
    % starting and stopping simulations, and retrieving object handles within the simulation.
    % It utilizes the CoppeliaSim remote API to perform operations such as connecting, 
    % disconnecting, and controlling the simulation environment.
    
    properties
        clientID      % Client ID for the connection to CoppeliaSim
        sim           % Instance of the remote API (sim)
        isSimulationRunning = false; % Flag to check if the simulation is running
    end

    methods
        % Constructor
        function obj = RobotConnection()
            % Initialize the remote API instance for CoppeliaSim.
            % Sets clientID to an invalid state (-1) by default.
            obj.sim = remApi('remoteApi');
            obj.clientID = -1; % Initialize clientID to an invalid state
        end

        % Connect to CoppeliaSim
        function success = connect(obj)
            % Attempt to establish a connection to CoppeliaSim.
            % Closes any previous connection before trying to connect on localhost at port 19997.
            %
            % Returns:
            %   success - A boolean indicating if the connection was successful.

            % Close any previous connections
            obj.sim.simxFinish(-1);
            
            % Attempt to connect to CoppeliaSim on localhost and port 19997
            obj.clientID = obj.sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
            
            if obj.clientID > -1
                disp('Connected to CoppeliaSim.');
                obj.startSimulation();
                success = obj.isSimulationRunning; % Check if the simulation was started successfully
            else
                disp('Failed to connect to CoppeliaSim.');
                success = false; % Return failure if the connection was not established
            end
        end

        % Disconnect from CoppeliaSim
        function disconnect(obj)
            % Disconnects from the CoppeliaSim server if there is an active connection.
            % Stops the simulation before closing the connection.
            if obj.clientID > -1
                obj.stopSimulation();  % Stop the simulation if it's running
                obj.sim.simxFinish(obj.clientID);  % Close the connection
                disp('Disconnected from CoppeliaSim.');
                obj.clientID = -1; % Reset client ID to an invalid state
            else
                disp('No active connection to disconnect.');
            end
        end

        % Start the simulation
        function startSimulation(obj)
            % Starts the simulation in CoppeliaSim if there is an active connection.
            % Uses a blocking mode to ensure the command completes before returning.
            if obj.clientID > -1
                returnCode = obj.sim.simxStartSimulation(obj.clientID, obj.sim.simx_opmode_blocking);
                if returnCode == obj.sim.simx_return_ok
                    disp('Simulation started successfully.');
                    obj.isSimulationRunning = true; % Update flag to indicate the simulation is running
                else
                    disp(['Failed to start simulation. Return code: ', num2str(returnCode)]);
                end
            else
                disp('No active connection to start the simulation.');
            end
        end

        % Stop the simulation
        function stopSimulation(obj)
            % Stops the simulation in CoppeliaSim if there is an active connection.
            % Uses a blocking mode to ensure the command completes before returning.
            if obj.clientID > -1
                returnCode = obj.sim.simxStopSimulation(obj.clientID, obj.sim.simx_opmode_blocking);
                if returnCode == obj.sim.simx_return_ok
                    disp('Simulation stopped successfully.');
                    obj.isSimulationRunning = false; % Update flag to indicate the simulation has stopped
                else
                    disp(['Failed to stop simulation. Return code: ', num2str(returnCode)]);
                end
            else
                disp('No active connection to stop the simulation.');
            end
        end

        % Retrieve handle for an object in CoppeliaSim
        function [handle, returnCode] = getHandle(obj, objectName)
            % Retrieves the object handle from CoppeliaSim for a given object name.
            % This handle is used to control or query the object within the simulation.
            %
            % Parameters:
            %   objectName - The name of the object in CoppeliaSim to retrieve the handle for.
            %
            % Returns:
            %   handle - The object handle retrieved from the simulation.
            %   returnCode - The return code from the remote API, indicating success or failure.
            
            [returnCode, handle] = obj.sim.simxGetObjectHandle(obj.clientID, objectName, obj.sim.simx_opmode_blocking);
            if returnCode == obj.sim.simx_return_ok
                disp(['Handle for ', objectName, ' retrieved successfully.']);
            else
                disp(['Failed to retrieve handle for ', objectName, '.']);
            end
        end
    end
end
