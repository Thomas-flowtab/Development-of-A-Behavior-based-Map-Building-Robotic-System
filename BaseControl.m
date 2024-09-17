classdef BaseControl
    properties
        wheels       % Array of Wheel objects
        allSpeed     % Speed value for all wheels
        moveButtonSelected  % Tracks which button is pressed (for movement direction)
        WheelMovements      % Struct for movement directions (e.g., FORWARD, BACKWARD, etc.)
    end
    
    methods
        % Constructor to initialize wheels
        function obj = BaseControl(wheelHandles)
            obj.wheels = cell(1, 4);
            for i = 1:length(obj.wheels)
                obj.wheels{i} = Wheel(wheelHandles(i));  % Initialize each wheel
            end
            obj.allSpeed = 1.0;  % Default speed
            obj.moveButtonSelected = 0;  % No movement selected
            obj.WheelMovements = struct(...
                'FORWARD', 1, 'BACKWARD', 2, 'LEFT', 3, 'RIGHT', 4, ...
                'CLOCKWISE', 5, 'COUNTER_CLOCKWISE', 6, 'DIAG_FL', 7, ...
                'DIAG_FR', 8, 'DIAG_BL', 9, 'DIAG_BR', 10);
        end
        
        % Method to move the base in a specified direction
        function move(obj, sim, clientID, direction,speed)
            velocity = speed;%obj.allSpeed;  % Use current speed setting
            switch direction
                case obj.WheelMovements.FORWARD
                    obj.setWheelVelocities(sim, clientID, -velocity, -velocity, -velocity, -velocity);
                case obj.WheelMovements.BACKWARD
                    obj.setWheelVelocities(sim, clientID, velocity, velocity, velocity, velocity);
                case obj.WheelMovements.LEFT
                    obj.setWheelVelocities(sim, clientID, -velocity, velocity, velocity, -velocity);
                case obj.WheelMovements.RIGHT
                    obj.setWheelVelocities(sim, clientID, velocity, -velocity, -velocity, velocity);
                case obj.WheelMovements.CLOCKWISE
                    obj.setWheelVelocities(sim, clientID, velocity, -velocity, velocity, -velocity);
                case obj.WheelMovements.COUNTER_CLOCKWISE
                    obj.setWheelVelocities(sim, clientID, -velocity, velocity, -velocity, velocity);
                case obj.WheelMovements.DIAG_FL
                    obj.setWheelVelocities(sim, clientID, -velocity, 0, 0, -velocity);
                case obj.WheelMovements.DIAG_FR
                    obj.setWheelVelocities(sim, clientID, 0, -velocity, -velocity, 0);
                case obj.WheelMovements.DIAG_BL
                    obj.setWheelVelocities(sim, clientID, 0, velocity, velocity, 0);
                case obj.WheelMovements.DIAG_BR
                    obj.setWheelVelocities(sim, clientID, velocity, 0, 0, velocity);
            end
        end
        
        % Method to stop all wheels
        function stop(obj, sim, clientID)
            obj.setWheelVelocities(sim, clientID, 0, 0, 0, 0);
        end
        
        % Method to set velocities for all wheels
        function setWheelVelocities(obj, sim, clientID, vel_fr, vel_fl, vel_rr, vel_rl)
            obj.wheels{1}.setVelocity(sim, clientID, vel_fr);  % Front-right wheel
            obj.wheels{2}.setVelocity(sim, clientID, vel_fl);  % Front-left wheel
            obj.wheels{3}.setVelocity(sim, clientID, vel_rr);  % Rear-right wheel
            obj.wheels{4}.setVelocity(sim, clientID, vel_rl);  % Rear-left wheel
        end
    end
end
