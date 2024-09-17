classdef Kinematics
    properties
        DH_params   % The DH parameters for the robot arm we get it from coppeliasim Dh extractor tool
    end
    
    methods
        % Constructor to initialize the DH parameters
        function obj = Kinematics(DH_params)
            obj.DH_params = DH_params;  % Pass the DH table when creating the object
        end

        % Function to calculate the forward kinematics and return the transformation matrix
        function T = calculateForwardKinematics(obj, jointPositions)
            % Initialize the transformation matrix as an identity matrix
            T = eye(4);  

            % Loop through each joint and calculate its transformation matrix
            for i = 1:size(obj.DH_params, 1)
                theta = deg2rad(jointPositions(i));  % Convert joint angle to radians
                d = obj.DH_params(i, 2);   % Link offset (d)
                a = obj.DH_params(i, 3);   % Link length (a)
                alpha = deg2rad(obj.DH_params(i, 4));  % Twist angle (alpha)

                % Build the transformation matrix for this joint
                T_i = [
                    cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
                    sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                    0,          sin(alpha),             cos(alpha),            d;
                    0,          0,                      0,                     1
                ];

                % Multiply the transformation matrices
                T = T * T_i;
            end
        end
        
        % Function to extract the end-effector position from the transformation matrix
        function endEffectorPos = getEndEffectorPosition(obj, jointPositions)
            % Calculate the transformation matrix
            T = obj.calculateForwardKinematics(jointPositions);

            % Extract the end-effector position (X, Y, Z coordinates)
            endEffectorPos = T(1:3, 4);  % Return the x, y, z position
        end
    end
end