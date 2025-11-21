classdef Robot
    properties
        % Static properties
        ID          % Robot number (same as labeled and XBee)
        stop        % Stop condition
        Kp          % Proportional Gain
        Kd          % Derivative Gain
        Kw          % Rotational Proportional Gain
        % Agent motor values
        u           % Action (input)
        u_r         % Reference (desired)
        % Robot Jacobian (new wheels 02/11/23) 
        J_r
        % Robot PD control parameters
        t_prev
        e_prev
        e_th_prev
        I_e_th      % Integral term for attitude correction
        f_initial   % Initial force reading for attitude correction
        low_e_ctr
        f_prev
        d_prev      % Robot driving direction
        % Dummy field for switching mode
        empty
    end

    methods
        function obj = Robot(ID)
            % Constructor
            obj.ID          = ID;
            obj.stop        = false;
            obj.Kp          = 140; % 170   
            obj.Kd          = 0.5;    
            obj.Kw          = 150;    
            obj.u.data      = zeros(6,1);
            obj.u.type      = 'uint8';
            obj.u.convertor = @(x) uint8(x);
            obj.u_r         = [0; 0; 0];
            obj.J_r         = construct_robot_jacobian(); % Assuming this is a function you have defined
            obj.t_prev      = 0;
            obj.e_prev      = 0;
            obj.e_th_prev   = 0;
            obj.I_e_th      = 0;
            obj.f_initial   = [];
            obj.low_e_ctr   = 0;
            obj.f_prev      = 0;
            obj.d_prev      = [0; 0];
        end

        %% Set motor torques using calculated desired motion (u_r)
        function obj = setTorques(obj, u_r)
            % Calculate motor torque values using ROBOT Jacobian
            f = obj.J_r * u_r;
        
            % Assign robot motor torques
            % k=1 front left; k=2 rear; k=3 front right
            for k = 1:3
                if (f(k) >= 0)
                    obj.u.data(2*k - 1, 1) = 0;
                    obj.u.data(2*k, 1) = uint8(abs(f(k)));
                else
                    obj.u.data(2*k - 1, 1) = uint8(abs(f(k)));
                    obj.u.data(2*k, 1) = 0;
                end
            end
        end

    end
end
