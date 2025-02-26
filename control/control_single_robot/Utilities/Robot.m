classdef Robot
    properties
        % Static properties
        ID          % Robot number (same as labeled and XBee)
        stop        % Stop condition
        % Agent motor values
        u           % Action (input)
        u_r         % Reference (desired)
        % Robot Jacobian 
        J_r
        % Robot PID control parameters
        Kp          % Proportional Gain
        Kd          % Derivative Gain
        Ki          % Integral Gain
        negBound    % Clockwise-most bound
        posBound    % Counter-Clockwise-most bound
        init_negB   % INITIAL negative bound
        init_posB   % INITIAL positive bound
        t_prev      % Previous timer value
        e_prev      % Previous angular velocity error
        prev_pay_th  % Previous angular velocity
        I_e         % Integral term
        % Atttude PID parameters
        Kw          % Rotational Proportional Gain
        e_th_prev   % Previous attitude error
        I_e_th      % Integral term for attitude correction
        f_initial   % Initial force reading for attitude correction
        low_e_ctr
        f_prev
        d_prev      % Robot driving direction
        att_status  % Toggle for attitude correction status 0=corrected 1=uncorrected
        % Dummy field for switching mode
        empty
        % Previous control 'baseline' step values
        baseline_rot % Payload orientation from previous baseline
    end

    methods
        function obj = Robot(ID)
            global opti_theta
            % Constructor
            obj.ID          = ID;
            obj.stop        = false;

            obj.u.data      = zeros(6,1);
            obj.u.type      = 'uint8';
            obj.u.convertor = @(x) uint8(x);
            obj.u_r         = [0; 0; 0];
            obj.J_r         = new_Jr_construct('EE'); % construct_robot_jacobian();

            obj.Kp          = 140; % 170   
            obj.Kd          = 0.5;
            obj.Ki          = 0;
            obj.negBound    = [1; 0.1];
            obj.posBound    = [-1; 0.1];
            obj.init_negB   = [1; 0.1];
            obj.init_posB   = [-1; 0.1];
            obj.t_prev      = 0;
            obj.e_prev      = 0;
            try
                obj.prev_pay_th = opti_theta(2);
            catch
                obj.prev_pay_th = 0;
            end
            obj.I_e         = 0;

            obj.Kw          = 150;
            obj.e_th_prev   = 0;
            obj.I_e_th      = 0;
            obj.f_initial   = [];
            obj.low_e_ctr   = 0;
            obj.f_prev      = 0;
            obj.d_prev      = [0; 0];
            obj.att_status  = 0;
            try
                obj.baseline_rot= opti_theta(2);
            catch
                obj.baseline_rot= 0;
            end
        end

        %% Set motor torques using calculated desired motion (u_r)
        function obj = setTorques(obj, u_r)
            % Calculate motor torque values using ROBOT Jacobian
            f = obj.J_r * u_r;
        
            % Assign robot motor torques
            % k=1 front left; k=2 rear; k=3 front right
            for k = 1:3
                if (f(k) >= 0)
                    obj.u.data(2*k - 1  , 1) = 0;
                    obj.u.data(2*k      , 1) = uint8(abs(f(k)));
                else
                    obj.u.data(2*k - 1  , 1) = uint8(abs(f(k)));
                    obj.u.data(2*k      , 1) = 0;
                end
            end
        end
    end
end
