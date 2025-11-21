function [robots, u_r_att] = controllerAttitude(robots, N, s1, opti_theta, u_r)
        %% ________________  ATTITUDE CORRECTION  ________________
        % u_r = [0; 0; 3500];
        % Error is simply desired angle (zero 0) minus current angle difference between robot (rigidbody 1) and payload (2)
        e_r = 0 - (opti_theta(1) - opti_theta(2));
        % Integral Term
        robots(1).I_e_th = robots(1).I_e_th + e_r;
        % Derivative Term
        de_r = e_r - robots(1).e_th_prev;
        robots(1).e_th_prev = e_r;
        % Control Law
        u_r_att = [0;0;0];
        u_r_att(1) = 60*e_r + 4*robots(1).I_e_th + 1*de_r;
        
        % If attitude error is small, set robot attitude status to 'corrected' = 0
        if abs(e_r) < deg2rad(5)
            robots(1).att_status = 0;
            robots(1).baseline_rot = opti_theta(2);
            u_r_att = [0;0;0];

            robots(1).prev_pay_th = opti_theta(2); % Make sure to update to avoid unexpected CoM estimate 'jumps'
            
            robots.u.data = zeros(6,1);
            for i=1:N
                SerialCommunication(s1, robots, 192, 'u');
            end
        end
        
    end