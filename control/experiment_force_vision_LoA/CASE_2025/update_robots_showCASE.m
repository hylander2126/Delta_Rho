function [robots, force_sensors] = update_robots_showCASE(s1, N, robots, force_sensors)
% CASE 2025
% RECALL AGENT AND SENSOR FRAMES:
%                           
%                      ^ y
%                      |
%         _      O     *-->x O  <- Marker 3 (2)
%         |       \         / 
%         |        \       /
%      2d |         \     /
%         |          \   /
%         |           \ /
%         _            O        <- Marker 1 (1)
%                |<--------->|
%                      d
%
%                      O p5
%                     / \
%                    /   \
%                l3 /     \ l4
%                  /       \
%                 /    ^ y  \
%                 \    |    /
%                  O   *---O--> x

    global opti_theta

    u_r = [0;0;0];
    u_r_att = [0;0;0];

    %% ******** Let's separate CoM from Attitude Control (for debugging) *********
    net_rotation = opti_theta(1) - opti_theta(2); %robots(1).baseline_rot - opti_theta(2); % Change from prev baseline (updated in attitud controller)
    
    if ~robots(1).att_status
        % If net rotation is below a threshold (wheels not colliding), push in straight desired line
        if abs(net_rotation) < deg2rad(25)
            % Let's align this CoM vector with our desired motion vector             
            LoA = [0; 1]; % CoM Pos 0
            % LoA = [-0.2; 1]; % CoM Pos 1
            % LoA = [0.2; 1]; % CoM Pos 2

            e = 0;

            % e = getAngle2(Rz2(opti_theta(1))*LoA, desired_direction);
            % K = 0.8;
            % u = unitize(K*Rz2(e) * LoA);
            
            u_r = 2900*[e/100; LoA];
            u_r = 4500*[-0.0001; LoA];

        % ... otherwise correct the attitude.
        else
            robots(1).att_status = 1; % Attitude needs corrected
            % u_r = [0;0;0];
            % 
            % robots.u.data = zeros(6,1);
            % for n=1:N
            %     SerialCommunication(s1, robots(n), 192, 'u');
            % end
            % pause(1)
        end
    end
    
    if robots(1).att_status
        [robots, u_r_att] = controllerAttitude(robots, N, s1, opti_theta, u_r);
        % u_r = 2900*[0; -0.07; 1];
    end

    % This worked for box payload
    % u_r = [u_r(1); -u_r(2); u_r(3)]; 
    % u_r = u_r - u_r_att; % was plus uratt. Box for some reason is flipped...

    u_r = u_r - u_r_att;

    %% Set new robot motor values
    robots = robots.setTorques(u_r);
end
