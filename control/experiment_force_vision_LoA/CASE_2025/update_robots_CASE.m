function [robots, force_sensors] = update_robots_CASE(s1, N, robots, force_sensors)
% CASE 2025
% RECALL AGENT AND SENSOR FRAMES:
%                           
%                      ^ x
%                      |
%         _      O     *-->y O  <- Marker 3 (2)
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
        % If net rotation is below a threshold (wheels not colliding), continue CoM search.
        if abs(net_rotation) < deg2rad(25)
            [robots, u_r] = controllerCoM(robots, opti_theta, u_r);
    
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
    end
    u_r = [u_r(1); -u_r(2); u_r(3)];
    u_r = u_r - u_r_att; % was plus uratt. Box for some reason is flipped...

    % try flipping x component of ur
    

    %% Set new robot motor values
    robots = robots.setTorques(u_r);
end
