function [robots, force_sensors] = update_robots_mocap(s1, t, N, robots, force_sensors)

global DIRECTION  data_robot_command data_sensor opti_theta % opti_T 
u_r = [0;0;0]; % No robot motion unless updated explicitly


%% ******** Let's separate CoM from Attitude Control (for debugging)

% Get change in rotation from previous baseline 
net_rotation = robots(1).baseline_rot - opti_theta(2);

% If that net rotation is below a threshold (wheels aren't about to collide), do CoM.
if abs(net_rotation) < deg2rad(50)
    
% ... otherwise correct the attitude.

%% ________________  COM ESTIMATE  ______________________
% Bound convergence (binary search) method from first paper (2023)
% Determine bound orientation update per time step: rotate bounds by payload's orientation
rot_amount = robots(1).prev_pay_th - opti_theta(2);
robots(1).negBound = Rz2(rot_amount) * robots(1).negBound;
robots(1).posBound = Rz2(rot_amount) * robots(1).posBound;

robots(1).prev_pay_th = opti_theta(2); % Set previous payload orientation to current

% Now update either bound based on payload rotation direction
if abs(rot_amount) > 0.04
    % Calculate the angle between bounds to 
    bound_angle = abs(getAngle(robots(1).negBound, robots(1).posBound, 'r'));
    if rot_amount > 0
%         disp('clockwise rotation!!')
        robots(1).negBound = Rz2(bound_angle/2) * robots(1).negBound;
    elseif rot_amount < 0
%         disp('counter clockwise rotation!!')
        robots(1).posBound = Rz2(-bound_angle/2) * robots(1).posBound;
    end
end

% Determine new motion direction in GLOBAL FRAME (this may be the issue with force feedback method...)
direction = robots(1).negBound + robots(1).posBound;
direction = direction / norm(direction);
% Convert global control input to local robot input
u = Rz2(-opti_theta(1)) * direction;
% Assign to robot control input
u_r(2:3) = 3500 * u;


%% PID method - will need to be tuned for every payload & CoM change ...
% default_direction = [0;3500]; % Initial direction is straight fwd. This is rotated as CoM converges
% % Error is angular velocity of payload (should be zero).
% e = 0 - (opti_theta(2) - robots(1).omega_prev); % Time is constant, can be eliminated from calculation.
% robots(1).omega_prev = e;
% % Integral Term
% robots(1).I_e = robots(1).I_e + e;
% % Derivative Term
% de = e - robots(1).e_prev;
% robots(1).e_prev = e;
% % Control Law
% u = 0.1*e + 0.01*robots(1).I_e + 0.01*de % This will output an angle
% % u = e + robots(1).I_e + de
% u_r(2:3) = Rz2(u) * default_direction;


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
u_r(1) = 60*e_r + 4*robots(1).I_e_th + 1*de_r;

% Attitude controller and CoM estimate fight each other, let's calm down
% the estimate if attitude error is too large.
% rad2deg(e_r)
% if abs(rad2deg(e_r)) > 15
%     u_r(2:3) = u_r(2:3)/2;
% end


for i=1:N
    % Read force sensor
    force_sensors(i) = force_sensors(i).readSensor(s1, robots(i));
    % Data returned by robot, defined in firmware
    temp = force_sensors(i).alpha;
    DIRECTION = temp(1:2)/(norm(temp(1:2))); % for plot
end


%% Set new robot motor values
for i=1:N
    robots(i) = robots(i).setTorques(u_r(:,i));
end

data_robot_command = [data_robot_command; u_r(1) u_r(2) u_r(3)];
data_sensor = [data_sensor; temp(1) temp(2)];

end