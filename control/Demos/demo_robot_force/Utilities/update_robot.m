function [Robot, five_bar] = update_robot(t,Robot,leader,five_bar,s1)

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

global PD_data;

% No robot motion unless updated explicitly
u_r = [0;0;0];


%% Get force data from sensors
sensor = cell2mat(SerialCommunication(s1,Robot,80))';
% If timeout error, continue moving
if isempty(sensor)
    return
else
    sensor(end) = []; % Remove last element, unused
end
data = [mapfun(sensor(1), 0, 1024, 0, deg2rad(330)) , mapfun(sensor(2), 0, 1024, 0, deg2rad(330))]; % map volts to rads
data = data - five_bar.offset; % Apply offset to get correct relative readings

% Set the five bar current base angles
five_bar.alpha = reshape(data, [2,1]); % WILL BE 'WRONG' UNTIL INIT IS FINISHED ~0.5s


%% ******** Initialization, Calibration, & Experiment *********
initialize_robot;

% If still initializing, skip rest of updating
if Robot.initialize
    return
end

% Cancel force sensor reading
five_bar = sensor_kinematics(five_bar);
five_bar.direction = [five_bar.direction(2); five_bar.direction(1)];
u_r = [Robot.Kp .* five_bar.direction/norm(five_bar.direction); 0];


%% Calculate motor torque values using ROBOT Jacobian
f = Robot.J_r*u_r;

%% Assign robot motor torques | k=1 front left; k=2 rear; k=3 front right
for k=1:3
    if(f(k) >= 0)
        Robot.u.data(2*k-1,1) = 0;
        Robot.u.data(2*k,1) = uint8(abs(f(k)));
    else
        Robot.u.data(2*k-1,1) = uint8(abs(f(k)));
        Robot.u.data(2*k,1) = 0;
    end
end

%% SEND serial communication to robot
SerialCommunication(s1,Robot,192,'u');

end
