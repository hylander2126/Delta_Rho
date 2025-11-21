function [Robot, five_bar] = update_robot(t,Robot,five_bar,s1)

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
%                  O   *---O---> x

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
    % disp(five_bar.calib)

% If still initializing, skip rest of updating
if Robot.initialize
    return
end
% Move forward for 1 second
if t < 2.5
    u_r = [170;0;0];
    Robot.d_prev = u_r(1:2); % /norm(u_r(1:2));
    five_bar = sensor_kinematics(five_bar);
    Robot.f_initial = [-five_bar.direction(2); -five_bar.direction(1)];

%% Run CoM search
else
    % Get the force feedback
    five_bar = sensor_kinematics(five_bar);
    % Align axes to robot's frame
    five_bar.direction = [-five_bar.direction(2); -five_bar.direction(1)];

    % Get dt since previous step
    dt       = t - Robot.t_prev;
    


    % Set u_r through PD function
    com_search;


    % u_r = [0;0;0];

%% Null-space control / attitude adjustment (Rotate in SAME direction y-force is measured)
% Record initial force reading, and maintain that vector
    e_th         = getAngle(five_bar.direction, Robot.f_initial, 'r');
    e_th_dot     = (e_th - Robot.e_th_prev)/dt;
    Robot.I_e_th = Robot.I_e_th + e_th*dt;
    
    % Catch if force is removed for whatever reason
    if abs(five_bar.force) < five_bar.deadzone
       u_r(3) = 0;
    else% 450 45 20 for circle % 200 15 20 for poly
        u_r(3) = 200*e_th + 15*e_th_dot + 20*Robot.I_e_th; % Need about 80 to turn motor slowly. 600, 20 good gain values
    end

    PD_data = [PD_data; t,  f_, u_, e_th] ; %u_r(3)];

    %% STOP CONDITION
    if abs(e) < 0.035
        Robot.low_e_ctr = Robot.low_e_ctr + 1;
    else
        Robot.low_e_ctr = 0;
    end

    if Robot.low_e_ctr >= 5
        Robot.stop = 1;
    end

end


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
