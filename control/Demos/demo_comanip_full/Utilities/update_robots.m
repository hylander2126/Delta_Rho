function [robots, force_sensors] = update_robots(s1, N, robots, force_sensors)

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

% global PD_data;

u_r = [0;0;0]; % No robot motion unless updated explicitly


%% ******** Initialization, Calibration, & Experiment *********
iter1 = 1;
data = zeros([N 2]);

while force_sensors(end).init
    % s1.Timeout = 0.5; % Increase timeout during initialization ONLY
    % Read force sensor, initialize robot and sensor baseline
    for i=1:N
        force_sensors(i) = force_sensors(i).readSensor(s1, robots(i));
        [force_sensors(i), iter1] = initialization(force_sensors(i), iter1);
    end
end


%% Cancel force sensor reading
for i=1:N
    % Read force sensor
    force_sensors(i) = force_sensors(i).readSensor(s1, robots(i));

    %% TEMP - Have programmed robots to do everything onboard, now just gathering data
    temp = force_sensors(i).alpha; % 'alpha' refers simply to data returned by robot, defined in firmware
    temp = temp/100; % Uncomment for EE position (sent in FW as *100)
    temp
    return
    %%

    % Run kinematics
    force_sensors(i) = force_sensors(i).kinematics();
    
    %% FOR NOW, JUST ASSUME TWO ROBOTS
    force = force_sensors(i).force;
    direc = force_sensors(i).direction; % /norm(force_sensors(i).direction);
    d = [direc(2); direc(1)];

    % u_r = [robots(i).Kp .* d; 0]
    u_r = 10*[d; 0];

    % "Calm down" in the forward/reverse direction
    % u_r(1) = u_r(1)*0.8

    % robots(i) = robots(i).setTorques(u_r);
end

% data.direction = [data.direction(2); data.direction(1)];
% u_r = [robots.Kp .* data.direction/norm(data.direction); 0];


%% Set new robot motor values
% for i=1:N
%     robots(i).setTorques(u_r(:,i));
% end

end
