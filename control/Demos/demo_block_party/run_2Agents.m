clear; clc; close all;
addpath("Utilities\");
global s1
if exist('s1','var')
    delete(s1)
end

%% Open SerialPort
s1 = serialport('COM4',9600,'DataBits',8, 'StopBits',1);
% s1 = serialport('/dev/tty.usbserial-DA01HMES',9600,'DataBits',8,'StopBits',1);
% fopen(s1);


% Agent attachment points (UNUSED)
% theta = [0 90 135 225 315];
% P = 160*[cosd(theta);sind(theta)];


%% *******  Define Agents and Properties  **********
N = 6;                  % Number of Agents
K_max = 200;             % Proportional Gain

actuation_mode = 1;     % 1==automated 'dance'; 2==keyboard; 3==xbox controller

% J_r = getRobotJacobian; % Calculate robot (local) jacobian
J_r = construct_robot_jacobian;
J_inv = [-J_r(:,1:2), J_r(:,3)]; % Not really inverted Jacobian, just inverting the translation components.


if actuation_mode ~= 3
    global agent
end

for n=1:N
    % Define Agent
    agent(n).ID = n;
    agent(n).K = K_max;
    agent(n).u.data = zeros(6,1);
    agent(n).u.type = 'uint8';
    agent(n).u.convertor = @(x)uint8(x);
    agent(n).stop = false;
    agent(n).u_r = [0;0;0];
    agent(n).J = J_r;
end

%% TO MIRROR CONTROLS:
agent(6).J = J_inv;


switch(actuation_mode)
    case 1          % Automated 'dance'
        tic
    case 2          % Keyboard Control
        figure('KeyPressFcn',@getKey);
        drawnow();
    case 3          % Xbox Control
        id = 1;
        joy = vrjoystick(id,'forcefeedback');
end

disp("Beginning control. Press Esc button to exit...")


%% ********  Run Experiment  *********
%%_________________________________________________________________________
while ~agent(1).stop
    pause(0.1);

    switch(actuation_mode)
        case 1
            dance_script;
        case 3
            getJoystick;
    end

    % Send motor commands to robots
    for n=1:N
        SerialCommunication(s1,agent(n),192,'u'); % Send actuator command (192) to robot i containing data 'robot.u'
    end
end

%% When finished running, stop robots
for n=1:N
    agent(n).u.data = zeros(6,1);
    SerialCommunication(s1,agent(n),192,'u');
end
delete(s1)
disp("Deleted Serial Port")
close all
