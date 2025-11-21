clc; clear; close all;
useOptiTrack = false;
tic
addpath('Utilities\');

%% Initialize Optitrack and XBee Serial connection
if useOptiTrack
    if exist("theClient",'var') || exist("frameOfData",'var') % Check Optitrack connection is closed
        disconnect_optitrack;
    end
    init_optitrack;
end

s1 = serialport('COM4',9600,'DataBits',8, 'StopBits',1);

%% Define parameters and initialize robot properties
N = 3; % Number of robots
exit = 0;
K = 100; % Gain
num = 1;
u_r = [0;0;0];

u_data = zeros(6,1);
u_type = 'uint8';
u_converter = @(x)uint8(x);

for i=1:N
    A(i) = robot('N',N,'K',K,'num',num,'u_r',u_r,'exit',exit,'ID',i,...
        'u_type',u_type,'u_data',u_data,'u_converter',u_converter);
end

%% Set agent's direction and velocity (vel ~ K)
figure('KeyPressFcn',{@getKey,A});
drawnow();

%% Send info to each robot via serial comms.
while(~A(1).exit)
    pause(0.1);
    time = round(toc);
   
    % Get object's rotation as a rigid body from Optitrack
    if useOptiTrack
        read_rigidbodys_frame_optitrack
%         disp(Translation)
        disp(Markers)
    end

    % For each robot
    for i=N:N
%         updateHeading(A(i),time)
        % Send actuator command 192 to all robots containing that robot's
        % 'robot.u' information (which is updated in the getKey fn.)
        SerialCommunication(s1,A(i),192,'u');
    end
end
disp("Control Successfully Exited!")

%% After exit command, STOP all robot motions
for i=1:N
    A(i).u_data = zeros(6,1);
    SerialCommunication(s1,A(i),192,'u');
end
clear s1

%% Disconnect Optitrack
disconnect_optitrack;
close all
