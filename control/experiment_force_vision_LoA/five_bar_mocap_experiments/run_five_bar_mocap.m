%% Validating force sensor attitude control with motion capture
% Robot is both searching for CoM AND performing attitude correction
% IN MOTION CAPTURE, and recording force data for comparison

%% **** MAKE SURE YOU HAVE RUN OPTITRACK_INIT BEFORE RUNNING THIS CODE ****
clc; close all; clearvars -except optiClient
addpath("Utilities\", 'Optitrack\');
warning('off','serialport:serialport:ReadWarning')

% Open SerialPort 
s1 = serialport('COM4',9600,'DataBits',8,'StopBits',1); % /dev/ttyUSB0
s1.Timeout = .15;

% Get rigidBody data ONCE to initialize robot+payload AND to check that Optitrack has initialized
optitrack_stream_getFrame

%% **** Robot and Force Sensor Definition ****
IDs = 6;       % ACTIVE AGENTS (LEADER MUST BE FIRST ELEMENT)
N = length(IDs);   % Number of robots
% leader = 4;           % Robot with active force sensing (Leader)
for i=1:N
    robots(i)           = Robot(IDs(i));
    force_sensors(i)    = ForceSensor(IDs(i));
end

%% Optitrack Connection
% optitrack_init

%% Generate figure - Used to send STOP command
f1 = figure("Position",[600 300 800 500]); hold on;
axis equal; xlim([-2 2]); ylim([-2 2]);
robot_vertices = [-1 1 0; 0 0 -1.7];
p1      = patch(robot_vertices(1,:), robot_vertices(2,:), 'k');
q1      = quiver(0,0,0,1, 'LineWidth', 2);
q2      = quiver(0,0,robots(1).negBound(1),robots(1).negBound(2),'LineWidth',2);
q3      = quiver(0,0,robots(1).posBound(1),robots(1).posBound(2),'LineWidth',2);
stop    = uicontrol('style', 'toggle', 'string', 'stop');
% For animation playback
frameCount = 1;
global DIRECTION

%% Optitrack global vars
global opti_T opti_theta

%% Initialize data collection: motion capture rotation, robot commands, actual robot motion, & force sensor feedback
global data_time data_payload_pose data_robot_command data_robot_pose data_sensor 
data_time           = [];
data_payload_pose   = [];
data_robot_pose     = [];
data_robot_command  = [];
data_sensor         = [];
%% Stopwatch and Timer
t           = 0;
t0          = tic;
RUN_TIME    = 200;


%% ______________   RUN EXPERIMENT   ________________

while (t < RUN_TIME  &&  ~robots(1).stop)
    if ~ishandle(f1) || stop.Value
        break
    end

    % Read rigid body from optitrack and return Transform and Rotation Data
    optitrack_stream_getFrame;
    
    % TEMP DELETE (for use without mocap for testing)
    % opti_theta = [pi/4; pi/4];
    % opti_T(1:4,1:4,1) = eye(4);
    % opti_T(1:4,1:4,2) = eye(4);

    % Update ALL robots actions
    [robots, force_sensors] = update_robots_mocap(s1, t, N, robots, force_sensors);
    
    % Send serial communication to robot
    for i=1:N
        SerialCommunication(s1, robots, 192, 'u');
    end
    
    % Update quiver and 'robot' in plot
%     delete(p1)
%     robot_vertices = Rz2(opti_theta(1))*[-1 1 0; 0 0 -1.7];
%     p1 = patch(robot_vertices(1,:), robot_vertices(2,:), 'k');
%     uistack(p1, 'bottom')

    set(q1, 'UData', DIRECTION(1), 'VData', DIRECTION(2));
    set(q2, 'UData', robots(1).negBound(1), 'VData', robots(1).negBound(2));
    set(q3, 'UData', robots(1).posBound(1), 'VData', robots(1).posBound(2));
    % For animation playback
    M(frameCount) = getframe;
    % For data collection
    data_robot_pose = [data_robot_pose; opti_T(1,4,1) opti_T(2,4,1) opti_theta(1)];
    data_payload_pose = [data_payload_pose; opti_T(1,4,2) opti_T(2,4,1) opti_theta(2)];
    data_time = [data_time; t];
    
    frameCount = frameCount + 1;
    pause(0.08)
    t = toc(t0);
end

% Get Final Run Time
disp(['Final run time: ', num2str(round(t,1))]);

%% Cleanup - Stop robot motors, delete serialport
close all;

robots.u.data = zeros(6,1);
for i=1:N
    SerialCommunication(s1, robots, 192, 'u');
end
clear s1
disp("Deleted Serial Port")

%% Record experiment - ********* REMEMBER TO UPDATE FILE NAME TO AVOID OVERWRITING *********
save('05-20-24-five_bar_mocap_com_trial8.mat',...
    'data_time', 'data_payload_pose', 'data_robot_pose', 'data_sensor', ...
    'data_robot_command', 'M', 'frameCount');

return

%% Playback animation of plot if desired
movie(M,frameCount)



