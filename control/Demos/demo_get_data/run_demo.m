%% Demo for co-manipulation with human-in-the-loop
% Define which RobotID's are being utilized (two robots for now)
clc; clear; close all;
addpath("Utilities\");


%% Open SerialPort 
s1 = serialport('COM3',9600,'DataBits',8,'StopBits',1);
s1.Timeout = .15;


%% **** Robot and Force Sensor Definition ****
IDs = [4];       % ACTIVE AGENTS (LEADER MUST BE FIRST ELEMENT)
N = length(IDs);   % Number of robots
% leader = 4;           % Robot with active force sensing (Leader)
for i=1:N
    robots(i) = Robot(IDs(i));
    force_sensors(i) = ForceSensor(IDs(i));
end


%% Generate figure - Used to send STOP command
f1 = figure("Position",[600 300 800 500]); hold on;
stop = uicontrol('style','toggle','string','stop');


%% Stopwatch for checking minimal rotation threshold
t = 0;
t_start = tic;
run_time = 120; % 5 mins


%% ---------------   RUN EXPERIMENT   ----------------

while (t < run_time  &&  ~robots(1).stop && ~stop.Value)
    % Update ALL robots actions based on sensor data
    [robots, force_sensors] = update_robots(s1, N, robots, force_sensors);
    
    % Send serial communication to robot
    for i=1:N
        SerialCommunication(s1, robots(i), 192, 'u');
    end

    pause(0.05)
    t = toc(t_start);
end

%% --------------------------------------------------


%% Cleanup - Stop robot motors, delete serialport
robots.u.data = zeros(6,1);
for i=1:N
    SerialCommunication(s1,robots(i),192,'u');
end
delete(s1)
disp("Deleted Serial Port")

% Get Final Run Time
t_final = round(toc(t_start),1);
disp(['Final run time: ', num2str(t_final)]);

return
