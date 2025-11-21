clc; clear; close all;
addpath("Utilities\");

%% Open SerialPort 
s1 = serialport('COM4',9600,'DataBits',8,'StopBits',1); % COM7
s1.Timeout = .15;


%% **** Robot and Force Sensor Definition ****
IDs = [5];       % ACTIVE AGENTS (# on Xbee module)
N = length(IDs);   % Number of robots
% leader = 4;           % Robot with active force sensing (Leader)
for i=1:N
    robots(i) = Robot(IDs(i));
    % force_sensors(i) = ForceSensor(IDs(i));
end


%% Generate figure - Used to send STOP command
f1 = figure("Position",[300 300 400 400]); axis equal;
patch([-1 1 0], [0 0 -1.7], [0.902, 0.62, 0.98]);
hold on;
q1 = quiver(0,0,0,1, 'LineWidth', 2);
xlim([-2 2]); ylim([-2 2]);
stop = uicontrol('style', 'toggle', 'string', 'stop');

% Stopwatch and master index/counter
t                   = 0;
t_start             = tic;
run_time            = 120;
masterIdx           = 1;


%% ---------------   RUN EXPERIMENT   ----------------
while (t < run_time  &&  ~robots(1).stop)
    if (~ishandle(f1) || stop.Value)
        break
    end

    % Update ALL robots actions based on sensor data
    [robots] = update_robots(s1, N, robots);
    
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
    SerialCommunication(s1,robots(IDs(i)),192,'u');
end
delete(s1)
disp("Deleted Serial Port")

% Get Final Run Time
t_final = round(toc(t_start),1);
disp(['Final run time: ', num2str(t_final)]);

return
