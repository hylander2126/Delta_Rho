%% Demo for co-manipulation with human-in-the-loop
% Define which RobotID's are being utilized (two robots for now)
clc; clear; close all;
addpath("Utilities\");


%% Open SerialPort 
s1 = serialport('COM4',9600,'DataBits',8,'StopBits',1); % /dev/ttyUSB0
s1.Timeout = .15;

%% **** Robot and Force Sensor Definition ****
global opti_theta
opti_theta(1:2,1) = 0;

IDs = [6];       % ACTIVE AGENTS (LEADER MUST BE FIRST ELEMENT)
N = length(IDs);   % Number of robots
% leader = 4;           % Robot with active force sensing (Leader)
for i=1:N
    robots(i) = Robot(IDs(i));
    force_sensors(i) = ForceSensor(IDs(i));
end


%% Generate figure - Used to send STOP command
f1 = figure("Position",[600 300 800 500]); hold on;
axis equal;
xlim([-2 2]); ylim([-2 2]);
patch([-1 1 0], [0 0 -1.7], 'k')
q1 = quiver(0,0,0,1, 'LineWidth', 2);
stop = uicontrol('style', 'toggle', 'string', 'stop');


%% Stopwatch for checking minimal rotation threshold
global t;
t = 0;
t_start = tic;
run_time = 20;
frameCount = 1; % For animation playback
global DIRECTION data_sensor
data_sensor = [0, 0];












%% **************** TEMP DELETE ************************
switchMode(6, s1);












%% ---------------   RUN EXPERIMENT   ----------------

while (t < run_time  &&  ~robots(1).stop)
    if ~ishandle(f1) || stop.Value
        break
    end

    % Update ALL robots actions based on sensor data
    [robots, force_sensors] = update_robots(s1, N, robots, force_sensors);
    % Send serial communication to robot
    for i=1:N
        SerialCommunication(s1, robots(i), 192, 'u');
    end
    
    set(q1, 'UData', DIRECTION(1), 'VData', DIRECTION(2));

    pause(0.5)
    t = toc(t_start);
    M(frameCount) = getframe;
    frameCount = frameCount + 1;
end

close all;

% Get Final Run Time
t_final = round(toc(t_start),1);
disp(['Final run time: ', num2str(t_final)]);

%% Cleanup - Stop robot motors, delete serialport
for i=1:N
    robots(i).u.data = zeros(6,1);
    SerialCommunication(s1, robots(i), 192, 'u');
end

pause(0.5)
clear s1
disp("Deleted Serial Port")

return

%% For plotting sensor data over time
n = size(data_sensor,1);
cmap = jet(n);
alphas = linspace(0.15, 1, n);

figure; hold on; grid;
xlim([-20 20]); ylim([-35 15]);

for i = 1:n-1
    segColor = cmap(i, :); % interp1(1:n, cmap, i+0.5, 'linear');
    line(data_sensor(i:i+1, 1), data_sensor(i:i+1, 2), 'Color', [segColor alphas(i)], 'LineWidth', 3);
    hold on;
    s = scatter(data_sensor(i,1), data_sensor(i,2), 15, segColor, 'filled');
    s.MarkerFaceAlpha = alphas(i);
end

colorbar;
colormap(jet);

