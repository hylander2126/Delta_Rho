%% Run the baseline experiments using motioncapture 
clc; close all; clearvars -except optiClient
addpath("Utilities\", "Optitrack\");

% ====================================================================
% ========== IMPORTANT: UPDATE FILE PATH SO NO OVERWRITE =============
% ====================================================================
DATE = '03-06-25';
filePath = [DATE, '_com0_trial11_box'];

%% Setting up this file as if it can be used for the default experiment, too.
baseline = 1;

%% Open SerialPort and Connect to Optitrack/Motive
s1 = serialport('COM4',9600,'DataBits',8,'StopBits',1); % /dev/ttyUSB0
s1.Timeout = .15;

% Get rigidBody data ONCE to check that Optitrack has initialized
optitrack_stream_getFrame;
if ~optiClient.IsConnected
    fprintf('\nMake sure Optitrack and Motive are configured!\n');
    clear optiClient
    return
end

%% **** Robot and Force Sensor Definition ****
IDs                     = [5];       % ACTIVE AGENTS (LEADER MUST BE FIRST ELEMENT)
N                       = length(IDs);   % Number of robots
for i=1:N
    robots(i)           = Robot(IDs(i));
    force_sensors(i)    = ForceSensor(IDs(i));
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
run_time            = 60;
masterIdx           = 1;

% Data collection variables
data_time           = 0;
data_sensor         = [0 0];
data_payload_pose   = [0 0 0];
data_robot_pose     = [0 0 0];


%% **************** SEND ROBOT START COMMAND ************************
switchMode(IDs(1), s1);

%% ---------------   RUN EXPERIMENT   ----------------

while (t < run_time  &&  ~robots.stop)
    if (~ishandle(f1) || stop.Value)
        break
    end

    % Get frame from optitrack for plotting ONLY
    try
        optitrack_stream_getFrame;
    catch
        opti_T(:,:,1) = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
        opti_T(:,:,2) = opti_T(:,:,1);
        opti_theta = [0, 0];
    end

    % Read force sensor, store to alpha property. RECALL robot sends EE*100
    force_sensors = force_sensors.readSensor(s1, robots);
    raw_sensor = force_sensors.alpha;

    if ~baseline
        %% Quick stop condition check (value of 69)
        if (raw_sensor(1)==69 && raw_sensor(2)==69)
            break
        end
    else
        [robots, force_sensors] = update_robots_CASE(s1, N, robots, force_sensors);
        SerialCommunication(s1, robots, 192, 'u');
    end

    % Update live plot
    norm_sensor = raw_sensor/norm(raw_sensor);
    set(q1, 'UData', -norm_sensor(1), 'VData', -norm_sensor(2));

    % For data collection
    data_time           = [data_time;           t];
    data_sensor         = [data_sensor;         raw_sensor(1)/100, raw_sensor(2)/100];
    data_robot_pose     = [data_robot_pose;     opti_T(1,4, 1) opti_T(2,4, 1) opti_theta(1)];
    data_payload_pose   = [data_payload_pose;   opti_T(1,4, 2) opti_T(2,4, 2) opti_theta(2)];

    masterIdx = masterIdx + 1;
    t = toc(t_start);
    % pause(0.1) % No longer need to pause, just collecting data in RT...
end

% Get Final Run Time
t_final = round(t,1);
disp(['Final run time: ', num2str(t_final)]);

%% Cleanup - Stop robot motors, delete serialport
for i=1:N
    robots(i).u.data = zeros(6,1);
    SerialCommunication(s1, robots(i), 192, 'u');
end

% SEND ROBOT STOP COMMAND
switchMode(IDs(1), s1);

pause(0.5)
clear s1
close all;
disp("Deleted Serial Port")



%% Record experiment - ********* REMEMBER TO UPDATE FILE NAME TO AVOID OVERWRITING *********
savePath = ['CASE_2025\baseline_exp_', DATE, '\' filePath, '.mat'];
save(savePath, 'data_time', 'data_sensor', 'data_payload_pose', 'data_robot_pose', 'filePath');

