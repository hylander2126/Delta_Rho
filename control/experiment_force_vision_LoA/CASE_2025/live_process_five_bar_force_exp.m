%% Process MoCap Five-Bar Experiment
clc; close all; clear;

% DATE = '08-27-24';
DATE = '09-10-24';

addpath('Utilities\', ['ICRA_2025/exp_', DATE]);

numPos      = 3;    % Number of CoM positions tested
numTrials   = 10;    % Number of trials for this experiment
divFactor   = 10;   % Every nth step for text and plot shapes

PLOTSERIES  = 1;    % Whether to plot time series of sensor vs payload orientation
PLOTFOOT    = 0;    % Whether to plot combined footprint plot
PLOTVEL     = 0;    % Whether to plot angular velocity linearization plot

% c           = mat2cell(lines(numTrials), ones(1, numTrials), 3); % Colormap
c = {'c', 'm', 'g'};

box_exp         = '';  % _box % % Box payload or not




%% Now AVERAGE each of the runs for each CoM Position
seriesFig = figure('Color','w'); grid; hold on;
% Pre-allocate plot handles for live plotting
hF = plot(0, 0, 'LineStyle', '-',  'LineWidth', 4); % Sensor angle
hP = plot(0, 0, 'LineStyle', '--', 'LineWidth', 4); % Payload angle
hR = plot(0, 0, 'LineStyle', '-.', 'LineWidth', 4); % Push angle
hold on;

% Tidy up the plot(s)
% title(['Payload \theta_z vs Force Response \theta_z CoM Position ', num2str(i)]);
ax = gca;
xhand = xlabel('time (s)'); 
yhand = ylabel('+Y Axis Deviation (deg)');

xlim(ax,[-0.1, 9]);
ylim(ax, [-60, 60]);
leg = legend('Force Vector', 'Payload Orientation', 'Push Vector');
set(leg, 'FontSize', 20);
set(xhand, 'FontSize', 20); set(yhand, 'FontSize', 20);
set(seriesFig, 'Position', [300 300 1100 1000])
ax.FontSize = 20;

input('enter');

for i = 0:numPos-1
    times={}; robot_poses={}; payload_poses={}; sensor_forces={}; sensor_angles={}; payload_angles={};
    % figure; grid;
    
    % WHICH TRIAL?
    j = 6; % 1 for box 6 circ 8 circ

    data_parent_path = [DATE, '_com', num2str(i), '_trial', num2str(j), box_exp];
    load([data_parent_path, '.mat']);

    %% FIRST clean and average the data as needed

    % Get index of 'flat spot' at the start of sensor data
    flatIdx     = 5;
    for k = 5:length(data_time)
        if data_sensor(k,:) == data_sensor(5,:)
            flatIdx = flatIdx + 1;
        end
    end
    data_sensor(1:flatIdx,:) = zeros([flatIdx, 2]);
    data_robot_pose(1:flatIdx, 1:2) = zeros([flatIdx, 2]);

    % Get our desired data for plotting
    push_global = [0 0; diff(data_robot_pose(:,1:2))];
    push_local  = zeros(size(push_global));
    sensor_angle = [zeros(size(data_time))];
    for k = 1:length(data_time)
        if i==0 && ~strcmp(box_exp, '_box')
            tempt = Rz2(deg2rad(-30))*(-data_sensor(k,:)');
        else
            tempt = -data_sensor(k,:);
        end
        sensor_angle(k) = getAngle2(tempt, [0 1]); % [0; -1]);
        
        push_local(k,:) = reshape(Rz2(-data_robot_pose(k,3)) * push_global(k,:)', [1 2]);
        push_angle(k)   = getAngle2(push_local(k,:),  [0 1]); % [0; -1]);
    end



    payload_angle = (data_payload_pose(:,3) - data_robot_pose(:,3))';

    % Assign Cells for access when plotting
    sensor_angles = sensor_angle*180/pi;
    payload_angles = payload_angle*180/pi;
    % Also apply smoothing to this diff 
    push_angles = sgolayfilt(push_angle*180/pi, 3, 11);

    times = data_time;
    % payload_angles_global = (180/pi)*data_payload_pose(:,3)';

    % Get the average time step to correctly update live plot
    timeStep = diff(times);
    pauseTime = mean(timeStep);
    
    % Temp clean the random spike at the end
    if i==0
        sensor_angles(70:end) = randn;
        push_angles(70:end) = randn;
    end

    for k = 1:length(times)
        avgTime = times(1:k);
        plotF = sensor_angles(1:k);
        plotP = payload_angles(1:k);
        plotR = push_angles(1:k);
        
        % Update plot data
        set(hF, 'XData', avgTime, 'YData', plotF); % Update sensor angle
        set(hP, 'XData', avgTime, 'YData', plotP); % Update payload angle
        set(hR, 'XData', avgTime, 'YData', plotR); % Update push angle
        
        % Pause and redraw for live update effect
        drawnow;
        pause(pauseTime);%0.05);
    end
end
