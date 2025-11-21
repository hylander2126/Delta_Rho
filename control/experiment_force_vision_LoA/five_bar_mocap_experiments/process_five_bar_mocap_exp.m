%% Process MoCap Five-Bar Experiment
clc; clear; close all;
addpath('Utilities\','five_bar_mocap_experiments\exp_05-20-24-five_bar_mocap_com\')

% Whether to plot ALL trials or not
plotTrials = 0;

% Load the experiment(s) or trial(s)
% data_parent_path = 'five_bar_mocap_experiments\exp_05-20-24-five_bar_mocap_com\05-20-24-five_bar_mocap_com_trial';
data_parent_path = '05-20-24-five_bar_mocap_com_trial';

% Plotting variables
% error_plot = figure();
% error_plot_ax = gca;
% 
% hold off

e_angle = cell(8,1);

%% Payload rotation vs Force sensor measurement
for i = 1:8
    load([data_parent_path, num2str(i), '.mat'])
    
    % Sensor data is expressed in local frame, need to rotate by OPPOSITE robot's orientation
    for j = 1:size(data_sensor,1)
        data_sensor(j,:) = (Rz2(-data_robot_pose(j,3)) * data_sensor(j,:)')';
    end

    % *OPTIONAL*: Get payload orientation in the LOCAL robot frame instead. Must comment out above code.
    % data_payload_pose(:,3) = data_payload_pose(:,3) - data_robot_pose(:,3);


     % Get angle of sensor feedback (reduce dimension of sensor data)
    for ii = 1:size(data_sensor,1)
        data_sensor_angle(ii,:) = getAngle2(data_sensor(ii,:), [0;-1]); %  - deg2rad(90); % Subtract 90 degs to get 
    end

    % For simplicity, get the orientation(s) of the payload for this trial
    payload_orient = data_payload_pose(:,3);
    % payload_orient = data_robot_pose(:,3);

    if plotTrials
        f1 = figure("Position",[600 300 800 500]); hold on;
        titleText = 'Payload Orientation vs Sensor Angle (from [0,-1]) in GLOBAL frame. Trial ';
        title([titleText, num2str(i)])

        plot(data_time, payload_orient, 'g')
        plot(data_time, data_sensor_angle, 'm')
        xlabel 'time (s)'; ylabel 'rad'
        legend('\theta\_p', '\phi\_sensor')
        hold off
    end
    
    % Get this trial's error between the sensor angle and the payload orientation
    e_angle{i} = data_sensor_angle - payload_orient;
    
    clearvars -except data_parent_path plotTrials e_angle
end

%% Error plotting
maxLength = max(cellfun(@length, e_angle));
paddedData = NaN(maxLength, 8);

for i = 1:8
    currLength = length(e_angle{i});
    paddedData(1:currLength, i) = e_angle{i};
end

boxplot(paddedData, 'Labels', {'Trial 1', 'Trial 2', 'Trial 3', 'Trial 4', 'Trial 5', 'Trial 6', 'Trial 7', 'Trial 8'})
title('Angle Error Between Force Feedback and Payload Orientation for All Trials')
ylabel('Angle (rads)')

return 


%% Now for payload rotation vs robot's command input (which is local frame)
for i = 1:8
    load([data_parent_path, num2str(i), '.mat'])

    f2 = figure("Position",[600 300 800 500]); hold on;
    axis equal;
    
    % Plot cos and sin of payload rotation (add dimension to payload rotation)
    title(['Payload & Robot Rotation vs Robot Command - Trial ', num2str(i)])
    plot(data_time, data_payload_pose(:,3), 'b')
    plot(data_time, data_robot_pose(:,3), 'g')
    plot(data_time, data_robot_command(:,1)/20, '--r')
    xlabel 'time (s)'; ylabel 'mm'
    legend('\theta\_p', '\theta\_r', 'robot\_\theta / 20')
    hold off
    
    clearvars -except data_parent_path
    
end

% xlim([-2 2]); ylim([-1 1]);
% % Initial center coordinates of the circle
% radius = .1;
% % Handle for the robot, initially drawn at the origin
% h = rectangle('Position',[0-radius, 0-radius, 2*radius, 2*radius], ...
%     'Curvature', [1, 1], 'FaceColor', 'b');
% 
% for i = 1:size(data_time,1)
%     x_center = data_robot_position(i,1);
%     y_center = data_robot_position(i,2);
%     set(h, 'Position', [x_center-radius, y_center-radius, 2*radius, 2*radius])
%     pause(0.1)
% end



