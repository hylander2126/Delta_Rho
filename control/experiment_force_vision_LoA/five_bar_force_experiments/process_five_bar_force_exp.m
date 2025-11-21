%% Process MoCap Five-Bar Experiment
clc; clear; close all;
% addpath('Utilities\', 'five_bar_force_experiments\exp_06-06-24_five_bar_force\');
addpath('Utilities\', 'five_bar_force_experiments\exp_06-12-24_five_bar_force+mocap_data\');

% Whether to plot ALL trials or not
plotTrials = 1;

% Load the experiment(s) or trial(s)
% data_parent_path = '06-06-24-five_bar_force_trial';
data_parent_path = '06-12-24-five_bar_force+mocap_data_trial';

% Plotting variables
% error_plot = figure();
% error_plot_ax = gca;
% 
% hold off


%% Payload rotation vs Force sensor measurement
for j = 1:6
    load([data_parent_path, num2str(j), '.mat'])
    
    if plotTrials
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
        
        title(['Trial ' num2str(j)])
        co = colorbar;
        colormap(jet);
        xlabel 'x (mm)'
        ylabel 'y (mm)'
        co.Label.String = 'time (s)';
    end
    
    clearvars -except data_parent_path plotTrials
end



%% Now for Payload pose and robot pose tracking

temp_indices = [1, 95, 195, 256, 367, 599];

for j = 1:6
    load([data_parent_path, num2str(j), '.mat'])

    if plotTrials
        figure; hold on; grid;
        title('Payload Orientation vs Force Response');
        xlabel('time'); ylabel('angle wrt -y axis (rad)');

        % plot(data_robot_pose(:,1))
        % Get angle of force vector wrt -y axis to compare against payload theta
        for ii = 1:size(data_sensor,1)
            data_sensor_angle(ii,:) = getAngle2(data_sensor(ii,:), [0;-1]); %  - deg2rad(90); % Subtract 90 degs to get 
        end
        
        % For some reason matlab decided to break my payload and robot pose data. So I have to
        % Manually set the range of the data that applies to each trial...
        % data_payload_pose = data_payload_pose()

        plot(data_sensor_angle)
        plot(data_payload_pose(:,3))
        
        legend('Sensor Angle', 'Payload Orientation')
    end

    clearvars -except data_parent_path plotTrials
end

return


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



