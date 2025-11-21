%% Process MoCap Five-Bar Experiment TRAJECTORY
clc; clear; close all;

DATE = '09-10-24';
addpath('Utilities\', ['ICRA_2025/exp_', DATE]);

plotTrials  = 0;    % Whether to plot ALL trials or not
numPos      = 3;    % Number of CoM positions tested
numTrials   = 10;    % Number of trials for this experiment
saveVids    = 0;    % Save animation as mp4
divFactor   = 10;   % Every nth step for text and plot shapes
% comPos      = 1;    % Which com position to visualize

box_exp         = '_box';  % _box % % Box payload or not


for i=2 %0:numPos-1
    for k=1:numTrials
        % Load the experiment and trial(s)
        % clearvars -except i k numTrials
        data_parent_path = [DATE, '_com', num2str(i), '_trial', num2str(k), box_exp];
        load([data_parent_path, '.mat']);

        % Creating a figure for the 3D plot
        f1 = figure;
        ax = gca;
        
        % Have to flip cause of optitrack mistake ...
        % robot_pose = data_payload_pose;
        % payload_pose = data_robot_pose;
        robot_pose = data_robot_pose;
        payload_pose = data_payload_pose;
        
        % Adding the 3D line plot for the payload trajectory
        plot3(payload_pose(:, 1), payload_pose(:, 2), data_time, 'DisplayName', ...
            'Payload Trajectory', 'Color', [0, 0, 1, 0.1]); % Tiny alpha
        hold on;
        % Adding the 3D line plot for the robot trajectory
        plot3(robot_pose(:, 1), robot_pose(:, 2), data_time, 'DisplayName', ...
            'Robot Trajectory', 'Color', [1, 0, 0, 0.1]); % Tiny alpha
        
        % Adding time annotations to key points
        for j = 1 : max(1, floor(length(data_time)/divFactor)) : length(data_time)
            text(payload_pose(j, 1), payload_pose(j, 2), data_time(j), sprintf('t=%.1f', ...
                data_time(j)), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', ... 
                'center', 'FontSize', 9);
        end
        
        
        
        %% Now plot as shapes also.
        
        % Plot payload and robot shapes over the trajectory
        for j = 1 : max(1, floor(length(data_time)/divFactor)) : length(data_time)
            % Plot the payload as a circle
            plot_oriented_shape(ax, ...
                                payload_pose(j, 1), ...
                                payload_pose(j, 2), ...
                                data_time(j), ...
                                payload_pose(j, 3), ...
                                'circle', ...
                                0.1, 'blue');
            
            hold on;
            
            % Plot the robot as a triangle
            plot_oriented_shape(ax, ...
                                robot_pose(j, 1), ...
                                robot_pose(j, 2), ...
                                data_time(j), ...
                                robot_pose(j, 3), ...
                                'triangle', ...
                                0.1, 'red');
            hold on;
        end
        
        % Labeling the axes
        xlabel('X Position');
        ylabel('Y Position');
        zlabel('Time');
        
        % Adding the title
        title(sprintf('3D Trajectory of Payload and Robot (Z = Time) CoM %i Trial %i', [i, k]));
        grid on;
        axis equal;
        
        % Calculate the limits with padding for both x and y axes
        x_min = min(data_payload_pose(:, 1)) - 0.4;
        x_max = max(data_payload_pose(:, 1)) + 0.4;
        y_min = min(data_payload_pose(:, 2)) - 0.4;
        y_max = max(data_payload_pose(:, 2)) + 0.4;
        
        % Find the overall limits for x and y
        lim_min = min(x_min, y_min);
        lim_max = max(x_max, y_max);
        
        % Set the axis limits to ensure they are equal
        lim = max(abs(lim_min), abs(lim_max));
        xlim([-lim lim]);
        ylim([-lim lim]);
        
        view(3); % Set the view to 3D
        
        % Alter the 'padding' of the z axis (make it dense)
        ax.DataAspectRatio = [1 1 6];
        % Save each figure
        saveas(f1, sprintf('08-27-24_com%i_trial%i', [i,k]), 'png');
    end
end



%% Functions
function plot_oriented_shape(ax, x, y, z, angle, shape, shape_size, shape_color)
    if nargin < 7
        shape_size = 0.1;
    end
    if nargin < 8
        shape_color = 'blue';
    end
    
    if strcmp(shape, 'circle')
        % Create points for a circle
        theta = linspace(0, 2 * pi, 100);
        circle_points = [shape_size * cos(theta); shape_size * sin(theta)];
        
        % Create rotation matrix
        rotation_matrix = [cos(angle), -sin(angle); sin(angle), cos(angle)];
        
        % Rotate the points
        rotated_points = rotation_matrix * circle_points;
        
        % Plot the circle
        plot3(ax, rotated_points(1, :) + x, rotated_points(2, :) + y, z * ones(1, length(theta)), 'Color', shape_color);
        
    elseif strcmp(shape, 'triangle')
        % Create points for a triangle
        shape_size = shape_size*1.5;
        h = (sqrt(3)/2 * shape_size);
        verts_x = [-shape_size/2, shape_size/2, 0];
        verts_y = [0+h/2, 0+h/2, -h+h/2];
        % triangle_points = [-size*sqrt(3)/2, size*2/3 ; sqrt(3)/2, size*2/3; 0, -size]';
        triangle_points = [verts_x; verts_y];

        % Create rotation matrix
        rotation_matrix = [cos(angle), -sin(angle); sin(angle), cos(angle)];
        
        % Rotate the points
        rotated_points = rotation_matrix * triangle_points;
        
        % Plot the triangle
        plot3(ax, ...
            [rotated_points(1, :) rotated_points(1, 1)] + x, ...
            [rotated_points(2, :) rotated_points(2, 1)] + y, ...
            z * ones(1, 4), 'Color', shape_color);
    end
end
