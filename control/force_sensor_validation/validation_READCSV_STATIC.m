addpath("Utilities\");
addpath("validation_data\");
clc; clear; close all;

% Show or not to show figures
show_plot = 0;

%% Define problem constraints
% Spring constant calculation for: https://www.mcmaster.com/9271K665/
maxTorque = 0.25 * (25.4/1)*(4.448/1);      % in-lb -> N-mm
maxDeflection = pi;                         % radians = 180 degrees
k = -maxTorque/maxDeflection;               % N-mm/rad

% Spring positions - resting (by design)
phi = [deg2rad(120); deg2rad(60)];
% Define link lengths for 1 through 5
links = [25; 25; 40; 40; 22.84];
% Define joint coordinates
p = [0 0; links(5) 0; 0 0; 0 0; 0 0]';

%% Read and convert CSV
% data_raw = readmatrix("static-head-on-data_2023_09_12.csv");
data_raw = readmatrix("static-30-deg-data_2023_09_12.csv");
% data_raw = readmatrix("static+20-deg-data_2023_09_12.csv");
% data_raw = readmatrix("data_2023_09_13.csv");

true_direction = [0 -1];


data = data_raw;

timesteps = size(data, 1);


% Convert all sensor data from voltage to radians, add offset (0deg is 270 wrt x-axis), and wrap to 2Pi
data(:,2:3) = [mapfun(data(:,2), 0, 1024, 0, deg2rad(330)) , mapfun(data(:,3), 0, 1024, 0, deg2rad(330))];

% Resting angles should be ~120 and 60 degrees. Get offset from initial readings.
offset = mean([mean(data(1:4,2)) - deg2rad(120), mean(data(1:4,3)) - deg2rad(60)]);
data(:,2:3) = data(:,2:3) - offset;

%% Define five-bar properties
% Static values
five_bar.spring_k   = k;
five_bar.phi        = data(1,2:3)';
five_bar.links      = links;
five_bar.home       = [0;0]; % [11.42; 53.7105]; % Base joints and EE coords
% Dynamic values
five_bar.calib      = 1;        % Calibration mode starts ON
five_bar.p          = p;
five_bar.alpha      = [];
five_bar.direction  = [];
five_bar.force      = 0;
% Comparison variables
five_bar.deadzone   = 0.5; % 3.5; % grams
five_bar.window_size= 2; % Take the average of n samples for direction mean filtering
five_bar.prev_direc = [];
five_bar.ctr    = 1;

% TEMP
five_bar.gamma = [];

%% Loop thru data, initialize plot arrays
q = zeros(timesteps, 6);
d = zeros(timesteps, 2);
forces = zeros(timesteps,1);

for i = 1:timesteps
    five_bar.alpha = data(i,2:3)';
    
    five_bar = sensor_kinematics(five_bar);

    % Plotting variables / arrays
    q(i,:)      = reshape(five_bar.p(:,3:5), [1,6]); % ONLY USED FOR 'LIVE' PLOT
    d(i,:)      = five_bar.direction;
    forces(i,:) = five_bar.force;

    %TEMP
    gamma(i,:) = five_bar.gamma';
end

%__________________________________________________________________________________
%% Plotting

% 'Live' graphic
if show_plot
    % Create figure window
    fig = figure;
    axis equal
    hold on
    xlim([-40, 60]); ylim([-5, 60])

    for i=1:length(q)
        if ~isgraphics(fig)         % Break loop when figure is closed
            break
        end
%         if i == 2
%             input('enter')
%         end

        % Draw links
        h = plot([p(1,1) q(i,1) q(i,5) q(i,3) p(1,2)], [p(2,1) q(i,2) q(i,6) q(i,4) p(2,2)], '-ok');
        % Draw incoming force vector
        quiv = quiver(five_bar.home(1), five_bar.home(2), d(i,1), d(i,2), 1, 'm');
        title(sprintf('t: %4.2f, F: %4.1f, Cell: %4.1f, Diff: %4.1f', data(i,1)/1000, forces(i), data(i,2), forces(i)-data(i,2)));

        drawnow
        % pause(0.05)
        if forces(i) > 200
            pause(20)
        end
        delete(h); delete(quiv)
    end
end

% ------------------------------------------------------------
% Magnitude error over time
fig2 = figure;
title('Force Magnitude Readings'); xlabel('Time'); ylabel('Force (g)','Color','k')
hold on

plot(data(:,1)/1000, forces,'b-','LineWidth',2)
% plot(forces-data(:,2), 'r--', 'LineWidth',2)

fontsize(gcf, 24, "points")
legend('Force Sensor')
hold off

%% Find average force reading (assuming same angle and same load applied)
avg_force = mean(findpeaks(forces(forces>20)))


% % ------------------------------------------------------------
% % Angular error over time (new fig)
% fig3 = figure;
% title('Force Angle Readings'); xlabel('Time'); ylabel('Angle (\circ)','Color','k')
% hold on
% 
% % Make array of load cell 'angle' over time - really just (true_direction)deg when >0g and (0)deg at ==0g
% true_angle = zeros(timesteps, 1);
% true_angle(data(:,2)>2, :) = 0; % rad2deg(atan2(true_direction(2), true_direction(1)));
% % true_directions = data(:,2) * true_direction;
% 
% % Make array of force sensor's 'angle' over time
% for i = 1:timesteps
%     u = true_direction;
%     v = d(i,:);
%     if abs(forces(i)) < 2.75
%         sensor_angle(i,:) = 0;
%     else
%         sensor_angle(i,:) = rad2deg(atan2(v(2), v(1)) - atan2(u(2), u(1)));% rad2deg(atan2(d(i,2),d(i,1)));
%     end
% end
% 
% % Root mean squared error
% % for i=1:timesteps
% %     angle_error(i,:) = rmse(true_directions(i,:), anglee(i,:));
% % end
% 
% % Calculate direction error over time
% angle_error = true_angle - sensor_angle;
% 
% plot(true_angle, 'k-','LineWidth',2)
% plot(sensor_angle,'b-','LineWidth',2)
% % plot(angle_error,'Color', [0.4940 0.1840 0.5560],'linestyle','--', 'LineWidth',1)
% 
% legend('Load Cell', 'Force Sensor');
% fontsize(gcf, 24, "points")

% ------------------------------------------------------------
% Display maximum difference
disp("Max magnitude error b/w sensor and ground truth:")
disp(max(abs(forces - data(:,2))))
disp("Max angle error b/w sensor and ground truth:")
disp(max(abs(angle_error)))
disp("Root Mean Squared Angular Error")
disp(rmse(true_angle, sensor_angle))
