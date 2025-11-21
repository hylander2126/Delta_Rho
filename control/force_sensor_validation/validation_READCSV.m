addpath("Utilities\");
addpath("validation_data\");
% clc; clear; close all;

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
data = data_raw;
timesteps = size(data, 1);

% Convert all sensor data from voltage to radians, add offset (0deg is 270 wrt x-axis), and wrap to 2Pi
data(:,3:4) = [mapfun(data(:,3), 0, 1024, 0, deg2rad(330)) , mapfun(data(:,4), 0, 1024, 0, deg2rad(330))];

% Resting angles should be ~120 and 60 degrees. Get offset from initial readings.
offset = mean([mean(data(1:4,3)) - deg2rad(120), mean(data(1:4,4)) - deg2rad(60)]);
data(:,3:4) = data(:,3:4) - offset;

%% Define five-bar properties
% Static values
five_bar.spring_k   = k;
five_bar.phi        = data(1,3:4)';
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

%% Loop thru data, initialize plot arrays
q = zeros(timesteps, 6);
d = zeros(timesteps, 2);
forces = zeros(timesteps,1);

for i = 1:timesteps
    five_bar.alpha = data(i,3:4)';
    
    five_bar = sensor_kinematics(five_bar);

    % Plotting variables / arrays
    q(i,:)      = reshape(five_bar.p(:,3:5), [1,6]); % ONLY USED FOR 'LIVE' PLOT
    d(i,:)      = five_bar.direction;
    forces(i,:) = five_bar.force;
end

%% Fix any nan forces
forces(isnan(forces)) = 0;

%__________________________________________________________________________________
%% Plotting

% 'Live' graphic
if show_plot
    % Create figure window
    fig(1) = figure;
    axis equal
    hold on
    xlim([-40, 60]); ylim([-5, 60])

    for i=1:length(q)
        if ~isgraphics(fig)         % Break loop when figure is closed
            break
        end

        % Draw links
        h = plot([p(1,1) q(i,1) q(i,5) q(i,3) p(1,2)], [p(2,1) q(i,2) q(i,6) q(i,4) p(2,2)], '-ok');
        % Draw incoming force vector
        quiv = quiver(five_bar.home(1), five_bar.home(2), d(i,1), d(i,2), 1, 'm');
        title(sprintf('Force: %4.1f, Cell: %4.1f, Diff: %4.1f', forces(i), data(i,2), forces(i)-data(i,2)));

        drawnow
        pause(0.05)
        delete(h); delete(quiv)
    end
end

% ------------------------------------------------------------
% Magnitude error over time
fig(2) = figure('units','normalized','OuterPosition',[0 0 0.8 0.8]);
xlabel("Time (sec)"); ylabel("Force (g)")
xlim([0 30])
ylim([-25 100])
grid on
hold on

h1 = plot(data(:,1)/100, data(:,2),'k-','LineWidth',4);
h2 = plot(data(:,1)/100, forces,'g-','LineWidth',4);
h3 = plot(data(:,1)/100, forces-data(:,2), 'r--', 'LineWidth',4);

fontsize(gcf, 36, "points")
legend('Load Cell','Force Sensor','Magnitude Error', 'Location','northwest')

input('enter')
%% For video:
for k = 2:size(data,1)
    % Update data for the three plots
    set(h1, 'XData', data(1:k,1)/100, 'YData', data(1:k,2));
    set(h2, 'XData', data(1:k,1)/100, 'YData', forces(1:k));
    set(h3, 'XData', data(1:k,1)/100, 'YData', forces(1:k) - data(1:k,2));

    drawnow;
    
    % Optional: pause to control speed
    pause(0.03);  % Pause for 50 milliseconds. Adjust as needed.
end

hold off

% ------------------------------------------------------------
% Angular error over time (new fig)
% fig(3) = figure('units','normalized','OuterPosition',[0 0 0.8 0.8]);
% hold on
% 
% % Make array of load cell 'angle' over time - really just (true_direction)deg when >0g and (0)deg at ==0g
% true_angle = (pi/4) * ones(timesteps, 1);
% 
% % Make array of force sensor's 'angle' over time
% for i = 1:timesteps
%     u = true_direction;
%     v = d(i,:);
%     if abs(forces(i)) < 2.75
%         sensor_angle(i,:) = 0;
%     else
%         sensor_angle(i,:) = rad2deg(acos(dot(v, [0,-1]) / (norm(v) * norm([0,-1])))) ;% rad2deg(atan2(v(2), v(1)) - atan2(u(2), u(1)));% rad2deg(atan2(d(i,2),d(i,1)));
%     end
% end
% 
% % Root mean squared error
% % for i=1:timesteps
% %     angle_error(i,:) = rmse(true_directions(i,:), anglee(i,:));
% % end
% 
% plot(true_angle, 'k-','LineWidth',2)
% hold on
% plot(sensor_angle,'b-','LineWidth',2)
% 
% legend('True angle', 'Force Sensor');
% fontsize(gcf, 24, "points")
% hold off 
% ------------------------------------------------------------
% Display maximum difference
% disp("Max angle error b/w sensor and ground truth:")
% % disp(max(abs(angle_error)))
% disp("Root Mean Squared Angular Error")
% disp(rmse(true_angle, sensor_angle))

% Get maximum magnitude error
disp("Max magnitude error b/w sensor and ground truth:")
disp(max(abs(forces - data(:,2))))
% Get RMSE of the forces
disp("Root Mean Squared Angular Error")
disp(rmse(forces,data(:,2)))
% Get standard deviation
disp("Standard Deviation:")
disp(std(forces-data(:,2)))
