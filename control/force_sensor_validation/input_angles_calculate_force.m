clc; clear; close all
addpath('Utilities\')


%% Create figure window
figure
axis equal
hold on

%% Define problem constraints
% Spring constant calculation
% For the following spring: https://www.mcmaster.com/9271K665/
maxTorque = 0.25; % in-lb
maxDeflection = pi; % radians = 180 degrees

maxTorque = maxTorque * (25.4/1) * (4.448/1); % N-mm
k = -maxTorque/maxDeflection; % N-mm/rad


%% INPUT FIRST TWO ANGLES IN DEGREES
alpha = [deg2rad(127); deg2rad(55)]; % Resting angles
% alpha = [deg2rad(130); deg2rad(50)]; % Straight down 10deg
% alpha = [deg2rad(140); deg2rad(40)]; % Straight down 20deg
% alpha = [deg2rad(110); deg2rad(70)]; % Straight up 10deg
% alpha = [deg2rad(95); deg2rad(85)]; % Straight up 25deg
% alpha = [deg2rad(130); deg2rad(70)]; % "Left" 10 deg
% alpha = [deg2rad(140); deg2rad(80)]; % "Left" 20 deg
% alpha = [deg2rad(110); deg2rad(50)];
 
% TEMP - FROM EXPERIMENT -30deg
% alpha = [2.1283; 0.4185]; % This gave a force of ~25.87. Should be ~50

% Spring positions - resting (by design)
phi = [deg2rad(120); deg2rad(60)];
% Define link lengths for 1 through 5
links = [25; 25; 40; 40; 22.84];
% Define joint coordinates
p = [0 0; links(5) 0; 0 0; 0 0; 0 0]';

%% Define five-bar properties
% Static values
five_bar.spring_k   = k;
five_bar.phi        = phi;
five_bar.links      = links;
five_bar.home       = [11.42; 53.7105]; % Base joints and EE coords
% Dynamic values
five_bar.calib      = 0;        % Calibration mode starts ON
five_bar.p          = p;
five_bar.alpha      = alpha;
five_bar.direction  = [];
five_bar.force      = 0;
% Comparison variables
five_bar.deadzone   = 0.5; % 3.5; % grams
five_bar.window_size= 2; % Take the average of n samples for direction mean filtering
five_bar.prev_direc = [];
five_bar.ctr    = 1;

%% Calculate force and direction
five_bar = sensor_kinematics(five_bar);

disp('Force and direction:')
five_bar.force
five_bar.direction

% Reassign for easy plotting
p = five_bar.p;

% Plot five bar
xlim([-links(1) p(1,2)+links(2)])
ylim([p(2,1) links(1)+links(3)])
pl = plot([p(1,1) p(1,3) p(1,5) p(1,4) p(1,2)], [p(2,1) p(2,3) p(2,5) p(2,4) p(2,2)], '-ok');
hold on;
quiv = quiver(five_bar.home(1), five_bar.home(2), p(1,5)-five_bar.home(1), p(2,5)-five_bar.home(2), 1, 'm');
hold off;

% global alpha
% 
% % Create sliders
% slider1 = uicontrol('Style', 'slider', 'Min', deg2rad(60), 'Max', deg2rad(180), 'Value', alpha(1), 'Units', 'normalized', ...
%     'Position', [0.1 0.01 0.8 0.05], 'Callback', @(src, event) updatePlot(src, event, 1, five_bar, pl, quiv, alpha));
% slider2 = uicontrol('Style', 'slider', 'Min', deg2rad(0), 'Max', deg2rad(120), 'Value', alpha(2), 'Units', ...
%     'normalized', 'Position', [0.1 0.05 0.8 0.05], 'Callback', @(src, event) updatePlot(src, event, 2, five_bar, pl, quiv, alpha));
% 
%  % Nested function to update plot
% function updatePlot(src, ~, sliderID, five_bar, pl, quiv, alpha)
%     if sliderID == 1
%         alpha(1) = src.Value;
%     elseif sliderID == 2
%         alpha(2) = src.Value;
%     end
%     
% 
%     five_bar.alpha = alpha;
%     % Update p
%     five_bar = sensor_kinematics(five_bar);
%     p = five_bar.p;
% 
%     pl.XData = [p(1,1) p(1,3) p(1,5) p(1,4) p(1,2)];
%     pl.YData = [p(2,1) p(2,3) p(2,5) p(2,4) p(2,2)];
%     
%     quiv.UData = p(1,5) - five_bar.home(1);
%     quiv.VData = p(2,5) - five_bar.home(2);
%
%     drawnow;
% end
