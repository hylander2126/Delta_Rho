%% Last updated 09/21/23
% Digital twin of five bar force sensor. Some values are hardcoded. 

clc; clear; close all
addpath('Utilities\')

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


%% Define five-bar properties
% Static values
five_bar.spring_k   = k;
five_bar.phi        = phi;
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
five_bar.ctr        = 1;

%% Live Plot - Create figure window
fig(1) = figure('Units','normalized','OuterPosition',[0.1 0.1 0.7 0.8]);
axis equal
hold on
xlim([-links(1) p(1,2)+links(2)]); xlabel('x (mm)');
ylim([p(2,1) links(1)+links(3)]); ylabel('y (mm)');

%% Arduino initialization and pin definitions
sensorPin1 = "A0";
sensorPin2 = "A1";
a = arduino("COM5","Uno","BaudRate",9600,"Libraries","I2C");


%% Calibration - unfortunately need separate calibration loop due to plotting...
init_raw_data = [readVoltage(a,sensorPin1); readVoltage(a,sensorPin2)] .*1023./5;
% Convert all sensor data from voltage to radians, add offset (0deg is 270 wrt x-axis), and wrap to 2Pi
init_data = [mapfun(init_raw_data(1), 0, 1024, 0, deg2rad(330)) ; mapfun(init_raw_data(2), 0, 1024, 0, deg2rad(330))];
% Resting angles should be ~120 and 60 degrees. Get offset from initial readings.
offset = mean([init_data(1) - deg2rad(120), init_data(2) - deg2rad(60)]);
five_bar.alpha = init_data - offset;
% Run kinematics
five_bar = sensor_kinematics(five_bar);
% Initialize plot object to delete at the proper time
p = five_bar.p;
h = plot([p(1,1) p(1,3) p(1,5) p(1,4) p(1,2)], ...
         [p(2,1) p(2,3) p(2,5) p(2,4) p(2,2)], '-ok');

q = quiver(11.42, 53.7105, p(1,5)-11.42, p(2,5)-53.7105, 1, 'm');

hold on

%% Loop for desired duration of sensor testing
timesteps = 500;
for i = 1:timesteps
    if ~isgraphics(fig(1))         % Break loop when figure is closed
            break
    end

    %% Read voltages and convert to angles
    raw_data = [readVoltage(a,sensorPin1); readVoltage(a,sensorPin2)] .*1023./5;
    data = [mapfun(raw_data(1), 0, 1024, 0, deg2rad(330)) ; mapfun(raw_data(2), 0, 1024, 0, deg2rad(330))];
    five_bar.alpha = data - offset;
    % Run kinematics
    five_bar = sensor_kinematics(five_bar);

%         j = i-1:i; % Range for path tracing
%         plot(Cx(j),Cy(j),'Color','r')

    % Update plot handles for links and arrow
    p = five_bar.p;
    h.XData = [p(1,1) p(1,3) p(1,5) p(1,4) p(1,2)];
    h.YData = [p(2,1) p(2,3) p(2,5) p(2,4) p(2,2)];
    q.UData = p(1,5)-11.42;
    q.VData = p(2,5)-53.7105;
    
    drawnow
    % pause(0.01)
end
close all;
