clc; clear; close all;
addpath("Utilities\")
if exist('s1','var')
    delete(s1)
end

%% Open SerialPort
s1 = serialport('COM4',9600,'DataBits',8, 'StopBits',1);

%% **** AGENT IDs and attachment points ****
N = [4];                            % ACTIVE AGENTS (LEADER MUST BE FIRST ELEMENT)
leader = 4;                                % Robot with active force sensing (Leader)

%% Define Robot Properties
for i=1:1 % length(N)
    n = 1; %N(i);
    Robot(n).ID            = 4;
    Robot(n).stop          = false;    % Robot stop condition
    Robot(n).Kp            = 170;      % Proportional Gain
    Robot(n).Kd            = 0.5;      % Derivative Gain
    Robot(n).Kw            = 150;      % Rotational Proportional Gain
    % Agent motor values
    Robot(n).u.data        = [0;0;0];
    Robot(n).u.type        = 'uint8';
    Robot(n).u.convertor   = @(x)uint8(x);
    % Robot Jacobian (new wheels 02/11/23) 
    Robot(n).J_r           = construct_robot_jacobian;
    % Robot PD control parameters
    Robot(n).t_prev        = 0;
    Robot(n).e_prev        = 0;
    Robot(n).e_th_prev     = 0;
    Robot(n).I_e_th        = 0;        % Integral term for attitude correction
    Robot(n).f_initial     = [];       % Initial force reading for attitude correction
    Robot(n).low_e_ctr     = 0;
    % Robot.de            = [0;0];
    Robot(n).f_prev        = 0;
    Robot(n).d_prev        = [0;0];    % Robot driving direction
    % Robot initialization mode
    Robot(n).initialize    = 1;
end


%% Define five-bar problem constraints
% Spring constant calculation for: https://www.mcmaster.com/9271K665/
maxTorque = 0.25 * (25.4/1)*(4.448/1);      % in-lb -> N-mm
maxDeflection = pi;                         % radians = 180 degrees
spring_k = -maxTorque/maxDeflection;               % N-mm/rad

% Define link lengths for 1 through 5
links = [25; 25; 40; 40; 22.84];
% Define joint coordinates
p = [0 0; links(5) 0; 0 0; 0 0; 0 0]';

%% Define five-bar properties
% Static values
five_bar.spring_k   = spring_k;
five_bar.links      = links;
five_bar.deadzone   = 14.0;      % Deadzone for minimum detection in grams % 3.5
five_bar.home       = [0;0];    % [11.42; 53.7105]; % This is set in the first few steps
five_bar.phi        = [0;0];
five_bar.offset     = 0;        % Nominal offset for potentiometers
% Arrays for 'avg'-ing on startup
five_bar.home_avg   = [];       % \
five_bar.phi_avg    = [];       % For getting an avg during calibration
five_bar.offset_avg = [];       % /
% Dynamic values
five_bar.init       = 1;        % Offset calculation
five_bar.calib      = 1;        % Calibration mode starts ON
five_bar.p          = p;        % Joint positions
five_bar.alpha      = [];       
five_bar.direction  = [0;0];    % Force direction
five_bar.force      = 0;        % Force magnitude
% Comparison variables
five_bar.window_size= 1; % Take the average of n samples for direction mean filtering
five_bar.prev_direc = [];
five_bar.ctr        = 1;


% 
s1.Timeout = 0.5;
%

%% Generate figure - Used to send STOP command
% generateFigure = false;
f1 = figure();
f1.Position = [600, 300, 800, 500];
hold on
stop = uicontrol('style','toggle','string','stop');

%% Stopwatch for checking minimal rotation threshold
runTime = tic;
global PD_data;
PD_data = [];

%% ****** RUN EXPERIMENT *******
Robot(1).stop
while (toc(runTime) < 290 && ~Robot(1).stop)
    %% Assign robot motor torques | k=1 front left; k=2 rear; k=3 front right
    t = toc(runTime);
    [Robot, five_bar] = update_robot(t, Robot, leader, five_bar, s1);
    
    Robot.t_prev = t;

    if stop.Value
        Robot.stop = 1;
    end
end
%%
final_estimate = Robot.d_prev;

%% Cleanup - Stop Robot motors, delete serialport
Robot.u.data = zeros(6,1);
SerialCommunication(s1,Robot,192,'u');
delete(s1)
disp("Deleted Serial Port")

% Get Final Run Time
time = round(toc(runTime),1);
disp(['Final run time: ', num2str(time)]);

return
