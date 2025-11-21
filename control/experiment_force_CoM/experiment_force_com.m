clc; clear; close all;
addpath("Utilities\")
if exist('s1','var')
    delete(s1)
end

%% Open SerialPort
s1 = serialport('COM4',9600,'DataBits',8, 'StopBits',1);

%% **** AGENT ID and attachment points ****
N = 4;                                  % ACTIVE AGENT


%% Define Robot Properties
% Define Agent
Robot.ID            = N;
Robot.stop          = false;    % Robot stop condition
Robot.Kp            = 150;      % Proportional Gain
Robot.Kd            = 0.5;      % Derivative Gain
Robot.Kw            = 150;      % Rotational Proportional Gain
Robot.v_desired     = 170;      % Desired velocity magnitude
% Timeout for sensor
Robot.timeout       = 0.5;
% Agent motor values
Robot.u.data        = [0;0;0];
Robot.u.type        = 'uint8';
Robot.u.convertor   = @(x)uint8(x);
% Robot Jacobian (new wheels 02/11/23) 
Robot.J_r           = construct_robot_jacobian;
% Robot PD control parameters
Robot.t_prev        = 0;
Robot.e_prev        = 0;
Robot.e_th_prev     = 0;
Robot.I_e_th        = 0;        % Integral term for attitude correction
Robot.f_initial     = [];       % Initial force reading for attitude correction
Robot.low_e_ctr     = 0;
% Robot.de            = [0;0];
Robot.f_prev        = 0;
Robot.d_prev        = [0;0];    % Robot driving direction
% Robot initialization mode
Robot.initialize    = 1;

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
s1.Timeout = Robot.timeout;
%

%% Generate figure - Used to send STOP command
% generateFigure = false;
% f1 = figure();
% f1.Position = [600, 300, 800, 500];
% hold on
% axis equal;
% stop = uicontrol('style','toggle','string','stop');
% axis([-0.2 1.5 -0.5 0.5]);
% grid on
% xlabel('x'); ylabel('y');

%% PLOTTING initialization
% circ = linspace(0, 2*pi);
% circX = (0.15)*cos(circ); % 0.15 just works...
% circY = (0.15)*sin(circ);
% 
% circle(1) = patch(circX + x(1), circY + y(1), 'k');
% arrow(1) = quiver(0,0,0.25*Robot.negBound(1),0.25*Robot.negBound(2));
% arrow(2) = quiver(0,0,0.25*Robot.posBound(1),0.25*Robot.posBound(2));
% arrow(3) = quiver(0,0,0.25*Robot.desired_direc(1),0.25*Robot.desired_direc(2));


%% Stopwatch for checking minimal rotation threshold
runTime = tic;
global PD_data;
PD_data = [];

% fig1 = figure();

%% ****** RUN EXPERIMENT *******
while toc(runTime) < 8 && ~Robot.stop
    %% Assign robot motor torques | k=1 front left; k=2 rear; k=3 front right
    t = toc(runTime);
    [Robot, five_bar] = update_robot(t, Robot, five_bar, s1);
    
    Robot.t_prev = t;

    % quiver(0,0,five_bar.direction(1), five_bar.direction(2), 1,'Color','b')
    % hold on;
    % quiver(0,0,Robot.d_prev(1), Robot.d_prev(2), 'Color', 'g')
    % xlim([-2, 2]); ylim([-2, 2]);
    % hold off;
%     pause(0.01)
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


% plot_test;
%%
% save 'Results_09-14-23\blob_pos2_trial3_PD.mat' PD_data final_estimate;
return

%% ***** RESULTS ******
% NOTE: Measurement is taken FROM attachment point, NOT from CoM. finalMeasurement2 MUST BE NEGATED cause 180deg
%__________________________________________________________________________
pseudo_bisect = Robot.negBound + Robot.posBound;
new_direction = pseudo_bisect./norm(pseudo_bisect);

if testPt2
    finalMeasurement_360 = Rz2d(rad2deg(-payload.x.data(3))) * Robot.desired_direc % Robot.desired_direc
else
    finalMeasurement_180 = Rz2d(rad2deg(-payload.x.data(3))) * Robot.desired_direc % Robot.desired_direc
end

% For visualization (not really necessary)
set(arrow(3),'xdata',x(2),'ydata',y(2),'udata',0.25*new_direction(1),'vdata',0.25*new_direction(2));


%%
return
%%



%% Calculate and visualize intersection *AND* Save Trial

if testPt2
    save trial_2_360deg_try2.mat
else
    save trial_2_180deg_try2.mat
end


%% ********************** RUN THIS BLOCK WHEN FINISHED ********************
%% Export image and figure
fileName = ['com_phys_fig2 ' num2str(time) 's'];
exportgraphics(gca,[fileName '.png'],'Resolution',300)
saveas(gca,[fileName '.fig'])
save com_phys_fig2_9.5s
