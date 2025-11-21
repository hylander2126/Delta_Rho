clc; close all;

if exist('s1','var')
    delete(s1)
end

%% Add Path, open SerialPort
addpath("Utilities\")

BaudRate = 9600;
s1 = serialport('COM3',9600,'DataBits',8, 'StopBits',1);

%% Generate figure - Used to send STOP command
generateFigure = false;
f1 = figure();
f1.Position = [600, 300, 800, 500];
hold on
axis equal;
stop = uicontrol('style','toggle','string','stop');
axis([-0.2 1.5 -0.5 0.5]);
grid on
xlabel('x'); ylabel('y');


%% Agent ID and attachment points
N = 4;                                 % ACTIVE AGENT

% NOTE: If default config (robot 180deg) set to 0. Otherwise, set to 1. This changes angle wrapping from Optitrack
testPt2 = 1;                            % ATTACH POINT 0 = 180 , 1 = 0 or 1 = 360

if ~testPt2
    theta = 180;                        % Default attachment point.
else
    theta = 90;                         % Secondary attachment point.
end

r = (172/2)*[cosd(theta);sind(theta)]; % Payload radius * unit on circle

%% Define Payload and Robot Properties
% Define object
payload.x.data          = [0;0;0];
payload.theta_prev      = 0;           % Previous rotation

% Define Agent
robot(1).ID             = N;
robot(1).K_wp           = 5;%3.5trial2_360        % Rotation Proportional Gain
robot(1).K_wd           = 20;%10trial2_180       % Rotation Pseudo-Derivative Gain
robot(1).v_desired      = 170;      % Desired velocity magnitude
robot(1).min_theta      = 3;        % Threshold for 'minimal' payload rotation (DEGREES)
robot(1).max_theta      = 10;       % Threshold for 'measureable' payload rotation (DEGREES)
% Agent position
robot(1).x.data         = [0;0;0];
robot(1).x.type         = 'int16';
robot(1).x.convertor    = @(x)int16(x);
% Agent motor values
robot(1).u.data         = [];
robot(1).u.type         = 'uint8';
robot(1).u.convertor    = @(x)uint8(x);
% Agent measurements
robot(1).desired_direc  = [];   % INITIAL DIRECTION - wrt World Frame
robot(1).prev_meas      = 0;        % Payload's rotation at previous measurement capture
robot(1).measure_t      = 0;        % Time at previous measurement
% Agent bounds
robot(1).negBound       = [];
robot(1).posBound       = [];
% Decentralized Jacobian from Agent Position "P" on Object
robot(1).J              = construct_jacobian(r(:,1));
robot(1).pJ_            = pinv(robot(1).J);   
% Robot Jacobian (new wheels 02/11/23)
robot(1).J_r            = construct_robot_jacobian;
% GLOBAL STOP condition
robot(1).stop           = 0;
% For cleaner code.
Robot = robot(1);


%% Get Initial Frame from Motive | Get x,y,z,Q,R       NOTE: Rotation stored IN RADIANS
optitrack_read_rigidbody_frame;
update_robot_payload_positions;

% Initial 'measurement' values
Robot.prev_meas = theta_p;
payload.theta_prev = payload.x.data(3);

%% Initial search bounds: unit vectors (previously get_init_bounds)
init_r = Rz2d(rad2deg(theta_p)) * r;                % Attachment point, updated w payload rotation
Robot.negBound = Rz2d(90) * init_r/norm(init_r);    % Clockwise-most bound
Robot.posBound = Rz2d(-90) * init_r/norm(init_r);   % C.Clockwise-most bound

% Set initial direction as bisecting initial bounds
first_push = payload.x.data(1:2) - Robot.x.data(1:2);
Robot.desired_direc = first_push/norm(first_push);


%% PLOTTING initialization
circ = linspace(0, 2*pi);
circX = (0.15)*cos(circ); % 0.15 just works...
circY = (0.15)*sin(circ);

circle(1) = patch(circX + x(1), circY + y(1), 'k');
arrow(1) = quiver(0,0,0.25*Robot.negBound(1),0.25*Robot.negBound(2));
arrow(2) = quiver(0,0,0.25*Robot.posBound(1),0.25*Robot.posBound(2));
arrow(3) = quiver(0,0,0.25*Robot.desired_direc(1),0.25*Robot.desired_direc(2));


%% Stopwatch for checking minimal rotation threshold
runTime = tic;


%% ***** Run Experiment ******
%__________________________________________________________________________
while(~get(stop,'value') && ~Robot.stop)

    % Update Robot and Payload properties: get x,y,z,Q,R and determine rotation
    optitrack_read_rigidbody_frame;
    update_robot_payload_positions;

%     disp(Robot.desired_direc);


    %% Update robot's desired heading and take measurement
    t = toc(runTime);
    Robot = updateRobot(t,Robot,payload,s1);
    

    % Send command to Robot
    SerialCommunication(s1,Robot,192,'u');

    % Update plot arrow information
    set(circle(1),'xdata',circX + x(1), 'ydata', circY + y(1));
    set(arrow(1),'xdata',x(2),'ydata',y(2),'udata',0.25*Robot.negBound(1),'vdata',0.25*Robot.negBound(2));
    set(arrow(2),'xdata',x(2),'ydata',y(2),'udata',0.25*Robot.posBound(1),'vdata',0.25*Robot.posBound(2));
    set(arrow(3),'xdata',x(2),'ydata',y(2),'udata',0.25*Robot.desired_direc(1),'vdata',0.25*Robot.desired_direc(2));

    drawnow();
end


%% Cleanup - Stop Robot motors, delete serialport
Robot.u.data = zeros(6,1);
SerialCommunication(s1,Robot,192,'u');
delete(s1)
disp("Deleted Serial Port")

% Get Final Run Time
time = round(toc(runTime),1);
disp(['Final run time: ', num2str(time)]);


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

return


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
