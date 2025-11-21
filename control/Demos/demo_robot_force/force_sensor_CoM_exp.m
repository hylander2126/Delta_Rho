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
Robot.ID             = N;
% Robot.K_wp           = 5;        % Rotation Proportional Gain
% Robot.K_wd           = 20;       % Rotation Pseudo-Derivative Gain
Robot.v_desired      = 170;      % Desired velocity magnitude
% Agent motor values
Robot.u.data         = [0;0;0];
Robot.u.type         = 'uint8';
Robot.u.convertor    = @(x)uint8(x);
% Robot Jacobian (new wheels 02/11/23) 
Robot.J_r            = construct_robot_jacobian;
% Robot resting sensor values
Robot.rest_angles    = [0;0];
Robot.rest_ee        = 0;
% Robot PD control parameters
Robot.de             = [0;0];
Robot.f_prev         = 0;
Robot.t_prev         = 0;


%% Stopwatch for checking minimal rotation threshold
run_time = tic;
global force_data PD_data;
PD_data = [];
force_data = [];


%% ****** RUN EXPERIMENT *******
while toc(run_time) < 15
    %% Assign robot motor torques | k=1 front left; k=2 rear; k=3 front right
    Robot = update_robot(toc(run_time), Robot, s1);
    pause(0.1)
end


%% Cleanup - Stop Robot motors, delete serialport
Robot.u.data = zeros(6,1);
SerialCommunication(s1,Robot,192,'u');
delete(s1)
return

%% ***** Run Experiment ******
%__________________________________________________________________________
% while(~get(stop,'value') && ~Robot.stop)
% 
%     % Update Robot and Payload properties: get x,y,z,Q,R and determine rotation
%     optitrack_read_rigidbody_frame;
%     update_robot_payload_positions;
% 
% %     disp(Robot.desired_direc);
% 
% 
%     %% Update robot's desired heading and take measurement
%     t = toc(runTime);
%     Robot = updateRobot(t,Robot,payload,s1);
%     
% 
%     % Send command to Robot
%     SerialCommunication(s1,Robot,192,'u');
% 
%     % Update plot arrow information
%     set(circle(1),'xdata',circX + x(1), 'ydata', circY + y(1));
%     set(arrow(1),'xdata',x(2),'ydata',y(2),'udata',0.25*Robot.negBound(1),'vdata',0.25*Robot.negBound(2));
%     set(arrow(2),'xdata',x(2),'ydata',y(2),'udata',0.25*Robot.posBound(1),'vdata',0.25*Robot.posBound(2));
%     set(arrow(3),'xdata',x(2),'ydata',y(2),'udata',0.25*Robot.desired_direc(1),'vdata',0.25*Robot.desired_direc(2));
% 
%     drawnow();
% end


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
