% Author: Steven M Hyland
% Date: 02/16/2023
% Copyright @ Steven M Hyland. All rights reserved

%% Got attachment process and manipulation process working. Agents avoid payload when connecting. Now working on single
% agent CoM estimation (like for ICRA)

% clc; clear; close all;
addpath('classes and functions');

%% Problem definition
N = 1;            % Team population
pd = [1; 1; pi];  % Desired point [x;y;q] in m

po_0 = [0; 0; 0]; % Initial position of the object
% po_0 = [0; 0; pi/2];

dpo_0 = [0;0;0];  % Initial velocity of the object

tspan = 0:0.1:30; % Time span of 12 seconds, 0.1s increments

%% ************************** CoM positions from Experiment *****************************************
% CoM = [0; 0];
% CoM = [.02268; -.01512];
% CoM = [.0189; .0189];
CoM = [-0.01512; 0.00756];
% CoM = [-0.01134; -0.01701];


%% 180 and 90 degree attachment, respectively (NOT ANYMORE, NOW JUST CHANGE INITIAL O POSITION)
aPoints = [-0.086 0];
% aPoints = [0 0.086];

%% Visual settings - Generate environment/plot based on supplied limits
scene = generateEnvironment([-0.5 2.2 -0.7 0.7]); % ,'dual');

detachedPoint = [-2; 0.5]; % Detached starting point for agents

Rz = @(q)[cos(q), -sin(q); sin(q), cos(q)];

%% Object (payload) information and initialization
mo = 0.182;         % Payload mass in kg
uk = 2.6;       % Coefficient of kinetic friction between the surface and object
Ro = 0.086;       % Body radius of the object in m
Io = mo*Ro^2;         % Payload inertia in kgm^2

% ObjectBodyPoints = [po_x po_y; po_x+1 po_y; po_x+1 po_y+1; po_x po_y+1];
ObjectBodyPoints = [];

O = payload('bodyPoints',Ro*ObjectBodyPoints,'mass',mo,'inertia',Io,'uk',uk,'bodyColor',scene.O,...
    'bodyRadius',Ro, 'com',CoM);%, 'prev_theta', po_0(3));
O.move(po_0,dpo_0);


%% Obstacle(s) information and initialization
VQ1 = [];
Q(1) = obstacle('Vertices',VQ1,'bodyColor',scene.Q);


%% Robot information and initialization
delta = 0.01;   % Robot radius in m
gamma = 0.0001; % Robot's vision range in m
Kp = 10;        % Proportional gain of the PD controller (Manipulation)
Kd = 2;         % Derivative gain of the PD controller (Manipulation)
fmax = .3;  %0.2 is better, 0.3 for figure     % Maximum force of the robot in N

max_theta = 10; % Threshold for measured rotation
min_theta = 4;  % Threshold to STOP simulation

% Array of length numAgents+1, with last entry empty (e.g. N long)
qq = linspace(0,2*pi,N+1); qq(end) = [];

% Agent attachment points
% aPoints = []; itr = 0;
% while(size(aPoints,1)<N)
%     aPoints = O.getBodyPoint(N+itr);
%     itr = itr+1;
% end

% Calculating bounds tangent to payload at attachment point (technically perpendicular to 'r')
negTan = Rz(-O.p(3)) * Rz(-pi/2) * -aPoints';
negBound = negTan/norm(negTan);
posTan = Rz(-O.p(3)) * Rz(pi/2) * -aPoints';
posBound = posTan/norm(posTan);

% posBound = [1;0];
% negBound = [-1;0];

% Calculating initial push direction TOWARDS CENTROID
init_direc = -aPoints/norm(aPoints);

for i=1:N
    % Create robot object with supplied parameters
    A(i) = robot('ID',i,'delta',delta,'gamma',gamma,'Kp',Kp,'Kd',Kd,'fmax',fmax,'AttachedColor',scene.Aattched,...
        'DetachedColor',scene.Adetached,'AvoidColor',scene.Aavoid,'FmanColor',scene.Fman,'FavoidColor',scene.Favoid, ...
        'p_attach',[aPoints(i,:) pi+qq(i)]','prev_meas', O.p(3),'negBound',negBound,'posBound',posBound,...
        'desired_direc', init_direc', 'max_theta', max_theta, 'min_theta', min_theta);
    
    % Move robot to attachment point on object
    A(i).move([aPoints(i,:) pi+qq(i)]);
%     A(i).move([detachedPoint; 0]); % To begin detached
    A(i).setr(aPoints(i,:),O.p); % Attachment point wrt centroid
end

O.r = A(1).r; % Set object attachment property equal to agent 1's property


%_____________________________________________________________________________________________
%% Simulation
%_____________________________________________________________________________________________

y0 = [po_0;dpo_0]; % Initial object position and velocity

for i=1:N 
   y0(6*i+1 : 6*i+3,1) = A(i).p; % Add three rows consisting of the robot's position
end

%%% TEMP TEST
global Bounds
Bounds = [0, 0, 0, 0, 0];


[t,y] = ode45(@(t,y) systemDynamics(t,y,pd,A,O,Q),tspan,y0);


fprintf('\n\n _____________________ END OF ODE45 ______________________________________\n\n')

%% Reconstruct CoM Location and Bounds
% com_loc = zeros([length(t),2]); % bounds = zeros([length(t),4]);
desired_direction = zeros([length(t),2]);
for n=1:length(tspan)
    com_loc(n,:) = y(n,1:2)' + [cos(y(n,3)) -sin(y(n,3)); sin(y(n,3)) cos(y(n,3))] * CoM;
end

reconstruct_bounds_GLOBAL;

%% Results
% input('Press Enter to Proceed...');

% Plot the obstacles:
for j=1:length(Q)
    Q(j).plot(scene.ax);
end

% Plot the object:
O.plot(scene.ax);

% Plot the agent(s):
for i=1:N
    A(i).plot(scene.ax);
end

% Initialize handle for CoM location point
c = scatter(0,0);


% Move the agent(s) and object and CoM
for n=1:length(t)

    delete(c)
    O.move(y(n,1:3));
    for i=1:N
        position = y(n, 6*i+1 : 6*i+3);
        bound1 = bounds(n,1:2);
        bound2 = bounds(n,3:4);
        direction = desired_direction(n,:);

        A(i).move(position,O,bound1,bound2,direction);
    end
    c = scatter(com_loc(n,1),com_loc(n,2),'ko','MarkerFaceColor','r','MarkerFaceAlpha',1,'DisplayName','CoM');
    
    % Expand plot window when system moves out of frame
    moveWindow;

    pause(0.05);
    legend({'','','','','','','Bounds',''},'FontSize',20);

    if n == 1
        input('enter')
    end
end

disp(A(1).measure_t)


