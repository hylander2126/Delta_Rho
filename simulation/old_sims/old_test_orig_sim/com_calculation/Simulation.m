% Author: Steven M Hyland
% Date: 04/03/2022
% Copyright @ Steven M Hyland. All rights reserved
clc; clear; close all;
addpath('classes and functions');

%% Problem definition
N = 2; % Team population
pd = [1; 1; pi]; % Desired point [x;y;q] in m

po_x = -1;
po_y = -1;
po_0 = [po_x; po_y; pi/6]; % Initial position of the object
dpo_0 = [0;0;0];     % Initial velocity of the object

tspan = 0:0.1:4;


% Visual settings - Generate environment/plot based on supplied limits
scene = generateEnvironment([-2.5 2.5 -2.5 2.5]);

%% Obstacle(s) information and initialization
% VQ1 = [-2 -1; -1 -1; -1 1; -2 1];
% VQ2 = [0 -1; 1 -1; 1 -3; 0 -3];
VQ1 = [];
VQ2 = [];

Q(1) = obstacle('Vertices',VQ1,'bodyColor',scene.Q);
Q(2) = obstacle('Vertices',VQ2,'bodyColor',scene.Q);

%% Object (payload) information and initialization
mo = 1;    % Payload mass in kg
Io = 1;     % Payload inertia in kgm^2
uk = 0.2;     % Coefficient of kinetic friction between the surface and object
Ro = 0.2;   % Body radius of the object

ObjectBodyPoints = [po_x po_y; po_x+1 po_y; po_x+1 po_y+1; po_x po_y+1];
ObjectBodyPoints = [];

O = payload('bodyPoints',Ro*ObjectBodyPoints,'mass',mo,'inertia',Io,'uk',uk,'bodyColor',scene.O,...
    'bodyRadius',Ro*2);
O.move(po_0,dpo_0);

% TEMP plot the object in initial position
O.plot(scene.ax);

%% Robot information and initialization
delta = 0.05;   % Robot radius in m
gamma = 0.2;    % Robot's vision range in m
Kp = 10;        % Proportional gain of the PD controller (Manipulation)
Kd = 2;         % Derivative gain of the PD controller (Manipulation)
fmax = 5;       % Maximum force of the robot in N

% Array of length numAgents+1, with last entry empty (e.g. N long)
qq = linspace(0,2*pi,N+1); qq(end) = [];

% Agent attachment points - increase N until we get 
aPoints = []; itr = 0;
while(size(aPoints,1)<N)
    aPoints = O.getBodyPoint(N+itr);
    itr = itr+1;
end

for i=1:N
    % Create robot object with supplied parameters
    A(i) = robot('delta',delta,'gamma',gamma,'Kp',Kp,'Kd',Kd,'fmax',fmax,...
        'AttachedColor',scene.Aattched,'DetachedColor',scene.Adetached,...
        'AvoidColor',scene.Aavoid,'FmanColor',scene.Fman,'FavoidColor',scene.Favoid);
    
    % Move robot to attachment point on object
    A(i).move([aPoints(i,:) pi+qq(i)]);
    A(i).setr(aPoints(i,:),O.p);
end

%% Simulation

y0 = [po_0;dpo_0]; % Initial object position and velocity

for i=1:N % Add three rows consisting of the robot's position
   y0(6*i+1 : 6*i+3,1) = A(i).p;
end

[t,y] = ode45(@(t,y) systemDynamics(t,y,pd,A,O),tspan,y0);


%% Results
% input('Press Enter to Proceed...');

% Plot the obstacles:
% for j=1:length(Q)
%     Q(j).plot(scene.ax);
% end

% Plot the object:
% O.plot(scene.ax);

% Plot the agent(s):
for i=1:N
    A(i).plot(scene.ax);
end

% Axes for desired point
quiver(pd(1)*[1 1],pd(2)*[1 1],...
    O.bodyRadius*[cos(pd(3)) -sin(pd(3))],...
    O.bodyRadius*[sin(pd(3)) cos(pd(3))],0,'Color',scene.pd);

% Move the agent(s) and object
for n=1:length(t)
    O.move(y(n,1:3));
    for i=1:N
        position = y(n, 6*i+1 : 6*i+3);
        A(i).move(position,pd,O);
    end
    pause(0.1);
end





