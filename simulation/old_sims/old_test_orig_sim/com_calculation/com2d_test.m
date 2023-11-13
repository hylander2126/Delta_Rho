clc; clear; close all;
addpath('classes and functions');

%% Problem definition
N = 1; % Team population
pd = [-4; 1.5; pi/4]; % Desired point [x;y;q] in m

po_x = -1;
po_y = -1;
po_0 = [po_x; po_y; 0]; % Initial position of the object
dpo_0 = [0;0;0];     % Initial velocity of the object

tspan = [0 4];

%% Generate Environment (plot limits) 
scene = generateEnvironment([-10 4 -4 6],'dual'); % REMOVE DUAL ARG IF 1 MONITOR


%% Obstacle(s) (UNUSED AT THIS TIME)
VQ1 = [];
Q = obstacle('Vertices',VQ1,'bodyColor',scene.Q);

%% Object (payload) information and initialization
mo = 1;    % Payload mass in kg
Io = 1;     % Payload inertia in kgm^2
uk = 0.2;     % Coefficient of kinetic friction between the surface and object
Ro = 0.4;   % Body radius of the object

% ObjectBodyPoints = [0.25 -0.25; 0.5 -0.25; 0.95 0.4; 0.25 0.8; -0.4 0.8;
%     -0.95 0.4; -0.6 0; -0.95 -0.6; -0.5 -0.5; -0.4 -0.7; 0.15 -0.6];

% ObjectBodyPoints = [po_x po_y; po_x+1 po_y; po_x+1 po_y+1; po_x po_y+1];
ObjectBodyPoints = []; % Draw a circle with radius = Ro

O = payload('bodyPoints',Ro*ObjectBodyPoints,'mass',mo,'inertia',Io,'uk',uk,'bodyColor',scene.O,...
    'bodyRadius',Ro*2);
O.move(po_0,dpo_0); % Move to initial object position


%% Robot information and initialization
delta = 0.05;   % Robot radius in m
gamma = 0.3;    % Robot's vision range in m
Kp = 10;        % Proportional gain of the PD controller (Manipulation)
Kd = 2;         % Derivative gain of the PD controller (Manipulation)
fmax = 5;       % Maximum force of the robot in N


qq = linspace(0,2*pi,N+1); qq(end) = []; % Circular linspace for N agents

aPoints = []; itr = 0;
while(size(aPoints,1)<N)
    aPoints = O.getBodyPoint(N+itr); % Get x,y attachment point for N agents
    itr = itr+1;
end

for i=1:N
    A(i) = robot('delta',delta,'gamma',gamma,'Kp',Kp,'Kd',Kd,'fmax',fmax,...
        'AttachedColor',scene.Aattched,'DetachedColor',scene.Adetached,...
        'AvoidColor',scene.Aavoid,'FmanColor',scene.Fman,'FavoidColor',scene.Favoid);

    % Move each agent to attach pt (x,y) and AGENT rotation (q)
    A(i).move([aPoints(i,:) qq(i)+pi]); 
    % Set agent position relative to object Center
    A(i).setr(aPoints(i,:),O.p)
end

%% Testing just pushing object



%% Simulation

y0 = [po_0;dpo_0]; % Set initial object position and velocty


% O.plot(ax);
% for i=1:N
%     A(i).plot(ax);
% end

F0 = [1; 1]; % Initial agent force (arbitrary for now)
M0 = 0; % Initial agent moment

for i=1:N
   y0(6*i+1 : 6*i+3, 1) = A(i).p; % Set initial agent position
   A(i).f = F0;
   A(i).m = M0;
end

initialY = [y0' A(1).f' A(1).m']

% TEMP SETTING tspan TO SEE MORE MOTION
tspan = [0 10];

% tic
[t,y] = ode45(@(t,y) systemDynamics(t,y,pd,A,O,Q),tspan,y0);
% toc

recallY = ['Ox     ' ' Oy     ' '     Otheta   ' '   Odx   ' ...
    '   Ody   ' ' OdTheta   ' '   Ax   ' '   Ay   ' ...
    '     Atheta  ']
yNow = y

%% Results

% Q.plot(scene.ax);
O.plot(scene.ax);
for i=1:N
    A(i).plot(scene.ax);
end


% Simulate the environment
% fArrow = quiver(0, 0, 0, 0);
vArrow = quiver(0, 0, 0, 0);
for n=1:length(t)
    objPos = y(n,1:3);
    objVel = y(n,4:5);
    actualObjVel = O.dp;
    O.move(y(n,1:3));
    for i=1:N % For each agent move to position defined by attach pt to obj
        agentPos = y(n, 6*i+1:6*i+3);
        A(i).move(y(n, 6*i+1:6*i+3),pd,O,Q);
%         delete(fArrow) % Delete previous force arrow
        delete(vArrow) % Delete previous velocity vector
%         fArrow = quiver(A(i).p(1), A(i).p(2), -2.7986, -4.1434, 0, Color='b');
        vArrow = quiver(O.p(1), O.p(2), y(n, 4), y(n, 5), 0, Color='g');
    end
    pause(0.1)    
    drawnow();
end





