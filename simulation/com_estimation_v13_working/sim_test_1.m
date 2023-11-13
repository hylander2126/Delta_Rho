clc; clear; close all;
addpath('classes and functions');

%% Problem definition
N = 1; % Team population
pd = [2; 1.5; pi/4]; % Desired point [x;y;q] in m

po_0 = [0;0;-pi/4]; % Initial position of the object
dpo_0 = [0;0;0];     % Initial velocity of the object

tspan = [0 18];


%% Visual settings
scene = generateEnvironment([-3 1.5 -1 3]);

%% Object (payload) information and initialization
mo = 1;    % Payload mass in kg
Io = 1;     % Payload inertia in kgm^2
uk = 0.2;     % Coefficient of kinetic friction between the surface and object
Ro = 0.3;   % Body radius of the object

ObjectBodyPoints = [];

O = payload('bodyPoints',Ro*ObjectBodyPoints,'mass',mo,'inertia',Io,'uk',uk,'bodyColor',scene.O,...
    'bodyRadius',Ro*2);
O.move(po_0,dpo_0);


%% Robot information and initialization
delta = 0.05;   % Robot radius in m
gamma = 0.3;    % Robot's vision range in m
Kp = 10;        % Proportional gain of the PD controller (Manipulation)
Kd = 2;         % Derivative gain of the PD controller (Manipulation)
fmax = 5;       % Maximum force of the robot in N

qq = linspace(0,2*pi,N+1); qq(end) = [];

aPoints = []; itr = 0;
while(size(aPoints,1)<N)
    aPoints = O.getBodyPoint(N+itr);
    itr = itr+1;
end

for i=1:N
    A(i) = robot('delta',delta,'gamma',gamma,'Kp',Kp,'Kd',Kd,'fmax',fmax,...
        'AttachedColor',scene.Aattched,'DetachedColor',scene.Adetached,...
        'AvoidColor',scene.Aavoid,'FmanColor',scene.Fman,'FavoidColor',scene.Favoid);
        
    A(i).move([aPoints(i,:) pi+qq(i)]);
    A(i).setr(aPoints(i,:),O.p)
end


%% Simulation

y0 = [po_0;dpo_0];


% O.plot(ax);
% for i=1:N
%     A(i).plot(ax);
% end

for i=1:N
   y0(6*i+1 : 6*i+3,1) = A(i).p;
   % TEMP Setting agent speed manually
   A(i).dp_desired = [0; 0.1; 0];
end

tic
[t,y] = ode45(@(t,y)systemDynamics(t,y,pd,A,O),tspan,y0);
toc
y;

% figure();
% plot(t,y(:,1:6))


%% Results

O.plot(scene.ax);
for i=1:N
    A(i).plot(scene.ax);
end


for n=1:length(t)
    O.move(y(n,1:3));
    for i=1:N
        A(i).move(y(n, 6*i+1:6*i+3),[0;0;0],O);
    end
    drawnow();
    pause(0.1)
end





