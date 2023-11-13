% Author: Steven M Hyland
% Date: 05/19/2023

% v1 - Works, but takes forever to run. Diff eq. too stiff to solve quickly. Uncertainty added to force sensor reading.
% TODO - Split up ode45 into multiple calls every time a force reading is taken in order to change our direction outside
% the dynamics loop. aka split up the ode45 calls and the control loop. Also add uncertainty into our CoM guess.

%% Got attachment process and manipulation process working. Agents avoid payload when connecting. Now working on single
% agent CoM estimation (like for ICRA)

% clc; clear; close all;
addpath('classes and functions');
Rz = @(q)[cos(q), -sin(q); sin(q), cos(q)]; % Anonymous function rot mat about z axis


%% Problem definition
N = 1;            % Team population
pd = [1; 1; pi];  % Desired point [x;y;q] in m

po_0 = [0; 0; 0]; % Initial position of the object
% po_0 = [0; 0; 3*pi/4];

dpo_0 = [0;0;0];  % Initial velocity of the object

tspan = 0:0.1:15; % Time span, 0.1s increments

%% ************************** CoM positions from Experiment *****************************************
% CoM = [0; 0];
CoM = [.02268; -.01512];
% CoM = [.0189; .0189];
% CoM = [-0.01512; 0.00756];
% CoM = [-0.01134; -0.01701];
% CoM = [-0.3;0.3];


%% 180 and 90 degree attachment, respectively (NOT ANYMORE, NOW JUST CHANGE INITIAL O POSITION)
aPoints = [-0.086 0];
aPoints = Rz(deg2rad(89))*aPoints';
aPoints = aPoints';
% aPoints = [0 0.086];

%% Visual settings - Generate environment/plot based on supplied limits
scene = generateEnvironment([-0.5 2.2 -0.7 0.7], 'big'); % ,'dual');

% OPTIONAL if starting detached from payload
detachedPoint = [-2; 0.5]; % Detached starting point for agents


%% Object (payload) information and initialization
mo = 0.182;      % Payload mass in kg
uk = 2.6;        % Coefficient of kinetic friction between the surface and object
Ro = 0.086;      % Body radius of the object in m
Io = mo*Ro^2;    % Payload inertia in kgm^2

% ObjectBodyPoints = [po_x po_y; po_x+1 po_y; po_x+1 po_y+1; po_x po_y+1];
ObjectBodyPoints = [];

O = payload('bodyPoints',Ro*ObjectBodyPoints,'mass',mo,'inertia',Io,'uk',uk,'bodyColor',scene.O,...
    'bodyRadius',Ro, 'com',CoM);
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

% Array of length numAgents+1, with last entry empty (e.g. N long)
qq = linspace(0,2*pi,N+1); qq(end) = [];

% Agent attachment points
% aPoints = []; itr = 0;
% while(size(aPoints,1)<N)
%     aPoints = O.getBodyPoint(N+itr);
%     itr = itr+1;
% end

% Calculating initial push direction TOWARDS CENTROID
init_direc = reshape(-aPoints/norm(aPoints), [2,1]);

% Creating pregenerated sensor 'noise' vector
sensor_noise_pregen = 3*randn(50,1);
% sensor_noise_pregen = [1;2;3;4;5;6;7;8;9;10;11;12;13;14;15;16;17;18;19;20;21;22;23;24;25;26;27;28;29;30;31;32;33;34;35];

for i=1:N
    % Create robot object with supplied parameters
    A(i) = robot('ID',i,'delta',delta,'gamma',gamma,'Kp',Kp,'Kd',Kd,'fmax',fmax,'AttachedColor',scene.Aattched,...
        'DetachedColor',scene.Adetached,'AvoidColor',scene.Aavoid,'FmanColor',scene.Fman,'FavoidColor',scene.Favoid, ...
        'p_attach',[aPoints(i,:) pi+qq(i)]','prev_meas', O.p(3), 'prev_direc', init_direc,...
        'desired_direc',init_direc','f_read',[0;0],'noise_pregen',sensor_noise_pregen,'counter',1);
    
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

% control_loop_resolution = 0.5; % Seconds between control loop 
% t = zeros(size(tspan,2),1);
% y_f = y0';
% while t(end) < tspan(end)
%     disp('next ode call')
% %     t_i = linspace(0,control_loop_resolution, 50)' + t(end);
%     y0 = y_f(end, :);
%     tspan = linspace(0,control_loop_resolution, 50);
% 
%     [t_i,y_i] = ode45(@(t,y) systemDynamics(t,y,pd,A,O,Q),tspan,y0) % Only ouputs a total of 50 for y
%     y_f = [y_f; y_i];
%     t = [t; t_i]; % Overall outputs 351
% end

% y_f

% y = y_f(2:end,:);

% return

tic
[t,y] = ode45(@(t,y) systemDynamics(t,y,pd,A,O,Q),tspan,y0);

fprintf('\n\n _____________________ END OF ODE45 IN %4.2f s____________________________________\n\n', toc)


% Getting velocity information
dy = zeros(size(y));
for i = 1:length(t)
    dy(i,:) = systemDynamics(t(i),y(i,:),pd,A,O,Q);
end


%% Reconstruct CoM Location
% com_loc = zeros([length(t),2]); % bounds = zeros([length(t),4]);
desired_direction = zeros([length(t),2]);
for n=1:length(tspan)
    com_loc(n,:) = y(n,1:2)' + [cos(y(n,3)) -sin(y(n,3)); sin(y(n,3)) cos(y(n,3))] * CoM;
end


%% Results
% input('Press Enter to Proceed...');
initialize_plot;

% Move the agent(s) and object and CoM
for n=1:length(t)
    delete(c)
    O.move(y(n,1:3));
    for i=1:N
        position = y(n, 6*i+1 : 6*i+3);
        direction = desired_direction(n,:);
        f_read = -dy(n,1:2);
        A(i).move(position, O, direction, f_read);
    end
    c = scatter(com_loc(n,1),com_loc(n,2),'ko','MarkerFaceColor','r','MarkerFaceAlpha',1,'DisplayName','CoM');
    
    % Expand plot window when system moves out of frame
    moveWindow;

    pause(0.05);
%     legend({'','','','','','','Bounds',''},'FontSize',20);

%     if n == 1
%         input('enter')
%     end
end

disp(A(1).measure_t)


