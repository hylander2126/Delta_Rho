clc; close all;
addpath('classes and functions');
%% Version 13 - All finished. Have to run intersection calculation seperately
%% for now, maybe implement in a future version.

%% Problem definition
%________________________________________________________________________
tic                 % For time keeping
N = 1;              % Team population

po_0 = [0;0;0];     % Initial position of the object
vo_0 = [0;0;0];     % Initial velocity of the object
ao_0 = [0;0;0];     % Initial acceleration of the object

com = [-0.2;0.3];   % Desired object com wrt obj geometric center
% com = [];

a_desired = 1;      % Desired (initial) acceleration of the agent

dt = 0.1;           % Size of time steps
tspan = 0:dt:20;    % Time span with step size dt

method = 'optical'; % Define optical or force method of determining CoM


%% FOR PLOTTING

full_sim = true;   % Whether to plot every N steps or delete the handle
alphaSteps = 1;
alpha = 1; 
if ~full_sim
    alphaSteps = 10;% Number of steps to skip before plotting object/agent
end
init_pos = 2;       % Which position to place robot (qq determines pos)

%% Object (payload) information and initialization
%________________________________________________________________________
mo = 1;     % Payload mass in kg
Io = 1;     % Payload inertia in kgm^2
uk = 0.2;   % Coefficient of kinetic friction between the surface and object
Ro = 0.3;   % Body radius of the object

ObjectBodyPoints = []; % No body points creates a circle with radius Ro

% Create a triangular object
% width = 4;
% height = 4;
% ObjectBodyPoints = [0 0; width 0; width/2 sqrt(width^2 - (width/2)^2)];
% xC = sum(ObjectBodyPoints(:,1),"all")/3;
% yC = sum(ObjectBodyPoints(:,2),"all")/3;
% ObjectBodyPoints = [ObjectBodyPoints(:,1) - xC, ObjectBodyPoints(:,2) - yC];

% Create convex object
% ObjectBodyPoints = [0.3750 -0.3750; 0.7500 -0.3750; 1.7000 0.5000; ...
%     0.3750 1.8000; -0.5000 1.2000; -1.4250 0.6000; -0.9000 0; ...
%     -1.4250 -0.9000; -0.7500 -0.7500; -0.6000 -1.0500; 0.2250 -0.9000];

% plot(ObjectBodyPoints(:,1),ObjectBodyPoints(:,2))
% return
O = payload('bodyPoints',Ro*ObjectBodyPoints,'mass',mo,'inertia',Io, ...
    'uk',uk,'bodyColor',[0.9020, 0.7333, 0.9725],'bodyRadius',Ro*2, ...
    'com',com);
O.move(po_0,vo_0,ao_0); % Set the object's initial pos, vel, & accel

%% Robot information and initialization
%________________________________________________________________________
delta = 0.05;   % Robot radius in m
fmax = 5;       % Maximum force of the robot in N
arrowSize = 1;  % Bounds arrow size

qq = linspace(0,2*pi,N+1); qq(end) = [];

aPoints = []; itr = 0;
while(size(aPoints,1)<N+1)
    aPoints = O.getBodyPoint(N+itr);
    itr = itr+1;
end

for i=1:N
    A(i) = robot('delta',delta,'fmax',fmax,'method',method,'arrowSize', ...
        arrowSize,'a_desired',a_desired,'prevTheta',O.p(3),'measuredTheta',O.p(3));

    A(i).move([aPoints(init_pos,:) qq(i)], vo_0, [0;0;0]); % Rot WAS pi+qq(i)
    A(i).setr(aPoints(init_pos,:), O.p, com)
end

%% Record robot initial position for CoM calculation
a_p_init(init_pos,:) = A.p(1:2)';

%% Set initial search bounds based on object edges
%________________________________________________________________________
O_bodyPoints = O.getBodyPoint(200);
A_pos = A.p(1:2)';
k = dsearchn(O_bodyPoints,A_pos); % Index of nearest point
if O_bodyPoints(k,:) == A_pos
    O_bodyPoints(k,:) = []; % If verbatim A_pos remove entry, try again
    k = dsearchn(O_bodyPoints, A_pos);
end
% Get first bound and make wrt A_pos
firstBound = O_bodyPoints(k,:) - A_pos;
firstBound = firstBound/norm(firstBound);

% Check direction and angle of first bound and set the second bound
r = O.p(1:2) - A.p(1:2); % Vector from A to O
c = cross([r' 0], [firstBound 0]);
theta = sign(c(3))*180/pi*atan2(norm(c),dot(r,firstBound));

%% MUST CHANGE FROM 180 TO 2*THETA FOR DIFFERENT OBJECTS
% Mirror firstBound ALMOST to 180 degrees
% secondBound = (Rzd(sign(c(3))*180)*firstBound')';
secondBound = (Rzd(sign(c(3))*theta*2)*firstBound')';

if sign(c(3)) > 0
    A.posBound = secondBound';
    A.negBound = firstBound';
else
    A.posBound = firstBound';
    A.negBound = secondBound';
end
% return
%% Simulation
%%________________________________________________________________________
disp('Running Simulation ...')

% Set initial conditions
y0 = [po_0; vo_0]; % Object position & velocity
for i=1:N
    y0(6*i+1 : 6*i+3,1) = A(i).p; % Agent position
    y0(9*i+1 : 9*i+3,1) = A(i).dp; % Agent velocity
end
% y0 is 12x1 vector consisting of obj pos/vel and agent pos/vel
y0 = y0';
% Get initial dy0 ~"action"
dy0 = systemDynamics(0,y0,A,O)';

y(1,:) = y0';
dy(1,:) = dy0';
com_loc = zeros(size(tspan,1), 2);
negBound = zeros(size(tspan,1), 2);
posBound = zeros(size(tspan,1), 2);

% Calculate dynamics over time using kinematic eq. of motion
%________________________________________________________________________
for i = 2:length(tspan) % Skip t=0 already have initial conditions
    t = tspan(i);
    % Get PREVIOUS ACTING velocity, acceleration at step i-1
    vo_0 = dy(i-1,1:3);
    ao_0 = dy(i-1,4:6);
    va_0 = dy(i-1,7:9);
    aa_0 = dy(i-1,10:12);
    % Get PREVIOUS position, velocity at step i-1
    po_0 = y(i-1,1:3);
    pa_0 = y(i-1,7:9);
    % Calculate current y at step "i"
    y(i,1:3) = po_0 + vo_0*dt + 0.5*ao_0*dt^2;
    y(i,4:6) = vo_0 + ao_0*dt;
    y(i,7:9) = pa_0 + va_0*dt + 0.5*aa_0*dt^2;
    y(i,10:12) = va_0 + aa_0*dt;
    % Calculate NEW ACTING velocity, acceleration at step i
    dy(i,:) = systemDynamics(t,y(i,:),A,O); % Get deriv of y

    % CoM and obj bound information for plotting only
    com_loc(i,:) = O.p(1:2) + Rzd(rad2deg(O.p(3))) * O.com;
    negBound(i,:) = A.negBound;
    posBound(i,:) = A.posBound;
    % Set stop condition
    if contains(A.mode,'stop')
        tspan(i:end) = []; % Truncate rest 
        break
    end
end
%________________________________________________________________________

t_y = round(y,3);
t_dy = round(dy,3);

% Truncate t_y and d_y to same length as tspan
t_y(size(tspan,2)+1:end, :) = []; 
t_dy(size(tspan,2)+1:end, :) = [];
com_loc(1,:) = com;

%% Visual settings
%________________________________________________________________________
scene = generateEnvironment([-5 5 -4 4]); %, 'dual'); % 'dual' or 'big' args

%________________________________________________________________________
%% ************** RESULTS ****************
%________________________________________________________________________
O.plot(scene.ax);
for i=1:N
    A(i).plot(scene.ax);
end

%% Record measured CoM wrt space frame for line intersection plot/calc
measuredCoM(init_pos,:) = A.measuredCoM';

%% Run Simulation Results
for n=1:length(tspan)
    if ~full_sim
        alpha = (n/length(tspan))^4; % Tailored to exponentially increase
    end
    if rem(n,alphaSteps) == 0
        % Change opacity over time
        O.move(y(n,1:3), y(n,4:6),dy(n,4:6), alpha, full_sim);
        c = scatter(com_loc(n,1),com_loc(n,2),'ko','MarkerFaceColor',...
            'r','MarkerFaceAlpha',alpha,'DisplayName','CoM');
                
        % Add the figure's title with elapsed time and legend
        title(['Center of Mass Estimation']); % | Elapsed Time: ' num2str(round(toc,2)) 's'])
        legend([c,A.handle(7)]),set(legend,'fontsize',20);

        A.move(y(n, 6*i+1:6*i+3), y(n,9*i+1:9*i+3), dy(n,9*i+1:9*i+3), ...
            negBound(n,:), posBound(n,:), O, alpha, full_sim);
        
        %% Expand plot window when system moves out of frame
        moveWindow;

        %% Get frame for saving video of simulation
        frames(n) = getframe(gcf);
        drawnow();
        pause(0.01)
        if full_sim
            if n<length(tspan)-1
                delete(c) % Delete CoM marker except for final iteration
            end
        end
    end
end


%% Plot velocity vs acceleration over time
%__________________________________________________________________________
plotDynamics = false;
if plotDynamics
    figure;
    y1 = t_y(:,10);
    y2 = t_y(:,11);
    y3 = t_dy(:,10);
    y4 = t_dy(:,11);
    plot(tspan,y1,'k',tspan,y2','b',tspan,y3,'k--',tspan,y4,'b--', LineWidth=8);
    set(gca,'FontSize',40)
    xlabel 'time (s)'
    ylabel 'mm'
    % title('Agent Dynamics Over Time'); 
    legend({'xVel','yVel','xAcc','yAcc'})
end

%% Save figure video to file
%__________________________________________________________________________
% create the video writer with 1 fps
vidTitle = ['vid_sim_pos' int2str(init_pos) '.mp4'];
writerObj = VideoWriter(vidTitle, 'MPEG-4');
writerObj.FrameRate = 10;
% set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(frames)
    % convert the image to a frame
    F = frames(i);
    writeVideo(writerObj, F);
end
% close the writer object
close(writerObj);


%%_________________________________________________________________________
%% ************* RUN THIS COMMAND WHEN FINISHED ***************************
%%_________________________________________________________________________
% fileName = 'com_sim_fig1';
% exportgraphics(gca,[fileName '.png'],'Resolution',300)
% saveas(gca,[fileName '.fig'])


%% Rotation matrix about z axis IN DEGREES
%________________________________________________________________________
function R = Rzd(q)
R = [cosd(q) -sind(q); sind(q) cosd(q)];
end
