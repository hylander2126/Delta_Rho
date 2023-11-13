% Collective manipulation simulator
%
% Author:   Siamak Ghorbani Faal
%           sghorbanifaal@wpi.edu
%           Last update: 8/31/2015
%
% External functions:
%           construct_jacobian
%           system_dynamics
%           animate
%
% Descriptions:
%
%       {O} -> The coordinate frame that is attached to the center of mass
%              of the object. 
%
% Parameters:
%       
%       mo: Mass of the object [Kg]
%       Io: Mass moment of inertia of the object [Kgm^2]
%        R: Radious of the object
%        N: Number of agents
%        W: Object weight = 9.81*mo
%     Pi_o: Positions of the agents that are connected to the object and
%           defined in {O} 
%           Pi_o = [x1 x2 ... xN;
%                   y1 y2 ... yN]
%        J: The complete jacobian of the system
%           J = [   1    0    1    0 ...    1    0;
%                   0    1    0    1 ...    0    1;
%                -ry1  rx1 -ry2  rx2     -ryN  rxN]
%       J_: The local jacobian that is defined for (or assumed by) each 
%           agent.
%           J_(:,:,i) = [   1    0    1    0;
%                           0    1    0    1;
%                        -ryi  rxi -ay2  ax2];
%
%      X0: Initial conditions for the object
%           X0 = [x0;y0;q0]
%      Xd: Desired position of the object
%           Xd = [xd;yd;qd]
%      us: Coefficient of static friction
%      uk: Coefficient of kinetic friction
%      fL: Force limits. The output of the controller is saturated by the
%          limits defined by fL
%          fL = [LowLimit HighLimit];
%
%
%% Initializations
clc; clear all; close all;

rng(3);

global mo Io R Pi_o Rz J J_ X0 Xd us uk N W fL


%% User defined parameters

Xd = [1;1;1];

mo = 1;
Io = 1;
R = 2;

us = 0;
uk = 0;

q = [0 90 100 220];
Pi_o = R*[cosd(q); sind(q)];
%Pi_o = [2 0 ; 0 -2];

fL = [-.4 .4];


%% Automatically generated parameters

W = mo*9.81;

Rz = @(q) [cos(q) -sin(q); sin(q) cos(q)];
      
N = size(Pi_o,2);

J = construct_jacobian(Pi_o);

for i=1:N
    J_(:,:,i) = construct_jacobian(Pi_o(:,i));
end

%% Simulation

X0 = zeros(6,1);

tspan = 0:0.05:10;

options = odeset('RelTol',1e-3,'AbsTol',1e-4,'Events',@instability);
tic
[t,X] = ode15s(@system_dynamics,tspan,X0,options);
toc

%% Animation
frames = animate(t,X,0);


%% Plots
figure();
axis_name{1} = 'x_o [m]'; axis_name{2} = 'y_o [m]'; axis_name{3} = '\theta_o [rad]';
for i=1:3
    subplot(3,1,i)
    plot(t,X(:,i),'b',[t(1), t(end)],[Xd(i) Xd(i)],':b');
    %plot(t,X(:,i+3),'b')
    %ylim([-0.2 1.2]);
    ylabel(axis_name{i});
end
xlabel('Time [sec]');

% figure();
% for i=1:N
%     for n=1:length(t)
%         f(:,n) = controller(X(n,:)',J(:,:,i));
%     end
%     subplot(N,1,i);
%     plot(t,f);
%     legend('f_x','f_y');
% end
% 
% for n=1:N
%     Jp(:,:,n) = pinv(J_(:,:,n));
%     k(2*n-1:2*n,:) = Jp(1:2,:,n);
% end
% 
% 
% A = J(:,:,1)*k
%  
% eig(A)
% 
% T = eye(3)*N;
% 
% for i=1:N
%     T(1,3) = T(1,3) - Pi_o(2,i)/(Pi_o(1,i)^2 + Pi_o(2,i)^2);
%     T(2,3) = T(2,3) + Pi_o(1,i)/(Pi_o(1,i)^2 + Pi_o(2,i)^2);
%     T(3,1) = T(3,1) - Pi_o(2,i);
%     T(3,2) = T(3,2) + Pi_o(1,i);
% end
% 
% T = T/2
% A-T

