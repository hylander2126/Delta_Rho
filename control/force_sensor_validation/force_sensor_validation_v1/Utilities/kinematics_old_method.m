%% Fkin and Ikin of 5-Bar linkage force sensor mechanism. Force will be 
% applied at end effector and we will get readings at the two base links 
% through two potentiometers.

%% Forward Kinematics

clc; clear; close all;
% Define spring information
tau_given = 0.042; % Mft. given spring torque
theta_given = 90*pi/180 % Mft. given angle for torque value

k = tau_given/theta_given % Calculated spring constant

% Define link lengths
l1 = 2;
l2 = 3.5;
l3 = 3.5;
l4 = 2;
l5 = 2.5;

% Adjust time interval
t = 0 : .05 : 10;
T = 0 : .05 : 9.975;

% Angular velocities of links
omega1 = 2;
omega2 = 1;

% Calculate two theta angles
theta1 = omega1*t; % linspace(0,2*pi,100);
theta4 = omega2*t; % linspace(0,pi,100);
% theta1 = theta1(25)
% theta4 = theta4(25)

% Assume point A is at origin, point E is at x-axis
A = [0;0];
E = l5 * [1;0];

B = [l1*cos(theta1); l1*sin(theta1)];
D = [l5+l4*cos(theta4); l4*sin(theta4)];

% Mechanism parameters
a = 2*l3*l4*sin(theta4) - 2*l1*l3*cos(theta1);
b = 2*l3*l5 - 2*l1*l3*cos(theta1) + 2*l3*l4*cos(theta4);
c = l1^2 - l2^2 + l3^2 + l4^2 + l5^2 - l1.*l4.*sin(theta1).* ...
    sin(theta4) - 2*l1.*l5.*cos(theta1) + 2*l4.*l5.*cos(theta4) - ... 
    2*l1.*l4.*cos(theta1).*cos(theta4);

% Get angles of second and third links
theta3 = real(2*atan((a+sqrt(a.^2 + b.^2 - c.^2))./(b-c)));
theta2 = asin((l3*sin(theta3) + l4*sin(theta4) - l1*sin(theta1))./l2);

% Define point C
Cx = l1*cos(theta1) + l2*cos(theta2);
Cy = l1*sin(theta1) + l2*sin(theta2);
C = [Cx;Cy];

Cx_alt = l5 + l4*cos(theta4)  + l3*cos(theta3);
Cy_alt = l4*sin(theta4) + l3*sin(theta3);
C_alt = [Cx_alt; Cy_alt];
% C = C_alt;

%% Plot position of C for array of test angles
for i = 1:length(T)
    ani = subplot(1,1,1);
    
    % Small circles to show joints clearly
    p1_circ = circle(A',.1);
    p2_circ = circle(B(:,i)',.1);
    p3_circ = circle(C(:,i)',.1);
    p4_circ = circle(D(:,i)',.1);
    p5_circ = circle(E',.1);

    % Draw linkages
    AB_bar = line([A(1) B(1,i)], [A(2) B(2,i)]);
    BC_bar = line([B(1,i) C(1,i)], [B(2,i) C(2,i)]);
    CD_bar = line([C(1,i) D(1,i)], [C(2,i) D(2,i)]);
    DE_bar = line([E(1) D(1,i)], [E(2) D(2,i)]);

    % Adjust axes
    axis(ani,'equal');
    set(gca,'XLim',[-5 5], 'YLim',[-5 5])

    pause(0.05);

    if i < length(T)
        delete(p1_circ);
        delete(p2_circ);
        delete(p3_circ);
        delete(p4_circ);
        delete(p5_circ);
        delete(AB_bar);
        delete(BC_bar);
        delete(CD_bar);
        delete(DE_bar);
    end

end

function h = circle(c,r)
hold on
x = c(1);
y = c(2);
th = 0:pi/50:2*pi;
xunit = real(r * cos(th) + x);
yunit = real(r * sin(th) + y);
h = plot(xunit, yunit,'Color','b');
hold off
end

