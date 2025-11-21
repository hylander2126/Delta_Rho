%% Fkin and Ikin of 5-Bar linkage force sensor mechanism. Force will be 
% applied at end effector and we will get readings at the two base links 
% through two potentiometers.

%% Forward Kinematics

clc; clear; close all;

% Define link lengths
l1 = 20;
l2 = 20;
l3 = 35;
l4 = 35;
l5 = 25;

% Adjust time interval
t = 0 : .05 : 10;
T = 0 : .05 : 9.975;

% Angular velocities of links
omega1 = 2;
omega2 = 1;

% Calculate two theta angles
theta1 = omega1*t;
theta4 = omega2*t;

% Assume point A is at origin, point E is at x-axis
A = [0;0];
E = l5 * [1;0];

B = [l1*cos(theta1); l1*sin(theta1)];
D = [l5+l4*cos(theta4); l4*sin(theta4)];




b  = 10;
b1 = pi/4;

a1 = 17*pi/180;
a2 = 220*pi/180;

P1 = [10;10];
P2 = [10;-10];


P3 = P1 + l1*[cos(a1);sin(a1)];
P4 = P2 + l2*[cos(a2);sin(a2)];


s = norm(P3 - P4);

xi_1 = atan2(P3(2) - P4(2),P3(1) - P4(1));
xi_2 = acos((l3^2 + s^2 - l4^2)/(2*l3*s));


a3 = pi + xi_1 + xi_2 - a1;

P5 = P3 + l3*[cos(a1 + a3);sin(a1 + a3)];
Pc = P5 + b*[cos(a1 + a3 + b1); sin(a1 + a3 + b1)];


fprintf('alpha1 = %.4f \nalpha2 = %0.4f\n\n', [a1 a2]*180/pi);
fprintf('P1 = [%.4f,%.4f]  ---  P3 = [%.4f,%.4f]\n',[P1' P3']);
fprintf('P2 = [%.4f,%.4f]  ---  P4 = [%.4f,%.4f]\n',[P2' P4']);
fprintf('P5 = [%.4f,%.4f]  ---  Pc = [%.4f,%.4f]\n',[P5' Pc']);


%% Inverse Kinematics

clc; clear; close;

l1 = 20;
l2 = 20;
l3 = 30;
l4 = 30;

P1 = [10;10];
P2 = [10;-10];

P5 = [45;-20];

s1 = norm(P5 - P1);
s2 = norm(P5 - P2);



a1 = atan2(P5(2)-P1(2),P5(1)-P1(1)) + acos((l1^2 + s1^2 - l3^2)/(2*l1*s1));
a2 = atan2(P5(2)-P2(2),P5(1)-P2(1)) - acos((l2^2 + s2^2 - l4^2)/(2*l2*s2));


fprintf('alpha1 = %.4f \nalpha2 = %0.4f\n\n', [a1 a2]*180/pi);



