% Testing dynamic response to low impact of object

clc; clear; close all;

g = 9.8; % gravity
m = 1;% mass
rw = 1; % Known distance from CoM to rear tipping (pushed) edge

rc = sqrt(5); % Unknown to us, just for testing. Attained with pythagoras

phi_dot = sqrt(2*g/rc) % critical angular velocity for object to not topple (tip backwards up until stability point)



rh = sqrt((4*g^2/phi_dot^4) - rw^2) % Calculated (estimated) CoM height

rh_ground_truth = sqrt(rc^2 - rw^2) % Geometrically calculated CoM height ground truth

Impulse = (0.5*m*rc^2*phi_dot^2) % Impulse required to impart enough energy to reach critical topple point