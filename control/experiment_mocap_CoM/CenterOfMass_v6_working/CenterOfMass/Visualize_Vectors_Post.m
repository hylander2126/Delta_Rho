%% TEST file to visualize vectors after running experiment
clc;

Rz = @(q)[cosd(q),-sind(q);sind(q),cosd(q)];

figure();
hold on


%% Draw Circle Payload
circ = linspace(0, 2*pi);
diam = 172;
circX = diam/2*cos(circ);
circY = diam/2*sin(circ);

patch(circX, circY, 'k')
axis equal


%% Plot ACTUAL CoM
actualCoM = [-15.12; 7.56];
plot(actualCoM(1),actualCoM(2),'pentagramr','MarkerSize',15)


%% Draw first Arrow
quiver(-86, 0, diam*finalMeasurement_180(1), diam*finalMeasurement_180(2),'--m')
axis equal

%% Draw second Arrow - SIGN FLIPPED SINCE IT IS 180 DEGREES ROTATED
% quiver(86, 0, -diam*finalMeasurement_360(1), -diam*finalMeasurement_360(2))

% ACTUALLY NOW trying from 90 degrees... no need to flip signs or rotate
quiver(0, 86, diam*finalMeasurement_90(1), diam*finalMeasurement_90(2),'--m')
axis equal



%% Plot and Compute Interesection
CoM = getIntersection(diam, finalMeasurement_180, finalMeasurement_90)';
plot(CoM(1),CoM(2), '.g','MarkerSize',30)
