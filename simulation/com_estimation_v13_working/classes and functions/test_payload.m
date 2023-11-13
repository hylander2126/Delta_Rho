clc; clear; close all;

figure(); ax = axes; hold(ax,'on'); box(ax,'on');
axis(ax,'equal');

objectColor = [247 230 254]/255;

po_0 = [2;3;pi/4]; % Initial position of the object
dpo_0 = [0;0;0];     % Initial velocity of the object

mo = 1;    % Payload mass in kg
Io = 1;     % Payload inertia in kgm^2
uk = 0.2;     % Coefficient of kinetic friction between the surface and object
Ro = 0.2;   % Body radius of the object

ObjectBodyPoints = [0.25 -0.25; 0.5 -0.25; 0.95 0.4; 0.25 0.8; -0.4 0.8;
    -0.95 0.4; -0.6 0; -0.95 -0.6; -0.5 -0.5; -0.4 -0.7; 0.15 -0.6];

O = payload('bodyPoints',Ro*ObjectBodyPoints,'mass',mo,'inertia',Io,'uk',uk,'bodyColor',objectColor,...
    'bodyRadius',Ro);

O.plot(ax);

O.move(po_0,dpo_0);






N = 11;

p = O.getBodyPoint(N);

hold on

plot(p(:,1),p(:,2),'or','MarkerFaceColor','r');

