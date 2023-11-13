clc; clear, close all
addpath 'classes and functions'/
% Visual settings - Generate environment/plot based on supplied limits
scene = generateEnvironment([-2.5 5 -2.5 5]);

%% Object (payload) information and initialization
po_0 = [-1.5;-2;-pi/5]; % Initial position of the object
po_x = linspace(0, 4)';
po_y = linspace(0, 4)';
po_q = linspace(0, 2*pi)';
po_0 = [-1; -1; 0];
po_1 = [-1; 0; -pi/6];
po_2 = [0; 1; -pi/3];
po_3 = [1; 1; -pi/1.5];
po_4 = [2; 0; -pi/0.75];

dpo_0 = [0;0;0];     % Initial velocity of the object

mo = 1;    % Payload mass in kg
Io = 1;     % Payload inertia in kgm^2
uk = 0.2;     % Coefficient of kinetic friction between the surface and object
Ro = 1;   % Body radius of the object

% ObjectBodyPoints = [0.25 -0.25; 0.5 -0.25; 0.95 0.4; 0.25 0.8; -0.4 0.8;
%     -0.95 0.4; -0.6 0; -0.95 -0.6; -0.5 -0.5; -0.4 -0.7; 0.15 -0.6];

ObjectBodyPoints = [0 0; 1 0; 1 1; 0 1];

O = payload('bodyPoints',Ro*ObjectBodyPoints,'mass',mo,'inertia',Io,'uk',uk,'bodyColor',scene.O,...
    'bodyRadius',Ro*2);

po = [po_x po_y po_q]';
for p = 1:size(po,2)
    O.move(po(:,p),dpo_0);
    % TEMP plot the object in initial position
    O.plot(scene.ax);
    hold on;
%     delete(h)
    pause(.05);
end
% O.move(po_0,dpo_0);

