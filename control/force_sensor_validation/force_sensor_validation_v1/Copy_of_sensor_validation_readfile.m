addpath("Utilities\");
clc; clear; close all;

% Rot matrix fn
Rz = @(th) [cos(th), -sin(th); sin(th), cos(th)];

%% Define problem constraints
% Spring constant calculation
% For the following spring: https://www.mcmaster.com/9271K665/
maxTorque = 0.402; % in-lb
maxDeflection = pi; % 180 degrees

maxTorque = maxTorque * (25.4/1) * (4.448/1); % Convert in-lb to N-mm
k = maxTorque/maxDeflection; % N-mm/rad

% Define link lengths
l1 = 25;
l2 = 40;
l3 = 40;
l4 = 25;
l5 = 22.84;

% First and Last point coordinates
A = [0; 0];
E = [l5; 0];

% Spring positions - resting & max (desired)
% theta2_rest = deg2rad(60);
% theta1_rest = 2*theta2_rest;

%% Read CSV
csv_data = readmatrix("validation_data_2023_07_27_manual_stage.csv");

% Offset - slightly different every run
offset = 280;

% Convert all sensor data from voltage to radians, add offset (0deg is 270 wrt x-axis), and wrap to 2Pi
csv_data(:,2) = wrapTo2Pi(mapfun(csv_data(:,2), 0, 1023, 0, deg2rad(330)) + deg2rad(offset));
csv_data(:,3) = wrapTo2Pi(mapfun(csv_data(:,3), 0, 1023, 0, deg2rad(330)) + deg2rad(offset));
load_cell_rest = csv_data(1,1);
s1_rest = csv_data(1,2); % TODO ON SETUP, MAKE SURE S1 IS 'LEFT' POT
s2_rest = csv_data(1,3);
csv_data(1,:) = []; % Remove initial conditions

rad2deg(s1_rest)
rad2deg(s2_rest)
% return

%% Create figure window
figure
axis equal
hold on
xlim([-40, 60])
ylim([-5, 60])
% initialize plot object to delete at the proper time
h = [];
q = [];

%% Loop thru all data
FappList = zeros(size(csv_data,1),1); % Initialize force readings

for i = 1:size(csv_data,1)
    theta1 = csv_data(i,2); % Theta1 is 'left' pot s1
    theta2 = csv_data(i,3); % Theta2 is 'right' pot s2
    dTheta1 = s1_rest - theta1;
    dTheta2 = s2_rest - theta2;
    
    %% Calculate coordinates of all other joints
    % Coordinates of joint 2 and 4
    [Bx, By] = deal(l1*cos(theta1) , l1*sin(theta1));
    B = [l1*cos(theta1) ; l1*sin(theta1)];
    [Dx, Dy] = deal(l5 + l4*cos(theta2) , l4*sin(theta2));
    D = [l5 + l4*cos(theta2) ; l4*sin(theta2)];
    
    % Get distance between joints 2 and 4
%     d = norm([Dx;Dy] - [Bx;By]);
    d = norm(D-B);
    
    % Calculate angle between d and l2
    alpha = acos((l2.^2 + d.^2 - l3.^2)./(2*l2.*d));
    
    % Get "V" vector with length l2 along d
    v = l2*(D - B)./d;
    
    % Calculate end effector x,y by rotating V vector by alpha (rot. matrix)
    C = Rz(alpha) * v + B;
    
    if i == 1
        Cx_init = C(1);
        Cy_init = C(2);
    end
    
    delete(h)
    delete(q)
    
    % Draw links
    h = plot([A(1) B(1) C(1) D(1) E(1)], ...
             [A(2) B(2) C(2) D(2) E(2)], '-ok');
    % Draw incoming force vector
    q = quiver(Cx_init, Cy_init, C(1)-Cx_init, C(2)-Cy_init, 1, 'm');
    
    if i == 2
        input('enter')
    end
    % Shift coordinates wrt different frames for angle calculation
    C1 = C - B;
    C2 = C - D;
%     Ftemp1 = F - C1 - B;
%     Ftemp2 = F - C2 - D;
        
    gamma1 = acos(dot(C1,-B)/(norm(C1)*norm(B))); % Angle between link 1 & 2
    gamma2 = acos(dot(C2,-(D-E))/(norm(C2)*norm(D-E))); % Angle between link 3 & 4
%     gamma3 = acosd(dot(Ftemp1,C1)/(norm(C1)*norm(Ftemp1))); % Angle between link 2 & Fapp
%     gamma4 = acosd(dot(Ftemp2,C2)/(norm(C2)*norm(Ftemp2))); % Angle between link 3 & Fapp

    Fapp1 = (-k*dTheta1 / (l1*sin(gamma1)));
    Fapp2 = (-k*dTheta2 / (l4*sin(gamma2)));
    
    FappList(i) = Fapp1 + Fapp2;
    newTitle = sprintf('Force: %.4f', FappList(i)*101.97); % Convert N to g-f
    title(newTitle);
    
    drawnow
    pause(0.01);
end

