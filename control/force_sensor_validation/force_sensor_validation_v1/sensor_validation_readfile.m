addpath("Utilities\");
clc; clear; close all;

% Rot matrix fn
show_plot = 0;

%% Define problem constraints
% Spring constant calculation for the following spring: https://www.mcmaster.com/9271K665/
maxTorque = 0.402 * (25.4/1) * (4.448/1);   % in-lb to N-mm
maxDeflection = pi;                         % =180 degrees

k = maxTorque/maxDeflection;                % N-mm/rad

% Define link lengths
l1 = 25;
l2 = 40;
l3 = 40;
l4 = 25;
l5 = 22.84;

% First and Last point coordinates
A = [0; 0];
E = [l5; 0];
link_coords = [];


%% Read CSV
csv_data = readmatrix("validation_data_2023_08_09_manual_stage.csv");

% Offset ~270 - slightly different every run
offset = 280;

%% Load cell MAY BE negative, swap to positive
% csv_data(:,1) = csv_data(:,1); 

% Convert all sensor data from voltage to radians, add offset (0deg is 270 wrt x-axis), and wrap to 2Pi
csv_data(:,2) = wrapTo2Pi(mapfun(csv_data(:,2), 0, 1023, 0, deg2rad(330)) + deg2rad(offset));
csv_data(:,3) = wrapTo2Pi(mapfun(csv_data(:,3), 0, 1023, 0, deg2rad(330)) + deg2rad(offset));

%% Loop thru all data
f_app = zeros(size(csv_data,1),1);  % Initialize force readings

for i = 1:size(csv_data,1)
    theta1 = csv_data(i,2);         % Theta1 is 'left' pot s1
    theta2 = csv_data(i,3);         % Theta2 is 'right' pot s2
    
    %% Calculate joint coordinates
    % Coordinates of joint 2 and 4
    B = [l1*cos(theta1) ; l1*sin(theta1)];
    D = [l5 + l4*cos(theta2) ; l4*sin(theta2)];
    
    % Get distance between joints 2 and 4
    d = norm(D-B);
    
    % Calculate angle between d and l2
    alpha = acos((l2.^2 + d.^2 - l3.^2)./(2*l2.*d));
    
    % Get "V" vector with length l2 along d
    V = l2*(D - B)./d;
    
    % Calculate EE coords by rotating V vector by alpha
    C = Rz(alpha) * V + B;

    %% Force calculation
    % Calculate angle between l1 & l2 and l4 & l3
    gamma1 = acos(dot(C-B, -B) / (norm(C-B)*norm(B)));          % Angle between link 1 & 2
    gamma2 = acos(dot(C-D, -(D-E)) / (norm(C-D)*norm(D-E)));    % Angle between link 3 & 4
    % Calculate angle diff from home configuration
    dTheta1 = csv_data(1,2) - theta1;
    dTheta2 = csv_data(1,3) - theta2;

    f1 = (-k*dTheta1 / (l1*sin(gamma1)));
    f2 = (-k*-dTheta2 / (l4*sin(gamma2)));                      % dTheta2 rotates opposite, so negate it
    
    f_app(i) = (f1 + f2) * (1/9.807) * (1000/1);                % Sum and convert N to g-f

    link_coords = [link_coords; [B' C' D']];                    % Array of list coordinates for plotting
    sensor_direction(i,:) = [C(1)-link_coords(1,3), C(2)-link_coords(1,4)];
end

%% Plot graphic 'live' plot
if show_plot
    % Create figure window
    fig = figure;
    axis equal
    hold on
    xlim([-40, 60])
    ylim([-5, 60])
    Cx_init = link_coords(1,3);
    Cy_init = link_coords(1,4);

    for i=1:length(link_coords)
        if ~isgraphics(fig)         % Break loop when figure is closed
            break
        end
        B = link_coords(i,1:2);
        C = link_coords(i,3:4);
        D = link_coords(i,5:6);

        % Draw links
        h = plot([A(1) B(1) C(1) D(1) E(1)], ...
             [A(2) B(2) C(2) D(2) E(2)], '-ok');
        % Draw incoming force vector
        q = quiver(Cx_init, Cy_init, C(1)-Cx_init, C(2)-Cy_init, 1, 'm');
        title(sprintf('Force: %4.1f, Cell: %4.1f, Diff: %4.1f', f_app(i), csv_data(i,1), f_app(i)-csv_data(i,1)));

        drawnow
        pause(0.01)
        delete(h)
        delete(q)
    end
end

%% Calculate direction vector & error
sensor_direction = [sensor_direction, zeros(length(link_coords),1)];
true_direction = [0,-1, 0];                                             % SETUP ANGLE - TODO MAKE THIS A VARIABLE AT TOP
for i=1:length(sensor_direction)
    u = sensor_direction(i,:);
    CosTheta = max(min(dot(u,true_direction)/(norm(u)*norm(true_direction)),1),-1);
    if f_app(i) < 18                                                     % DEADZONE
        angle_error(i) = 0;
    else
        angle_error(i) = real(acosd(CosTheta));
    end
end


%% Add scaling factor to force sensor data
% scaled_f_app = f_app;
% sigmoid_transition = 1./ (1 + exp(-(f_app - 40) / 2));
% scaled_f_app = scaled_f_app + (f_app>40) .* (f_app * (1/1.6) - f_app);
% % scaled_f_app(f_app>40) = f_app(f_app>40) / 1.6;
% f_app = scaled_f_app;


%% Plot magnitude error over time
fig2 = figure;
colororder({'k','k'})
title('Force Sensor vs Load Cell Readings')
yyaxis left                                 % Plot relative to left axes
xlabel('Time')
ylabel('Force (g)','Color','k')
xlim([0,400])
ylim([min(f_app-csv_data(:,1))-5 , 180])    % TODO: Make this variable / automated
hold on

plot(csv_data(:,1),'k-','LineWidth',1);
plot(f_app,'b-','LineWidth',1)
plot(f_app-csv_data(:,1), 'r--')

% Plot angular error over time (same fig)
yyaxis right
ylabel('Angular Error (deg)','Color','k')
xlim([0,400])
ylim([min(f_app-csv_data(:,1))-5, 120])     % TODO: Make this variable / automated

plot(angle_error,'Color', [0.4940 0.1840 0.5560],'linestyle','--', 'LineWidth',1)
legend('Load Cell','Force Sensor','Error','Angular Error')


%% Display maximum difference
disp("Max difference between sensor and ground truth:")
disp(max(abs(f_app - csv_data(i,1))))
