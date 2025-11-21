addpath("Utilities\")

%% Plotting Controller Data

fig = figure('Units','normalized','Position',[0 0.1 0.7 0.7]);
hold on;
fontsize(fig, 18, "points")
xlabel('Time (s)'); ylabel('Angle (rad)');
ylim([-inf max(PD_data(:,4)+0.2)])


closed = 0;
totalSteps = size(PD_data,1);

markers = '* diamond o';
i = 1;
% plot(PD_data(:,1), PD_data(:,2), 'g--', 'LineWidth',3); % Force sensor
scatter(PD_data(:,1), PD_data(:,2), 'g', markers(i), 'LineWidth',3); % Force sensor
hold on
i = 2;
% plot(PD_data(:,1), PD_data(:,3), 'b', 'LineWidth',3); % CoM response
scatter(PD_data(:,1), PD_data(:,3), 'b', markers(i), 'LineWidth',3); % CoM response
i = 3;
% plot(PD_data(:,1), PD_data(:,4), 'r', 'LineWidth',3); % Attitude response
scatter(PD_data(:,1), PD_data(:,4), 'r', markers(i), 'LineWidth',3); % Attitude response

legend('Force Sensor','CoM Estimate', 'Yaw Correction'); %,'Location','best')


%% Plotting CoM line data
f1 = figure();
f1.Position = [600, 300, 800, 500];
hold on
axis equal;
% stop = uicontrol('style','toggle','string','stop');
axis([-100 100 -100 100]);
xlabel('x'); ylabel('y');


%% PLOTTING initialization
% Draw payload
circ = linspace(0, 2*pi);
circX = (172/2)*cos(circ); % 0.15 just works...
circY = (172/2)*sin(circ);
circle(1) = patch(circX, circY, [173 214 225]/255);

% Draw CoM
COM = scatter(0, 26.71, 300, 'pentagram r', 'filled');
% COM = scatter(0, -26.71, 300, 'pentagram r', 'filled');

% Draw centroid
centroid = scatter(0,0,100,'.k');

% Draw attachment point
att_pt = scatter(-172/2,0,300,'*g');

% Draw our estimate
temp = [Robot.d_prev(1); Robot.d_prev(2)];
quiver(-172/2,0, temp(1), temp(2), 'g', 'LineWidth', 3)

legend('Payload', 'CoM', 'Centroid', 'Attach Pt.', 'CoM Estimate')