addpath("Utilities\")
addpath("Results_09-14-23\")

%% Circle first
pos1(1) = load('circ_pos1_trial1_PD.mat');
pos1(2) = load('circ_pos1_trial2_PD.mat');
pos1(3) = load('circ_pos1_trial3_PD.mat');

pos2(1) = load('circ_pos2_trial1_PD.mat');
pos2(2) = load('circ_pos2_trial2_PD.mat');
pos2(3) = load('circ_pos2_trial3_PD.mat');

%% Polygon next
pos1(1) = load('blob_pos1_trial1_PD.mat');
pos1(2) = load('blob_pos1_trial2_PD.mat');
pos1(3) = load('blob_pos1_trial3_PD.mat');

pos2(1) = load('blob_pos2_trial1_PD.mat');
pos2(2) = load('blob_pos2_trial2_PD.mat');
pos2(3) = load('blob_pos2_trial3_PD.mat');

%% Load each final estimate (each col of v is an estimate)
for n=1:length(pos1)
    % Rotate estimates by average payload ending orientation
    v1(:,n) = Rz2(deg2rad(0)) * pos1(n).final_estimate;
    v2(:,n) = Rz2(deg2rad(0)) * pos2(n).final_estimate;
    % v1(:,n) = Rz2(deg2rad(-90)) * pos1(n).final_estimate;
    % v2(:,n) = Rz2(deg2rad(51)) * pos2(n).final_estimate;
end


% Plotting CoM line data
fig1 = figure('Units', 'normalized', 'Position', [0 0.1 0.8 0.8]);
hold on; axis equal;
axis([-100 100 -100 100]);          % For circle
% axis([-60 60 -70 135]);           % For polygon
xlabel('x (mm)'); ylabel('y (mm)');


% Draw payload
circ_poly = linspace(0, 2*pi);
x_poly = (172/2)*cos(circ_poly); % 0.15 just works...
y_poly = (172/2)*sin(circ_poly);
% x_poly = -7.6*[  0     1     2   2.6  2.2   3   4    5   5.75  5.9  6.1 5.9  5.7   5   4  3.75  3.5  3.75  4    4.2  4   3   2    1   0 ...
%     -1  -2  -3  -4 -5 -5.2 -5.4 -5.2  -5  -4 -3.5 -3.15 -3.25 -3.7   -4     -5    -6  -6.6   -6.9    -6.6    -6    ...
%     -5    -4   -3   -2     -1];
% y_poly = -7.6*[-14.9 -14.5 -13.3 -10  -7   -3  -2  -1.2  -0.3 0.2 0.9  1.1  1.2  1.8  3  3.3    4   4.8   5.2  6.2  7  7.5 7.7 7.65 ...
%     7.45 7  6.3 5.3  4  2  1.7  0.9  0.2   0  -1  -2   -4     -5    -6   -6.6  -7.5 -8.65 -9.75 -10.8  -11.6  -12.4 ...
%     -13.3 -14 -14.5 -14.8 -15.1];

patch(x_poly, y_poly, [0.6784 0.8392 0.8824]);

% Draw centroid
scatter(0,0,100,'.k');

% Draw attachment point
scatter(-172/2,0,300,'og','Filled');
% scatter(-46.36, -6.84,300,'og','Filled');

% 'Fake' markers for legend - won't actually be plotted
h(1) = plot(nan,nan, 'pentagram b', 'MarkerSize', 20, 'MarkerFaceColor','b', 'DisplayName', 'CoM 1');
h(2) = plot(nan,nan, 'pentagram m', 'MarkerSize', 20, 'MarkerFaceColor','m', 'DisplayName', 'CoM 2');
h(3) = plot(nan,nan, 'og', 'MarkerSize', 16, 'MarkerFaceColor','g', 'DisplayName', 'Attach Pt.');



% Recall that Y axis is flipped on robot but only on v2 for circle and v1 for polygon
for n=1:3
    quiver(-172/2, 0, v1(1,n), v1(2,n), 'm', 'LineWidth', 3); % Refers to com1
    quiver(-172/2, 0, v2(1,n), -v2(2,n), 'b', 'LineWidth', 3); % Refers to com2
    % quiver(-46.36, -6.84, v1(1,n), -v1(2,n), 0.4, 'm', 'LineWidth', 3); % Refers to com1
    % quiver(-46.36, -6.84, v2(1,n), v2(2,n), 0.4, 'b', 'LineWidth', 3); % Refers to com2
end

% Draw CoMs
act_com = [0 -16.32; 0 26.71]; % Circle
com1 = scatter(0, -16.32, 300, 'pentagram b','Filled'); % Circle
com2 = scatter(0, 26.71, 300, 'pentagram m', 'Filled'); % Circle

% act_com = [-1.14 -3.43; 1.14 22.86];
% com1 = scatter(-1.14, -3.43, 300, 'pentagram b','Filled'); % -2.71, -3.61, 300, 'pentagram b','Filled');
% com2 = scatter(1.14, 22.86, 300, 'pentagram m', 'Filled'); % 0.9, 18.05, 300, 'pentagram m', 'Filled');

com1.MarkerEdgeColor = 'k';
com2.MarkerEdgeColor = 'k';

legend(h);
fontsize(gcf, 24, "points")
% fontsize(gcf, 17, "points")

hold off


% Now to get the avg estimate and the actual CoM
v2(2,:) = -v2(2,:); % For circle
% v1(2,:) = -v1(2,:); % For polygon
temp1 = mean(v2,2);
temp1 = temp1/norm(temp1) % Remember to flip y sign
temp2 = mean(v1,2);
temp2 = temp2/norm(temp2)

actual1 = act_com(1,:) - [-172/2, 0]; % Circle
% actual1 = act_com(1,:) - [-46.36, -6.84]; % Polygon
% actual1(2) = -actual1(2);
actual1 = actual1/norm(actual1) % Remember to flip y sign

actual2 = act_com(2,:) - [-172/2, 0]; % Circle
% actual2 = act_com(2,:) - [-46.36, -6.84]; % Polygon
actual2 = actual2/norm(actual2)

getAngle(temp1, actual1, 'd')
getAngle(temp2, actual2, 'd')

%% Plotting Controller Data
fig2 = figure('Units','normalized','Position',[0 0.1 0.55 0.55]);
hold on;
xlabel('Time (s)'); ylabel('Angle (rad)');
xlim([2.5 5.3])
ylim([-0.45 -0.05])

colors = 'gbr';
markers = ['o','*','x'];
for n=1:3
    plot(pos1(n).PD_data(:,1), pos1(n).PD_data(:,2), strcat('-', markers(n)), 'Color', [0 0.5 0], 'MarkerSize', 10); % Force sensor
    plot(pos1(n).PD_data(:,1), pos1(n).PD_data(:,3), strcat('-', markers(n)),  'Color', 'b', 'MarkerSize', 10); % CoM response
%     plot(pos1(n).PD_data(:,1), pos1(n).PD_data(:,4), strcat('-', markers(n)), 'Color', 'r', 'MarkerSize', 10); % Attitude response
end
legend('Force Sensor','CoM Estimate'); %,'Location','best')
fontsize(gcf, 24, "points")
hold off

% And for com2
fig3 = figure('Units','normalized','Position',[0 0.1 0.55 0.55]);
hold on;
xlabel('Time (s)'); ylabel('Angle (rad)');
xlim([2.5 4.75])
ylim([-0.3 -0.02])

for n=1:3
    plot(pos2(n).PD_data(:,1), pos2(n).PD_data(:,2), strcat('-', markers(n)), 'Color', [0 0.5 0], 'MarkerSize', 10); % Force sensor
    plot(pos2(n).PD_data(:,1), pos2(n).PD_data(:,3), strcat('-', markers(n)),  'Color', 'b', 'MarkerSize', 10); % CoM response
%     plot(pos2(n).PD_data(:,1), pos2(n).PD_data(:,4), strcat('-', markers(n)), 'Color', 'r', 'MarkerSize', 10); % Attitude response
end

legend('Force Sensor','CoM Estimate'); %,'Location','best')
fontsize(gcf, 30, "points")
