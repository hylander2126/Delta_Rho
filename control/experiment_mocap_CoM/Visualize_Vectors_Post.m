%% TEST file to visualize vectors after running experiment
clc; close all;
addpath("Utilities\")

Rz = @(q)[cosd(q),-sind(q);sind(q),cosd(q)];

figure
ax1 = axes;
set(ax1,'XColor','none','YColor','none')
axis square
% axis off
hold on

figure
ax2 = axes;
ax2.FontSize = 20;
xlabel(gca,'X (mm)')
ylabel(gca,'Y (mm)')
axis square
set(gcf,'Color','w')
hold on

colors = {'g','c', [0.1 1 0.75], [0.466 0.674 0.188], [0.4000 0.6980 1.0000] };

%% ***** DATA *****
v1 = [0.9968	0.9998	0.9992	0.9959	0.9988;
-0.0802	0.0183	-0.0393	0.0904	0.0492];


v2 = [-0.1498	-0.1827	-0.2117	-0.1226	-0.221;
-0.9887	-0.9832	-0.9773	-0.9925	-0.9753];

% actualCoM = [0; 0];
% actualCoM = 1000*[.02268; -.01512];
% actualCoM = 1000*[.0189; .0189]
actualCoM = 1000*[-0.01512; 0.00756]
% actualCoM = 1000*[-0.01134; -0.01701];


%% Draw Circle Payload
circ = linspace(0, 2*pi);
diam = 172;
circX = diam/2*cos(circ);
circY = diam/2*sin(circ);

patch(ax1, circX, circY, 'k');
patch(ax2, circX, circY, 'k');

%% Draw lines of action and intersections
for col=1:size(v1,2)

    % Select this trial's color
    color = colors{col};

    % Compute intersection
    CoM = getIntersection(diam, v1(:,col), v2(:,col))

    % Draw first Arrow (from 180 deg)
    quiver(ax1, -86, 0, diam*v1(1,col), diam*v1(2,col),'Color',color,'LineWidth',0.6,LineStyle='--')
    quiver(ax2, -86, 0, 86+CoM(1), CoM(2),'Color',color,AutoScale='off',ShowArrowHead='off')

    % Draw second arrow (from 90 deg) - No need to flip signs or rotate
    quiver(ax1, 0, 86, diam*v2(1,col), diam*v2(2,col),'Color',color,'LineWidth',0.6,LineStyle='--')
    quiver(ax2, 0, 86, CoM(1), -86+CoM(2),'Color',color,AutoScale='off',ShowArrowHead='off')
    
    % Draw Interesection
    plot(ax1, CoM(1),CoM(2),'o','MarkerFaceColor',color,'MarkerEdgeColor','w','MarkerSize',8)
    plot(ax2, CoM(1),CoM(2),'o','MarkerFaceColor',color,'MarkerEdgeColor','w','MarkerSize',8)
end


%% Plot ACTUAL CoM
plot(ax1, actualCoM(1),actualCoM(2),'pentagram','MarkerFaceColor','r','MarkerSize',19)
plot(ax2, actualCoM(1),actualCoM(2),'pentagram','MarkerFaceColor','r','MarkerSize',19)

%% Plot Centroid
plot(ax1, 0, 0, 'ok', 'MarkerFaceColor',[0.63 0.63 0.63])
plot(ax2, 0, 0, 'ok', 'MarkerFaceColor',[0.63 0.63 0.63])

% Create Legend
legend(ax1,'Payload','','','','','','','','','','','','','','LoA Estimate','CoM Estimate','Actual CoM')
