%% TEST file to visualize vectors after running experiment
clc;

Rz = @(q)[cosd(q),-sind(q);sind(q),cosd(q)];

figure();
set(gca,'XColor','none','YColor','none')
set(gcf,'Color','w')
hold on

%% Lines
v1 = [1; 0.026];


v2 = [-0.161; -0.987];



%% Draw Circle Payload
circ = linspace(0, 2*pi);
diam = 172;
circX = diam/2*cos(circ);
circY = diam/2*sin(circ);

patch(circX, circY, 'k')
axis equal

CoM = [];

for col=1:size(v1,2)
    %% Draw first Arrow
    quiver(-86, 0, diam*v1(1,col), diam*v1(2,col),'g')
    axis equal
    
    %% Draw second Arrow - SIGN FLIPPED SINCE IT IS 180 DEGREES ROTATED
    % quiver(86, 0, -diam*finalMeasurement_360(1), -diam*finalMeasurement_360(2))
    
    % ACTUALLY NOW trying from 90 degrees... no need to flip signs or rotate
    quiver(0, 86, diam*v2(1,col), diam*v2(2,col),'g')
    axis equal
    
    
    %% Plot and Compute Interesection
    CoM = getIntersection(diam, v1(:,col), v2(:,col));
    plot(CoM(1),CoM(2),'o','MarkerFaceColor','c','MarkerEdgeColor','w','MarkerSize',9)
    disp(CoM)
end


%% Plot ACTUAL CoM
% actualCoM = [0; 0];
% actualCoM = 1000*[.02268; -.01512];
% actualCoM = 1000*[.0189; .0189];
actualCoM = 1000*[-0.01512; 0.00756];
% actualCoM = 1000*[-0.01134; -0.01701];

plot(actualCoM(1),actualCoM(2),'pentagram','MarkerFaceColor','r','MarkerSize',18)

legend('','LoA Estimate','','CoM Estimate','Actual CoM')
