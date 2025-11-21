% Simulation of double-crank mechanism moving
clc; clear; close all

%% Define problem constraints
% Spring constant calculation
% For the following spring: https://www.mcmaster.com/9271K665/
maxTorque = 0.25; % in-lb
maxDeflection = pi; % radians = 180 degrees

maxTorque = maxTorque * (25.4/1) * (4.448/1); % N-mm
k = -maxTorque/maxDeflection; % N-mm/rad

% Define link lengths
l1 = 20;
l2 = 35;
l3 = 35;
l4 = 20;
l5 = 24;

% First and Last point coordinates
Ax = 0;
Ay = 0;
Ex = l5;
Ey = 0;

% Spring positions - resting & max (desired)
theta1_rest = 4*pi/6;
theta2_rest = 2*pi/6;

% theta1_max = pi; % 7*pi/6;
% theta2_max = -pi/4; % - (theta1_max - pi);

% Create figure window
figure
axis equal
hold on
xlim([-l1 Ex+l4])
ylim([Ay l1+l2])

h = []; % initialize plot object to delete at the proper time
q = [];

for iter = 1:3

    %% Loading joint configurations
    
    % Configurations for base angles
    %% REPLACE WITH ACTUAL SENSOR VALUES
%     n = 150; % Number of configurations to try
%     theta1 = linspace(theta1_rest, theta1_max, n);
%     theta2 = linspace(theta2_rest, theta2_max, n);

    n = 1;
    theta1 = input("Enter theta1 angle (in degrees)")*pi/180;
    theta2 = input("Enter theta2 angle (in degrees)")*pi/180;
    
    delete(h)
    delete(q)

    % Convert raw angle to a Delta-angle change wrt resting positions
    dTheta1 = theta1 - theta1_rest;
    dTheta2 = theta2_rest - theta2;
    
    %% Get sensor values for Theta1 and Theta 2
    % % Initialize configurations w/ rest as first configuration
    % x1 = [3*pi/4 x1];
    % x2 = [pi/4 x2];
    % 
    % n = length(x1);
    % theta1 = theta1_0 + x1*pi/180;
    % theta2 = theta2_0 - x2*pi/180;
    
    %% Calculate coordinates of all other joints
    % Coords of joints 2 and 4
    Bx = l1*cos(theta1);
    By = l1*sin(theta1);
    Dx = l5 + l4*cos(theta2);
    Dy = l4*sin(theta2);
    
    % Get distance between joints 2 and 4
    d = sqrt((Dx - Bx).^2 + (Dy - By).^2);
    
    % if any(abs(l3-l4) > d | d > l3+l4)
    %     msgbox('Simulation is impossible. Check dimensions');
    %     return
    % end
    
    % Calculate angle between d and l2
    alpha = acos((l2.^2 + d.^2 - l3.^2)./(2*l2.*d));
    
    % Get "V" vector with length l2 along d
    vx = l2*(Dx - Bx)./d;
    vy = l2*(Dy - By)./d;
    
    % Calculate end effector x,y by rotating V vector by alpha (rot. matrix)
    Cx = (vx.*cos(alpha) - vy.*sin(alpha) + Bx)';
    Cy = (vx.*sin(alpha) + vy.*cos(alpha) + By)';
    
    if iter == 1
        Cx_init = Cx;
        Cy_init = Cy;
    end
    
    %% Create array of force vectors TEMP since we want this to coincide with the Delta-X of the end effector
    % Fx = Cx;
    % Fy = Cy + 4;
    
    FappList = zeros(1,n); % Initialize force readings
    
    for i = 1:n
%         j = i-1:i; % Range for path tracing
%         plot(Cx(j),Cy(j),'Color','r')
	    h = plot([Ax Bx(i) Cx(i) Dx(i) Ex], ...
                 [Ay By(i) Cy(i) Dy(i) Ey], '-ok');
    
        q = quiver(Cx_init, Cy_init, Cx(i)-Cx_init, Cy(i)-Cy_init, 1, 'm');
    
        % Define link coordinates for angle calculations
        A = [Ax Ay];
        B = [Bx(i) By(i)];
        C = [Cx(i) Cy(i)];
        D = [Dx(i) Dy(i)];
        E = [Ex Ey];
%         F = [Fx(i) Fy(i)];
    
        % Shift coordinates wrt different frames for angle calculation
        C1 = C - B;
        C2 = C - D;
    %     Ftemp1 = F - C1 - B;
    %     Ftemp2 = F - C2 - D;
        
        gamma1 = acos(dot(C1,-B)/(norm(C1)*norm(B))); % Angle between link 1 & 2
        gamma2 = acos(dot(C2,-(D-E))/(norm(C2)*norm(D-E))); % Angle between link 3 & 4
    %     gamma3 = acosd(dot(Ftemp1,C1)/(norm(C1)*norm(Ftemp1))); % Angle between link 2 & Fapp
    %     gamma4 = acosd(dot(Ftemp2,C2)/(norm(C2)*norm(Ftemp2))); % Angle between link 3 & Fapp

        Fapp1 = (-k*dTheta1(i) / (l1*sin(gamma1)));
        Fapp2 = (-k*dTheta2(i) / (l4*sin(gamma2)));
    
        Fapp = Fapp1 + Fapp2
    %     Fapp = (-k*dTheta1(i) / l1*sin(gamma1)) + (-k*-dTheta2(i) / l4*sin(gamma2));
    %     Fapp = -k*angle2 / (cos(angleC) * sin(angleC) * l4);
        
        FappList(i) = Fapp;
    
	    pause(0.1);
%         if i<100
%             delete(h)
%             delete(q)
%         end
    end
end
    hold off
    
    figure
    xlim("auto")
    ylim("auto")
    
    plot(dTheta1,FappList)
    xlabel("Delta Theta1 (rad)")
    ylabel("Force Measurement (N)")

