clc; clear; close all

%% Define problem constraints
% Spring constant calculation
% For the following spring: https://www.mcmaster.com/9271K665/
maxTorque = 0.25; % in-lb
maxDeflection = pi; % radians = 180 degrees

maxTorque = maxTorque * (25.4/1) * (4.448/1); % N-mm
k = -maxTorque/maxDeflection; % N-mm/rad

% Define link lengths
l1 = 25;
l2 = 40;
l3 = 40;
l4 = 25;
l5 = 22.84;

% First and Last point coordinates
Ax = 0;
Ay = 0;
Ex = l5;
Ey = 0;

% Spring positions - resting & max (desired)
theta2_rest = deg2rad(60);
theta1_rest = 2*theta2_rest;


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

%% Arduino initialization and pin definitions
comPort = "COM5";
baudRate = 9600;
sensorPin1 = "A0";
sensorPin2 = "A1";

a = arduino(comPort,"Uno","BaudRate",baudRate,"Libraries","I2C");

% %% TEMPORARY TEST
% while true
%     t1 = readVoltage(a,sensorPin1)*1024/5;
%     t2 = readVoltage(a,sensorPin2)*1024/5;
%     readings = round([t1, t2])
% end
% return


%% Calibrate sensor
disp("Calibrating sensor. Make sure the sensor is in its resting state.")
pause(2)
for i=1:50
    rest_x1 = readVoltage(a,sensorPin1)*1024/5;
    rest_x2 = readVoltage(a,sensorPin2)*1024/5;
end

%% Loop for desired duration of sensor testing
n = 300;
FappList = zeros(1,n); % Initialize force readings
% input("Enter")
for i = 1:n
    
    %% Get sensor values for Theta1 and Theta2    
    raw_sensor1 = readVoltage(a,sensorPin1)*1024/5;
    raw_sensor2 = readVoltage(a,sensorPin2)*1024/5;

    deltaTicks1 = abs(raw_sensor1 - rest_x1);
    deltaTicks2 = abs(raw_sensor2 - rest_x2);

    degrees1 = mapfun(deltaTicks1,0,1023,0,330);
    degrees2 = mapfun(deltaTicks2,0,1023,0,330);

    dTheta1 = degrees1*pi/180; % [x1 degrees1];
    dTheta2 = degrees2*pi/180;

    % Add dTheta from sensor to resting value to get "net" theta1 & theta2
    %% THIS IS WRONG, SWAP TO rest_x1, and rest_x2
    theta1 = theta1_rest + dTheta1;
    theta2 = theta2_rest - dTheta2;
    

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
    
    if i == 1
        Cx_init = Cx;
        Cy_init = Cy;
    end
    
    delete(h)
    delete(q)

%         j = i-1:i; % Range for path tracing
%         plot(Cx(j),Cy(j),'Color','r')
    h = plot([Ax Bx Cx Dx Ex], ...
             [Ay By Cy Dy Ey], '-ok');
    
    q = quiver(Cx_init, Cy_init, Cx-Cx_init, Cy-Cy_init, 1, 'm');
    
    % Define link coordinates for angle calculations
    A = [Ax Ay];
    B = [Bx By];
    C = [Cx Cy];
    D = [Dx Dy];
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

    Fapp1 = (-k*dTheta1 / (l1*sin(gamma1)));
    Fapp2 = (-k*dTheta2 / (l4*sin(gamma2)));
    
    Fapp = Fapp1 + Fapp2;
%     Fapp = (-k*dTheta1(i) / l1*sin(gamma1)) + (-k*-dTheta2(i) / l4*sin(gamma2));
%     Fapp = -k*angle2 / (cos(angleC) * sin(angleC) * l4);
        
    FappList(i) = Fapp;
    
%     pause(0.1);
end

    hold off    
    figure
    xlim("auto")
    ylim("auto")
    
    plot((1:n),FappList)
    xlabel("Time (s)")
    ylabel("Force Measurement (N)")

