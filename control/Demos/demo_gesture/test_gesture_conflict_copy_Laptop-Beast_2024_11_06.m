 % Move robot based on gesture
% Closed fist stops robot, pointing controls direction
clc; clear all; close all;
addpath("Utilities/");

global robot
global J_r
global t

% Start the robot communication serialport
if exist('s1','var')
    clear global s1
else
    global s1
    % s1 = serialport('/dev/ttyUSB0',9600,'DataBits',8,'StopBits',1);
    s1 = serialport('COM4',9600,'DataBits',8,'StopBits',1);
end

% Create 'Robot' object
robot = Robot(5);
J_r = [0.8192    0.5736   -0.1621;
       0   -1.0000   -1.3600;
       -0.8192    0.5736   -0.1621];

% Start the gesture recognition client
start_gesture_client(@printData);

% Data Handler function for timer callback in client
function printData(G)
    global robot
    global s1

    disp('')
    disp('Gesture:')
    % disp(str2num(G))
    disp(G)
    disp('')

    if strcmp(G,'Closed_Fist') 
        u_r = [0; 0; 0];
    elseif isempty(str2num(G))
        u_r = [0; 0; 0];
    else
        G = str2num(G);
        u_r = 3000*[G' ; 0];
        % Remember x and y are flipped
        temp = u_r(1,:);
        u_r(1,:) = u_r(2,:);
        u_r(2,:) = temp;
    end

    % u_r'
    
    % robot.u.data = J_r*u_r;
    robot = robot.setTorques(u_r);
    SerialCommunication(s1, robot, 192, 'u');
end