%% Read analog signal from sensor angles

% function [x1, x2] = sensor_read()

    % baudRate = 9600;
    % s1 = serialport("COM5",baudRate,"DataBits",8,"StopBits",1);
    % fopen(s1);
    % 
    % reading = fscanf(s1)
    clc; clear; close all;
    a = arduino("COM5","Uno","BaudRate",9600,"Libraries","I2C");
    
    sensorPin1 = "A0";
    sensorPin2 = "A1";
    
    interv = 1000;
    init_time = 1;
    x1 = 0;
    x2 = 0;
    
    disp("Calibrating sensor")
    while (init_time < 50)
        restSensor1 = readVoltage(a,sensorPin1)*1024/5
        restSensor2 = readVoltage(a,sensorPin2)*1024/5
        init_time = init_time + 1;
    end
    
    while (init_time < interv)
    sensor1 = readVoltage(a,sensorPin1)*1024/5
    sensor2 = readVoltage(a,sensorPin2)*1024/5

    deltaTicks1 = (sensor1 - restSensor1);
    deltaTicks2 = (restSensor2 - sensor2);

    degrees1 = mapfun(deltaTicks1,0,1023,0,330);
    degrees2 = mapfun(deltaTicks2,0,1023,0,330);

    x1 = [x1 degrees1];
    x2 = [x2 degrees2];

    plot(x1)
    hold on
    plot(x2)
    grid ON
    init_time = init_time + 1;
    hold off
    legend("Theta1", "Theta2")
    drawnow

    end
% end

