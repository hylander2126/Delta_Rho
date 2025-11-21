%% Read analog signal from sensor angles

function [x1, x2] = sensor_read(rest_x1, rest_x2)

    % For Laptop COM5 for PC COM3
    a = arduino("COM5","Uno","BaudRate",9600,"Libraries","I2C");
    
    sensorPin1 = "A0";
    sensorPin2 = "A1";
    interv = 1000;
    init_time = 1;
    x1 = 0;
    x2 = 0;
    
%     while (init_time < interv)
    raw_sensor1 = readVoltage(a,sensorPin1)*1024/5;
    raw_sensor2 = readVoltage(a,sensorPin2)*1024/5;

    deltaTicks1 = abs(raw_sensor1 - rest_x1);
    deltaTicks2 = abs(raw_sensor2 - rest_x2);

    degrees1 = mapfun(deltaTicks1,0,1023,0,330);
    degrees2 = mapfun(deltaTicks2,0,1023,0,330);

    x1 = degrees1; % [x1 degrees1];
    x2 = degrees2; % [x2 degrees2];

%     plot(x1)
%     hold on
%     plot(x2)
%     grid ON
%     init_time = init_time + 1;
%     hold off
%     legend("Theta1", "Theta2")
%     drawnow

%     end
end

