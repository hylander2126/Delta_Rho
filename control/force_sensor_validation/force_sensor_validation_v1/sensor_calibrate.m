%% Read sensor signal in base/resting configuration for calibration

function [rest_x1, rest_x2] = sensor_calibrate()

    % For Laptop COM5 for PC COM3
    a = arduino("COM5","Uno","BaudRate",9600,"Libraries","I2C");
    
    sensorPin1 = "A0";
    sensorPin2 = "A1";
    interv = 50;
    init_time = 1;

    disp("Calibrating sensor")
    while (init_time < interv)
        rest_x1 = readVoltage(a,sensorPin1)*1024/5;
        rest_x2 = readVoltage(a,sensorPin2)*1024/5;
        init_time = init_time + 1;
    end
    
end

