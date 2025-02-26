function switchMode(ID, s1)
    arguments
        % By default use robot 3
        ID (1,1) double = 3;
        s1 (1,1)  internal.Serialport = serialport('COM4',9600,'DataBits',8,'StopBits',1);
    end

    % try
    %     s1 = serialport('COM4',9600,'DataBits',8,'StopBits',1); % /dev/ttyUSB0
    % catch
    %     disp('s1 already exists')
    % end
    
    % Robot constructor for SerialCommunication
    robot.ID            = ID;
    robot.u.data        = zeros(6,1);
    robot.u.type        = 'uint8';
    robot.u.convertor   = @(x) uint8(x);
    
    % Stop robot motion
    SerialCommunication(s1, robot, 192, 'u');
    % Send 'switch mode' command
    SerialCommunication(s1, robot, 208, 'u');
    % Delete serialport
    clear
end