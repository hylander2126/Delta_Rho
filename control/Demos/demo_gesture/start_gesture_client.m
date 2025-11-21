function start_gesture_client(dataHandler)
% Create a TCP/IP object 't' associated with the server
% Must pass a dataHandler function to process data in master program
    
    % Ensure persistent variable for timer to prevent out of scope
    persistent timerObj
    global t
    global s1
    % 
    % % Create serial port connection
    % s1 = serialport('/dev/ttyUSB0',9600,'DataBits',8,'StopBits',1);
    
    % Check if tcp client exists
    if ~isempty(t) %|| isvalid(t)
        disp('TCP client already running, retrying connection...')
        clear t;
    end

    %% Attempt to create TCP client with retries
    address = 'localhost';
    port = 10000;
    maxAttempts = 5;
    attempt = 0;
    while attempt < maxAttempts
        try
            t = tcpclient(address, port);
            disp('Connection Successful!')
            break; % Break loop if connection successful
        catch ME
            disp(['attempt ' num2str(attempt+1) ' failed: ' ME.message])
            pause(2); % Wait 1 second before retrying
        end
        attempt = attempt+1;
    end

    if attempt == maxAttempts
        disp('Failed to connect after maxnum attempts.')
        return;
    end
    
    %% Initialize and start the timer
    attempt = 0;
    while attempt < maxAttempts
        try
        % if isempty(timerObj) % || ~isvalid(timerObj)
            timerObj = timer('TimerFcn',@timerCallback, 'Period',0.01, ...
                'ExecutionMode', 'fixedRate');
            start(timerObj);
            fprintf('TCP client started at %s:%i \n\n', address, port);
            break
        catch ME
            % else
            % disp('TCP client is already running, timerObj not empty...')
            disp(['Error: ', ME.message])
        end
        attempt = attempt + 1;
    end


    %% Define timer callback function
    function timerCallback(~,~)
        if t.NumBytesAvailable > 0
            % Read from data port and convert to string
            data = char(read(t));

            % Stop condition if 'STOP' bit is read
            if contains(data, 'STOP')
                stop(timerObj)
                delete(timerObj)
                timerObj = [];
                clear t
                clear global s1
                disp('TCP Client Stopped.')
                return;
            end

            % Call the external handler
            dataHandler(data);
        end
    end
end