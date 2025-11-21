function Gesture = gesture_client
    % Create a TCP/IP object 't' associated with the server
    % The IP address the IP of the Python server, 
    % and the port number should match, too
    
    t = tcpclient('localhost', 10000, 'Timeout',5);
    if ~exist("t","var")
        error("Have you started the tcpclient yet as 't'?")
    end
    % Continuously read data from the server
    while true
        while t.NumBytesAvailable > 0
        
            data = read(t); %, t.NumBytesAvailable);
            disp(char(data))
        
            % Convert bytes to string if necessary
            gesture = char(data);
            
            % Display the received gesture
            % disp(['Received gesture: ', gesture]);
            % disp(t.NumBytesAvailable)
        end
    
        pause(0.01)
        if toc > 10
            break
        end
    end
    % Close the connection when done
    clear t
end
