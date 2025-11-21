%% Robot communication protocol
% Start = 0x55 = 85 (dec)
% 
% Communication format:
% 
% Start -> address -> (data transmit) or (data receive)
% 
% address = [W][code*][addr]
% 
% W: (1 bit)(MSB) = read/write request
%     W = 1 -> Write
%     W = 0 -> Read
%     if(write) -> Robot only receives information
%     if(read) -> Robot only sends back information
% 
% code*: (3 bits) = Call code
%         
%                                              W + Decimal Equivalent 
%         code* | Description                |   W = 1   |   W = 0
%         ------|----------------------------|-----------|------------
%         000   | Desired position (xd)      |    128    |     0
%         001   | Current position (x)       |    144    |    16
%         010   | Object position (xO)       |    160    |    32
%         011   | In (W=1) / Out (W=0)       |    176    |    48
%         100   | Actuator values(u)         |    192    |    64
%         101   | Reserved                   |           |
%         110   | Reserved                   |           |
%         111   | Sleep(W=1)/Reset(W=0)      |    240    |   112
%         ------|----------------------------|-----------|------------
%
% addr: (4 bits) = Robot ID
%        addr = 0 -> General call
%%
function data = SerialCommunication(s1,robot,code,field)

flushinput(s1);

% Sending "start" byte (start = 0x55 = 85
fwrite(s1,85, 'uint8');

if(code >= 128) % if no response required
    for n=1:length(robot)
        AddressCode = code;
        if(isnumeric(robot(n).ID))
            AddressCode = AddressCode + robot(n).ID;
        end

        % Write request
        fwrite(s1,uint8(AddressCode), 'uint8'); 

        temp = strcat(field,'_data');
        temp2 = strcat(field,'_converter');
        data = robot(n).(temp);
        data = robot(n).(temp2)(data);
        
%         temp_data = data

        for i=1:length(data)
            temp3 = strcat(field,'_type');
            fwrite(s1,data(i),robot(n).(temp3));
        end
    end
    
else    % if robot will send back information
    for n=1:length(robot)
        AddressCode = code;
        if(isnumeric(robot(n).ID))
            AddressCode = AddressCode + robot(n).ID;
        end
        fwrite(s1,uint8(AddressCode), 'uint8'); 
        if(code ~= 112)
            data{n} = fread(s1,3,'int16');
        end
    end
    
end