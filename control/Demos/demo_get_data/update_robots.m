function [robots, force_sensors] = update_robots(s1, N, robots, force_sensors)

% RECALL AGENT AND SENSOR FRAMES:
%                           
%                      ^ x
%                      |
%         _      O     *-->y O  <- Marker 3 (2)
%         |       \         / 
%         |        \       /
%      2d |         \     /
%         |          \   /
%         |           \ /
%         _            O        <- Marker 1 (1)
%                |<--------->|
%                      d
%
%                      O p5
%                     / \
%                    /   \
%                l3 /     \ l4
%                  /       \
%                 /    ^ y  \
%                 \    |    /
%                  O   *---O--> x

    % global PD_data;
    global t DIRECTION data_sensor
    u_r = [0;0;0]; % No robot motion unless updated explicitly

    
    %% Cancel force sensor reading
    for i=1:N
        % Read force sensor
        force_sensors(i) = force_sensors(i).readSensor(s1, robots(i));
    
        % Data returned by robot, defined in firmware
        temp = force_sensors(i).alpha;
        temp2 = temp/100
        % norm(temp2)

        data_sensor = [data_sensor; temp(1)/100, temp(2)/100];
        DIRECTION = temp(1:2)/(norm(temp(1:2)));
    end
    
    %% Set new robot motor values
    for i=1:N
        robots(i) = robots(i).setTorques(u_r);
    end
end
