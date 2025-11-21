classdef ForceSensor
    properties
        % Static values
        ID              % Same as robot ID
        spring_k        % Spring constants
        links           % Link lengths
        deadzone        % Deadzone for minimum detection in grams % 3.5 nominal value
        home            % This is set in the first few steps % [11.42; 53.7105] nominal value 
        phi             % Base resting angles for joints 1&2
        offset          % Nominal offset for potentiometers
        % Dynamic values
        alpha           % --- BASE JOINT ANGLES ---
        p               % Joint coordinates (x,y)
        direction       % Force direction
        force           % Force magnitude
        EE              % End effector coordinates (x,y)
        % For averaging upon calibration
        init            % Initialization mode
        calib           % Calibration mode
        offset_avg
        prev_raw        % Previous RAW sensor reading (handles packet loss)
    end


    methods
        % Constructor
        function obj = ForceSensor(ID)
            % Initialize static values
            obj.ID          = ID;
            obj.spring_k    = -0.25*(25.4/1)*(4.448/1)/ pi; % (max torque / max deflection) converted from in-lb to N-mm
            obj.links       = [25; 25; 40; 40; 22.84];
            obj.deadzone    = 18; % 15; % 14.0;
            obj.home        = [0, 0];
            obj.phi         = [0, 0];
            obj.offset      = 0;
            % Dynamic values
            obj.alpha       = [];
            obj.p           = [0 0; obj.links(5) 0; 0 0; 0 0; 0 0];
            obj.direction   = [0; 0];
            obj.force       = 0;
            % Average values for calibration
            obj.init        = 1;
            obj.calib       = 0;
            obj.offset_avg  = [];
            obj.prev_raw    = [0; 0];
        end


        %% Read sensor, map volts to rads, apply offset, and return result
        function obj = readSensor(obj, s1, robot)
            try
                raw_data = cell2mat(SerialCommunication(s1, robot, 80))'; raw_data(end) = []; % remove last unused element
            catch
                disp("Couldn't receive data")
                raw_data = [0; 0];
            end
            obj.alpha = reshape(raw_data(1:2), [2,1]);

            % % Catch timeout error, otherwise remove last unused element
            % if isempty(raw_data)
            %     raw_data = obj.prev_raw;   % For now, just return 0 for sensor data. TODO: Switch to previous reading.
            % else
            %     raw_data(end) = [];
            % end
            % 
            % obj.prev_raw = raw_data;
            % 
            % % Map volts to rads AND apply offset: 5.76 rads ~ 330 degs (from datasheet)
            % cleaned_data = [mapfun(raw_data(1), 0, 1024, 0, 5.76), mapfun(raw_data(2), 0, 1024, 0, 5.76)] - obj.offset;
            % 
            % obj.alpha = reshape(cleaned_data, [2,1]); % WILL BE 'WRONG' UNTIL INIT IS FINISHED ~0.5s
        end


        %% Kinematics (calculate force based on pot data)
        function [obj] = kinematics(obj)
            %% Calculate coordinates of all other joints
            % Coordinates of joint 3 and 4
            for i=[1,2]
                jointCoords(i+2, :) = obj.p(i, :) + (obj.links(i)*[cos(obj.alpha(i)), sin(obj.alpha(i))]);
            end

            % Get vector between joints 3 and 4
            lambda = jointCoords(4,:) - jointCoords(3,:);

            % Calculate angle between lambda and l3 using law of cosines
            xi = acos((obj.links(3).^2 + norm(lambda).^2 - obj.links(4).^2)./(2*obj.links(3).*norm(lambda)));

            % Find EE (joint 5): multiply unit vector along lambda by l3, rotating by xi, then moving by p3
            jointCoords(5,:) = jointCoords(3,:) + (Rz2(xi) * (obj.links(3) * (lambda/norm(lambda))'))';
            obj.EE = jointCoords(5,:);
            
            %% Calculate EE displacement (Delta) and Force
            % Home config of EE calculated with resting alpha=(60,120) - run script with these values to generate    
            Delta = (jointCoords(5,:) - obj.home)';
            % ROTATING DELTA DUE TO WEIRD SKEW IN FORCE SENSOR
            % delta = Rz2(0.1974) * delta;

            % Calculate force (temporary, not robust) SCALE DELTA BY A FACTOR DEPENDING ON ITS ANGLE
            delta_angle = acos(dot(-Delta, [0,1]) / (norm(-Delta) * norm([0,1])));
            % Linear interpolation (from 0 degrees to 30 degrees)
            scaling_fn = @(x) interp1([0 deg2rad(30)], [2.7 3.2], x, 'linear','extrap');
            scaling_factor = scaling_fn(delta_angle);

            Force = scaling_factor*norm(Delta);

            % Return no force if within deadzone
            if abs(Force) < obj.deadzone
                Force = 0;
                Delta = [0; 0];
            end

            % Update class force, direction, other plotting
            obj.direction   = Delta;
            obj.force       = Force;
            obj.p           = [obj.p(1:2,:); jointCoords]; % ONLY USED FOR 'LIVE' PLOT
        end
    end
end
