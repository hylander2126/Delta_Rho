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
            obj.spring_k    = 0.25*(25.4/1)*(4.448/1)/ pi; % (max torque / max deflection) converted from in-lb to N-mm
            obj.links       = [25; 25; 22.84];
            obj.deadzone    = 0.1; % 15; % 14.0;
            obj.home        = [0, 0];
            obj.phi         = deg2rad([120; 60]);
            obj.offset      = 0;
            % Dynamic values
            obj.alpha       = [];
            obj.direction   = [0; 0];
            obj.force       = 0;
            obj.p           = [];
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
%                 disp("Couldn't receive data")
                raw_data = [0; 0];
            end
            obj.alpha = reshape(raw_data(1:2), [2,1]);
        end


        %% Kinematics (calculate force based on pot data)
        function obj = kinematics(obj)
            dTheta = obj.phi - obj.alpha;

            P = [cos(obj.alpha(1))*obj.links(1) cos(obj.alpha(2))*obj.links(2); ...
                 sin(obj.alpha(1))*obj.links(1) sin(obj.alpha(2))*obj.links(2)];

            f1 = obj.spring_k * dTheta(1) / obj.links(1);
            f2 = obj.spring_k * dTheta(2) / obj.links(2);

            Force = f1+f2;

            % Get unit direction of F which is perpendicular to link. 
            % Alpha measured from resting link position
            f1_hat = Rz2(dTheta(1) + pi/2) * [obj.links(1); 0];
            f2_hat = Rz2(dTheta(2) - pi/2) * [obj.links(2); 0];
            
            Delta = f1_hat + f2_hat;

            % Return no force if within deadzone
            if abs(Force) < obj.deadzone
                Force = 0;
                Delta = [0; 0];
            end

            % Update class force, direction, other plotting
            obj.p           = P;
            obj.direction   = Delta;
            obj.force       = Force;
        end
    end
end
