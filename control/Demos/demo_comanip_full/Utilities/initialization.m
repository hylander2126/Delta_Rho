function [force_sensor, iter1] = initialization(force_sensor, iter1)

%% INITIALIZE sensor - record resting offset of pots from 0
% Take raw data and record average pot offsets
if ~(force_sensor.calib)
    if iter1 < 50   % Get 50 initial measurements
        force_sensor.offset_avg = [force_sensor.offset_avg; force_sensor.alpha]; % data];
    else
        % Set the offset ONCE for rest of demo (subtracting expected values 120&60 degs)
        force_sensor.offset = mean([mean(force_sensor.offset_avg(1)) - deg2rad(120), mean(force_sensor.offset_avg(2)) - deg2rad(60)]);
        % Begin calibration period
        force_sensor.calib = 1;
        disp('Initialization complete!')
    end
    
    % Increment temporary counter
    iter1 = iter1 + 1;

% Average calibration data for some steps
else
    if iter1 < 100   % Get 50 calibration measurements
        force_sensor = force_sensor.kinematics();
        force_sensor.home = mean([force_sensor.home; force_sensor.EE]);  % Add to average calibration array
        force_sensor.phi  = mean([force_sensor.phi;  force_sensor.alpha']);
    
    % Set the home configuration and phi values ONCE
    else
        % Exit initialization period
        force_sensor.init = 0;
        force_sensor.calib = 0;
        disp('Calibration complete!')
    end

    % Increment temporary counter
    iter1 = iter1 + 1;
end