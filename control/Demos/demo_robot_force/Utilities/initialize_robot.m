% Take raw data and record average pot offsets
if five_bar.init && t < 0.5
    five_bar.offset_avg = [five_bar.offset_avg; data];
% Set the offset ONCE
elseif five_bar.init
    disp('Initialization complete!')
    % Should be ~[120,60] degrees
    five_bar.offset = mean([mean(five_bar.offset_avg(1)) - deg2rad(120), mean(five_bar.offset_avg(2)) - deg2rad(60)]);
    five_bar.init = 0;

% Average calibration data for 1 second
elseif five_bar.calib && t < 1.5
    five_bar = sensor_kinematics(five_bar);
    five_bar.home_avg = [five_bar.home_avg; five_bar.p(:,5)'];  % Add to average calibration array
    five_bar.phi_avg  = [five_bar.phi_avg; five_bar.alpha'];

% Set the home configuration and phi values ONCE
elseif five_bar.calib
    disp('Calibration complete!')
    five_bar.home = mean(five_bar.home_avg)';
    five_bar.phi = mean(five_bar.phi_avg)';
    five_bar.calib = 0;
    % Complete initialization period
    Robot.initialize = 0;
end