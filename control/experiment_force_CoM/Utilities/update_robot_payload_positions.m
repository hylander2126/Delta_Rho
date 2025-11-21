%% RUN AFTER READING OPTITRACK FRAME

theta_p = round(Opti_theta(1), 3);
theta_r = round(Opti_theta(2), 3);

% Record TRUE 'previous' position & rotation from prior time step
payload.theta_prev = payload.x.data(3);

% Update 'current' position & rotation
payload.x.data = [x(1); y(1); theta_p]; % *57.2958 for degrees
Robot.x.data   = [x(2); y(2); theta_r];