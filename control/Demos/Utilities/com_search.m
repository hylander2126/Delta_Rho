% PD Controller - error is angle between push vector and force sensor feedback
vec_temp = Robot.d_prev; % Rz2(0.1974) * Robot.d_prev;
e       = getAngle(vec_temp, five_bar.direction, 'r');
deg_e = rad2deg(e)
e_dot   = (e - Robot.e_prev)/dt;

% Now calculate PD control input
u_r_new = Rz2(0.08*e + 0.01*e_dot) * Robot.d_prev; % 0.15, 0.05 gains worked well so did 0.05 for P

% Catch if no force applied for whatever reason
if abs(five_bar.force) < five_bar.deadzone
    u_r = [0;0;0];
else
    u_r = u_r_new;
    Robot.d_prev = u_r;
end

% Record current error for subsequent time step
Robot.e_prev = e;

f_ = getAngle(five_bar.direction, [1;0], 'r');
u_ = getAngle(u_r_new, [1;0], 'r');
% u_ = u_r_new;
e_ = e;


