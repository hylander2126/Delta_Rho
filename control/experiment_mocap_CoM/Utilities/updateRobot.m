function [Robot] = updateRobot(t,Robot,payload,s1)

Rz = @(q)[cosd(q),-sind(q),0;sind(q),cosd(q),0;0 0 1];
%% Jacobian Assignment
% J_rf = [-1.7322, 1, 1; 0, -2, 1; 1.7322, 1, 1]; % Jacobian mapping wheels to robot frame (between front wheels)
% J_temp = [-1.7321   1.0000    0.4226;     0    -2.0000    1.0000;   1.7321   1.0000    0.4226]; % Jacobian from calculation then modified
% J_r = 3*[-0.5774, 0.7314, 5.7504; 0, -0.2686, 5.7504; 0.5774, 0.7314, 5.7504]; % Jacobian mapping wheels to robot end effector

robotRot = rad2deg(Robot.x.data(3));
payloadRot = rad2deg(payload.x.data(3));
payloadRot_prev = rad2deg(payload.theta_prev);

%% Update Bounds Every Frame
Robot.negBound = Rz2d(payloadRot - payloadRot_prev) * Robot.negBound;
Robot.posBound = Rz2d(payloadRot - payloadRot_prev) * Robot.posBound;

%% Take measurement and check thresholds
measured_rotation = rad2deg(payload.x.data(3) - Robot.prev_meas); % DEGREES difference b/w prior measure & current rot.

% IF below 'measureable' threshold
if abs(measured_rotation) < Robot.max_theta
    % Update desired direction based on agent rotation
    new_direction = Rz2d(-robotRot) * Robot.desired_direc;

% IF GREATER than threshold
else 
    Robot.prev_meas = payload.x.data(3);

    if sign(measured_rotation) < 0
        Robot.posBound = Rz2d(measured_rotation) * Robot.desired_direc; % Update positive (c.clock-most bound)
    else
        Robot.negBound = Rz2d(measured_rotation) * Robot.desired_direc; % Update negative (clock-most bound)
    end
    % Get new direction - bisect bounds w/ unit vector, then multiply by desired acceleration
    pseudo_bisect = Robot.negBound + Robot.posBound;
    new_direction = pseudo_bisect./norm(pseudo_bisect);
    
    % Update world frame desired direction
    Robot.desired_direc = new_direction;

    % Reset time since last measurement
    Robot.measure_t = t;
    
%     fprintf('\nUPDATE BOUNDS, measurement:%d\n\n',round(measured_rotation))
end

% STOP CONDITION: Checking for min rotation threshold every N seconds of uninterrupted motion.
if (t - Robot.measure_t) > 3
    Robot.measure_t = t;
    
    if measured_rotation < Robot.min_theta %% THIS MIGHT BE A POOR STOP CONDITION - VERIFY
        fprintf("\n\nSTOP at time %d with measured theta: %d\n\n", round(t), round(measured_rotation))
        Robot.stop = 1;
        % Get final push vector, bisecting final bounds
        pseudo_bisect = Robot.negBound + Robot.posBound;
        Robot.desired_direc = pseudo_bisect./norm(pseudo_bisect);
        new_direction = [0;0];
    end
end

% Assign direction (u_r is always wrt ROBOT frame)
u_r(1:2,1) = Robot.v_desired .* new_direction;
u_r(2,1) = - u_r(2,1);                          %% MUST FLIP Y VALUE: ROBOT IS INITIALIZED 'UPSIDOWN' CURRENTLY (J_r)


%% Calculate attitude error term
% Get UNIT vectors of agent heading and payload heading
r = [Robot.J(3,2);-Robot.J(3,1)]; % MUST be local Jacobian NOT J_r
r = r/norm(r); % Unit vector of attachment pt to payload

% Calculate vectors representing payload & robot heading
noW = Rz(payloadRot)*[r;0]; % ~ vector: O to r, updated w/Rot.
nrW = Rz(robotRot)*[-1;0;0];

% Calculate error vector (atan2 gives negative values also)
e_r = -atan2d(noW(1)*nrW(2)-noW(2)*nrW(1), noW(1)*nrW(1)+noW(2)*nrW(2));

% Piecewise (adaptive control) for larger errors
if abs(e_r) > 5 % 10 for extreme case, 5 otherwise
    e_r = 1.5*(1.1^(e_r+26)); % Robot.K_wp*e_r + Robot.K_wd*sign(e_r);
end
u_r(3,1) = e_r;


%% Calculate motor torque values using ROBOT Jacobian
f = Robot.J_r*u_r;

% fPulse(s1,robot,f,100); % Send pulse to motors below 'stick' value


%% Assign robot motor torques | k=1 front left; k=2 rear; k=3 front right
for k=1:3
    if(f(k) >= 0)
        Robot.u.data(2*k-1,1) = 0;
        Robot.u.data(2*k,1) = uint8(abs(f(k)));
    else
        Robot.u.data(2*k-1,1) = uint8(abs(f(k)));
        Robot.u.data(2*k,1) = 0;
    end
end
end
