function [robot] = updateRobot(robot,object,s1)

%% Jacobian Assignment
J_rf = [-1.7322, 1, 1; 0, -2, 1; 1.7322, 1, 1]; % Jacobian mapping wheels to robot frame (between front wheels)
J_temp = [-1.7321   1.0000    0.4226;     0    -2.0000    1.0000;   1.7321   1.0000    0.4226]; % Jacobian from calculation then modified
J_r = 3*[-0.5774, 0.7314, 5.7504; 0, -0.2686, 5.7504; 0.5774, 0.7314, 5.7504]; % Jacobian mapping wheels to robot end effector

%% Update Bounds Every Frame
dTheta = object.dTheta; % Rotataion since prev frame
R_frame = Rz2d(rad2deg(dTheta));
p = (object.x.data(1:2) - object.x_prev(1:2));
robot.negBound = R_frame*robot.negBound + p;
robot.posBound = R_frame*robot.posBound + p;

%% Check Robot Mode (Measure, Correct, or STOP)
if robot.measureMode
    disp("Measurement Mode!")
    u_r = [0; 0; 0];

    measuredRotation = object.x.data(3) - robot.measuredTheta;

    %% Stop Robot Motion for 2 seconds
    robot.u.data = zeros(6,1);
    SerialCommunication(s1,robot,192,'u');
    pause(2)

    if abs(measuredRotation) < 15 % 20 degrees is working
        robot.u.data = zeros(6,1);
        SerialCommunication(s1,robot,192,'u');
        robot.stop = true; % STOP the experiment!
        disp("ALL DONE!")
        return;
    elseif measuredRotation > 0 % If rotating counter-clockwise
        newPosBound = Rz2d(measuredRotation) * robot.dxd(1:2);
        robot.posBound = newPosBound./norm(newPosBound);
    elseif measuredRotation < 0 % If rotating clockwise
        newNegBound = Rz2d(measuredRotation) * robot.dxd(1:2);
        robot.negBound = newNegBound./norm(newNegBound);
    end
 
    %% Since the bounds are defined from the robot center, but they actually 
    %% exist at the end effector, I need to rotate both of them OPPOSITE
    %% the agent's rotation    
    Ra = Rz2d(robot.x.data(3));
    robot.posBound = Ra*robot.posBound;
    robot.negBound = Ra*robot.negBound;
    
    robot.measuredTheta = object.x.data(3);

    robot.measureMode = false; % Switch measure mode off
    robot.correctionMode = true; % Switch att. correction mode on
    disp("Correction Activated")

    u_r(1:2,1) = Rz2d(object.x.data(3))'*robot.dxd;

elseif robot.correctionMode

    robot.u.data = zeros(6,1);
    SerialCommunication(s1,robot,192,'u');

    %% Get UNIT vectors of agent heading and object heading
    r = [robot.J(3,2);-robot.J(3,1)];
    r = r/norm(r);
    
    noW = Rz3d(object.x.data(3))*[r;0];
    nrW = Rz3d(robot.x.data(3))*[1;0;0];
    
    e_r = atan2d(noW(1)*nrW(2)-noW(2)*nrW(1), noW(1)*nrW(1)+noW(2)*nrW(2));
    e_r = 1+(e_r/2); % Chill with the error a bit (in place of D control)

    % Set stop condition
    if abs(e_r) <= 3 && abs(e_r - robot.e_prev) < 0.2
        %% Calculate new direction based on bounds
        R_a = Rz2d(robot.x.data(3)- object.x.data(3));
        newDirection = robot.negBound + robot.posBound;
        norm_newDirection = newDirection./norm(newDirection);
        temp = R_a'*(robot.v_desired * norm_newDirection(1:2)); % Our new desired velocity
        robot.dxd = [temp(1); temp(2)];
        robot.correctionMode = false; % Switch correction mode off
        disp("Correction Complete!")
        pause(2)
    end
    
    % Update our previous error and our measuredTheta
    robot.e_prev = e_r;
    robot.measuredTheta = object.x.data(3);

    u_r = [0; 0; robot.Kw * e_r];
else
    %% If not measure mode or correction mode, keep same heading
    disp('Keep on swimming!')
    u_r = [0; 0; 0];
    u_r(1:2,1) = robot.dxd;

end

% Calculate motor torque values wrt robot Jacobian
f = J_temp*u_r;  % was J_r

if robot.correctionMode % If correction mode, use original Jacobian
    disp("Correction Mode!")
    f = J_r*u_r;
end

%% Set robot motor torques | k=1 front left; k=2 rear; k=3 front right
for k=1:3
    if(f(k) >= 0)
        robot.u.data(2*k-1,1) = 0;
        robot.u.data(2*k,1) = uint8(abs(f(k)));
    else
        robot.u.data(2*k-1,1) = uint8(abs(f(k)));
        robot.u.data(2*k,1) = 0;
    end
end

