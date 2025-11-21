function [robots, u_r] = controllerCoM(robots, opti_theta, u_r)
    
%% ________________  COM ESTIMATE  ______________________
% Bound convergence (binary search) method from first paper (2023)


%% Continually update bound direction, track payload's change in orientation
robots(1).negBound = Rz2(opti_theta(2)) * robots(1).init_negB;
robots(1).posBound = Rz2(opti_theta(2)) * robots(1).init_posB;


% CONDITIONAL update either bound based on +/- payload rotation
rot_amount = opti_theta(2) - robots(1).baseline_rot;

if abs(rot_amount) > deg2rad(3) % empirical value 3 degrees
    bisect_angle = abs(getAngle(robots(1).negBound, robots(1).posBound, 'r'))/2; % 1/2 bounds angle
    if rot_amount < 0
        disp('counter-clockwise rotation!!')
        robots(1).negBound = Rz2(bisect_angle) * robots(1).negBound;
        robots(1).init_negB = Rz2(bisect_angle) * robots(1).init_negB;
    elseif rot_amount > 0
        disp('clockwise rotation!!')
        robots(1).posBound = Rz2(-bisect_angle) * robots(1).posBound;
        robots(1).init_posB = Rz2(-bisect_angle) * robots(1).init_posB;
    end

    % Set new 'baseline' (comparison value) to current
    robots(1).baseline_rot = opti_theta(2);
end


% Determine new GLOBAL motion direction (this may be the issue with force feedback method...)
direction = robots(1).negBound + robots(1).posBound;
% Convert global control input to local robot input
u = Rz2(-opti_theta(1)) * direction/norm(direction);
% Assign to robot control input
u_r(2:3) = 2900 * u; % 3500 * u;

end
