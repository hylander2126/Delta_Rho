function [pitch, yaw, roll] = quat2euler(q)
%QUAT2EULER Quaternion to Euler angles conversion
% Input quaternion q should be in the form [qw, qx, qy, qz] where
% qw is the scalar component
    
    % Extract the scalar and vector parts of the quaternion
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    
    % Calculate the Euler angles from the quaternion
    % roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz);
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    % pitch (y-axis rotation)
    % sinp = 2 * (qw * qy - qz * qx);
    % if abs(sinp) >= 1
    %     pitch = copysign(pi / 2, sinp); % use 90 degrees if out of range
    % else
    %     pitch = asin(sinp);
    % end
    sinp = -2 * (qw * qy - qz * qx); % Negate to invert direction
    pitch = atan2(sinp, sqrt(1 - sinp^2));
    
    % yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy);
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    yaw = atan2(siny_cosp, cosy_cosp);
    
    % Return the Euler angles in radians
end

