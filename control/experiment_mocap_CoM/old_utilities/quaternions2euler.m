function [psi,theta,phi] = quaternions2euler(q)

% Phi is the signed angle between the x axis and the N axis (x-convention 
% – it could also be defined between y and N, called y-convention).

% Theta is the angle between the z axis and the Z axis.

% Psi is the signed angle between the N axis and the X axis (x-convention).

phi = atan2(2*(q(1)*q(2)+q(3)*q(4)), 1 - 2*(q(2)*q(2)+q(3)*q(3)));
theta = asin(2*(q(1)*q(3)-q(4)*q(2)));
psi = atan2(2*(q(1)*q(4)+q(2)*q(3)), 1 - 2*(q(3)*q(3)+q(4)*q(4)));

% phi = atan2(2*(q(3)*q(4)-q(1)*q(2)),2*(q(1)*q(1)-q(4)*q(4))-1);
% theta = -asin(2*(q(2)*q(4)+q(1)*q(3)));
% psi = atan2(2*(q(2)*q(3)-q(1)*q(4)),2*(q(1)*q(1)-q(2)*q(2))-1);



