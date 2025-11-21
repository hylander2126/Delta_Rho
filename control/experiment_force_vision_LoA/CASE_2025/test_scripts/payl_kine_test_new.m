clear; close all; clc;

vr = [1; 4; 0];
sigma = [1; -2; 0];
phi = findAngle2(vr, sigma);
rad2deg(phi)

%% Default definition of ang vel
omega = cross(sigma, vr)/norm(sigma)^2;
%% Scalar definition of ang vel (NOT VALIDATED)
omega_scalar = norm(sigma) * norm(vr) * sin(-phi)/norm(sigma)^2 * [0;0;1];
%% Simplified determinant definition of ang vel (validated)
omega_simplified = (sigma(1)*vr(2) - vr(1)*sigma(2))/norm(sigma)^2 * [0;0;1];

%% Default definition of v_app/r
v_app_r = cross(omega, -sigma);
%% Scalar interpretation of v_app/r (NOT VALIDATED)
n = Rz2(sign(omega(3))*pi/2) * -sigma(1:2);
n_hat = n/norm(n);
v_app_r_scalar = norm(omega)*norm(sigma) * n_hat;


v_app = vr + v_app_r;
v_app_scalar = vr + [v_app_r_scalar; 0];
% fix rounding error
round(v_app_scalar);

twist_p = v_app + omega;
twist_p_scalar = v_app_scalar + omega_simplified;

%%% Now trying to do inverse:
% We only know a desired payload motion, and sigma (from CoM estimate):
des_ang = omega(3)/2
des_lin = twist_p(1:2)

% Find the required robot tangential vel
vr_tangent = des_ang*norm(sigma)

% Find the required robot linear vel
% We can determine n_hat using omega and sigma
n_calc = -Rz2(sign(des_ang)*pi/2)*sigma(1:2);
n_hat_calc = n_calc/norm(n_calc);
vr_linear = des_lin(1:2) - norm(des_ang)*norm(sigma) * n_hat_calc;


% And a final test with simpler linear vel equation:
vr_linear_simple = des_lin(1:2) + norm(des_ang) * -n_calc

function angle = findAngle2(v1, v2)
    % Find angle between v1 and v2, following Right-hand-rule
    x1 = v1(1);
    y1 = v1(2);
    x2 = v2(1);
    y2 = v2(2);
    angle = atan2(x1*y2-y1*x2 , x1*x2+y1*y2);
end