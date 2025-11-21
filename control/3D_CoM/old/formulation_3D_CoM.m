%% Mathematical model testing. Shift Shadi and Siamak's equations to 3D and then isolate CoM components...
clc; clear; close all;

m           = 1;
v_dot       = [1; 0; 0];
omega       = [0; 0.1; 1];
omega_dot   = [0; 0.1; 1];
r_o         = [0; 0; 1];

centrip = cross(omega, cross(omega, r_o));
coriolis = cross(omega_dot, r_o);
f = m*(v_dot + centrip + coriolis);

% So f = [1; 0; 0]; Let's try to invert for r_o

R1 = (-norm(omega)^2 * eye(3) + (omega*omega') + skew(omega_dot))
r_o_inverse = pinv(R1) * (f/m - v_dot)


%% Now trying to find failure cases
clc;
disp("Failure test case analysis!")
% Fn to generate random test cases
ntests = 1000;
tol = 1e-6;

failures = [];
r_o_errors = [];

for test_idx = 1:ntests
    % Random test parameters
    m = 1;
    v_dot = rand(3, 1) * 2 - 1;
    omega = [0;0;0];%rand(3, 1) * 2 - 1;
    omega_dot = rand(3, 1) * 2 - 1;
    r_o = rand(3, 1) * 2 - 1;

    % Forward dynamics
    centrip = cross(omega, cross(omega, r_o)); % original vector form
    coriolis = cross(omega_dot, r_o);
    f = m*(v_dot + centrip + coriolis);

    % Inversion to recover r_o
    R1 = (-norm(omega)^2 * eye(3) + (omega*omega') + skew(omega_dot));
    r_o_inverse = inv(R1) * (f/m - v_dot);

    % Check if recovered matches original
    error = norm(r_o - r_o_inverse);
    if error > tol
        % Log failure details
        failures = [failures; test_idx];
        r_o_errors = [r_o_errors; error];
        fprintf('Test %d failed: Error = %.6f\n', test_idx, error);
        fprintf('Original r_o: [%f; %f; %f]\n', r_o);
        fprintf('Recovered r_o: [%f; %f; %f]\n\n', r_o_inverse);
    else
        % fprintf('r_o: [%.2f;%.2f;%.2f] and recovered: [%.2f;%.2f;%.2f]\n\n', r_o, r_o_inverse);

    end
end

% Report summary
if isempty(failures)
    fprintf('All tests passed successfully!\n');
else
    fprintf('%d out of %d tests failed.\n', numel(failures), ntests);
    fprintf('Maximum error: %.6f\n', max(r_o_errors));
end
