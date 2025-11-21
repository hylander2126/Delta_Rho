% Modern Robotics Chapter 12.1.2 Rolling, Sliding, and Breaking

clc; clear; close all;

% Working example of 2D case (SEE mod_rob_example.png) with stationary
% object B, with both objects onlay moving LINEARLY (no rot)

% Wrench
F = [0 0 0 1 -1 0]';

B_constraint = 0;
R_constraint = 0;
S_constraint = 0;

V_A = zeros(6,1);
V_B = zeros(6,1);

n_iters = 0;

while S_constraint == 0

% Twist(s)
V_A = [0 0 0 round(rand,3) round(rand,3) 0]'

B_constraint = F'*(V_A - V_B) > 0;
fprintf('Breaking Constraint: %i \n', B_constraint);

R_constraint = norm(V_A - V_B) == 0;
fprintf('Rolling Constraint: %i \n', R_constraint);

S_constraint = F'*(V_A - V_B) == 0;
fprintf('Sliding Constraint: %i \n', S_constraint);

    if B_constraint == 0 && R_constraint == 0 && S_constraint == 0
        fprintf('Invalid V_A, it violates the non-penetration constraint!! \n');
    end

n_iters = n_iters+1;
end

% Print V_A that satisfies sliding constraint
fprintf('Acceptable V_A found in %i iterations: [%i %i %i %i %i %i]\n', n_iters, V_A)


%% Trying to understand sliding constraint. How to find the line perpendicular 
% to F along which sliding is active? Of course, if the dot product of two vectors 
% == 0 then they are orthogonal! So naturally F'*V_A or F'*(V_A - V_B) when obj B
% is in motion equals zero when the Wrench (which is in the normal contact direction,
% which is normal to the contact surface aka into obj A) is orthogonal to the 
% twist (velocity) difference (V_A - V_B)...

%% And trying to understand the breaking constraint. For F'(V_A - V_B) > 0 
% to be true, the PROJECTION of the twist (velocity) difference (V_A - V_B)
% MUST be on the positive 'part' of wrench F
