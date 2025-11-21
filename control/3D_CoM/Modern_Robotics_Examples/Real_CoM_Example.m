%% Now for the inverse case
% We want to determine an area the CoM could lie in given everything else!
% Known: $W\;H\;g\;r_{x+} \;r_{x-}$
% 
% Environment (unknown but in order to simulate): $\mu \;m\;$
% 
% *To Find:* $h_f$

%% Case study:
% We have pushed on the rectangular object which is resting on the table with force F_p
% We know the height of the push and we know the magnitude of the push.
% Determine the (approximate) CoM height OR height of the push above the CoM.
% The CoM is NOT at the center of the object and can be anywhere inside.
% We also happen to know the width and height of the rectangular box.
% We can perform another push at another height to get a second data point if needed.

% Modern Robotics Chapter 12 Manipulation
clc; clear; close all;

% Define variables
syms P_x B_x C_x h_c h_f mu m g F_p positive

P = [-P_x; h_f; 0];
B = [B_x; -h_c; 0];
C = [-C_x; -h_c; 0];
%% First we analyze sliding case

f_p = [F_p; 0; 0];
f_B = (1/sqrt(mu^2+1))*[-mu; 1; 0];
f_C = f_B;

F_p_skew = skew(P)*f_p;
F_B_skew = skew(B)*f_B;
F_C_skew = skew(C)*f_C;

FP = [F_p_skew(3); f_p(1:2)];
FB = [F_B_skew(3); f_B(1:2)];
FC = [F_C_skew(3); f_C(1:2)];

% Construct Fi matrix
Fi = [FB FC FP]
% External wrench due to gravity
F_ext = [0; -m*g; 0];

% Construct ki matrix
syms k1 k2 k3 real
ki = [k1; k2; k3];

% Now write EoM for quasistatic case of sliding
eqn = F_ext + Fi*ki == 0

% Solve for ki
s = solve(eqn, [k1 k2 k3], 'ReturnConditions',true)
k = [s.k1; s.k2; s.k3]

% Now apply constraint to ki
s1 = solve(k>=0)


%% Functions

function S = skew(v)
    S = [0 -v(3) v(2);
        v(3) 0 -v(1);
        -v(2) v(1) 0];
end