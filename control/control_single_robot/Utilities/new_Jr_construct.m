%% Script to get new Jacobian based on desired null space control

%% ========= RECALL AGENT FRAME =========
%                       * Sensor EE
%                       .
%             beta2 /   .
%                  /    y
% Wheel 2 -> _    O - - ^ - - O  <- Wheel 1
%         h1 |     \    |    / \
%            -      \  {b}-->x  \ beta1
%            |       \     /
%         h2 |        \   /
%            |         \ /
%            _  beta3---O  <- Wheel 3
%                 |<--- w --->|
% NOTES: 
% wheel center to body edge     ~ 11 mm (more like 14 mm w/ new body)
% {b} to front edge             ~ 36.88 mm (more like 36 mm)
% front edge to sensor tip      ~ 50.1 mm (more like 52 mm)

function J = new_Jr_construct(varargin)
%% ========= Jacobian decomposition =========
% Distance 'd'
    w   = 88.7694 + 2*cosd(30)*11;  % mm, more like (~89 mm + ...)
    h1  = 25.6235 + sind(30)*11;    % mm, more like (~24 mm + ...)
    h2  = 51.25 + 11;             % mm, more like (~51 mm + ...)
% Wheel radius 'r'
    r = 19;                       % mm
% Angle between the wheels DRIVING DIRECTION and the positive X axis
    beta = [-pi/3; pi/3; pi];
%% Distance FROM center of rotation TO wheel center
    C = [w/2 h1; -w/2 h1; 0 -h2];                         % Confirmed
        
    if nargin
    % For rotation about sensor (default/resting state)
        sx = 0;                   % To sensor x from {b} (mm)
        sy = 36.88 + 0 + 50.1;   % To sensor y from {b} (mm) 
        % (36 is front edge to {b}, 50 is resting y distance to front edge, mid # is empirical factor)
        
        COR = [-sx -sy; -sx -sy; -sx -sy]; % center of rotation to {b}
        C = C + COR;
    end

%% Calculate Jacobian: formula from 'Modern Robotics'
    J = zeros(3);
    % Of the form (FR, FL, R)
    for i = 1:3
        J(i,:) = (1/r) * [1 0] * [cos(beta(i)) sin(beta(i)); -sin(beta(i)) cos(beta(i))] * [-C(i,2) 1 0; C(i,1) 0 1];
    end
    
    % For current robot where wheels are actually (FL, R, FR)
    J = [J(2,:); J(3,:); J(1,:)];
end
