%% Script to get new Jacobian based on desired null space control

function J = construct_robot_jacobian()

%% RECALL AGENT FRAME:
%                      _         O   <- Marker 1
%                      |        / \
%                      |       /   \
%                   2d |      /  x  \
%                      |     /   ^   \
%                      |    /    |    \
%   Marker 3 (2) ->    _   O y<--*     O  <- Marker 2 (3)
%
%                          |<--- d --->|
%
% RECALL wheel 1 starts from bottom left (see above) to 3 clockwise direct.

%% Jacobian decomposition:
% aw = angle between the wheels and the positive X axis
% p = distance from each wheel to either (center of robot OR end effector)

aw = [120; 0; -120]; % Wheel angles actual
% aw = [125; 0; 235]; % 2/12/23 had to adjust wheel angles to make front two wheels more agressive (new wheels)

%% For rotation about robot center
% d = 1.1547;
% p = [d/2 -d/2; d*0.433 0; d/2 d/2];
% p = [0 -d/2; sqrt((2*d)^2 - (d/2)^2) 0; 0 d/2]; % Experimental...

%% For rotation about end effector
% dx = 0.372; % Original
% dx = 0.46;
dx = 0.36;
% dy = 0.478; % Original
% dy = 0.60;
dy = 0.45;

p = [dx -dy; 1.15 0; dx dy]; % 1.15 originally
% p = [dx -dy; 1.36 0; dx dy]; % 1.36 w new wheels

%% Calculate Jacobian: formula from "A Decentralized, Comm..." Paper
J = zeros(3);
for i = 1:3 % For each wheels
%     J(:,i) = 2*[-sind(aw(i)); cosd(aw(i)); p(i,1)*cosd(aw(i)) - p(i,2)*sind(aw(i))]; % Original directly from paper
    J(:,i) = -[-sind(aw(i)); cosd(aw(i)); p(i,1)*cosd(aw(i)) - p(i,2)*sind(aw(i))]; % Need to flip signs for intuitive keyboard ctrl
end

% Jacobian needs to be transposed for all other functions. Each row now represents a wheel
J = real(J');

% J(2,3) = -1*J(2,3); % This just swaps the rotation control - fine for now

end
