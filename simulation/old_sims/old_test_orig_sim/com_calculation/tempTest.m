clc; clear; close all;
%% Find the angle between two vectors

% Vectors of form (x,y)
o = [0 0];
u = [-.5 1];
v = [2 1];

% Plot the vectors from origin o
quiver(o(1), o(2), u(1), u(2), 0, Color='r')
hold on
quiver(o(1), o(2), v(1), v(2), 0, Color='b')

% Find first midVector
% [xy,theta] = midVector(u,v);

% Vector with same magnitude as original V vector
theta = midAngle(u,v)/2;
Rz(theta)
f = Rz(theta)*v'
f = reshape(f, [2,1])
newLengthOfMidV = norm(f)
quiver(o(1),o(2),f(1),f(2),0, Color='g')

% Trying to just add the original vectors for the new direction
% K = sum(u,v)
% plotQuiver(K);
% L = sum(u,K)
% plotQuiver(L);
% P = sum(u,L)
% plotQuiver(P);
% X = sum(u,P)
% plotQuiver(X);

%% FUNCTIONS
function pl = plotQuiver(v1)
    o = [0 0];
    pl = quiver(o(1), o(2), v1(1), v1(2), 0, Color='m');
end


function q = sum(v1,v2)
    q = v1/norm(v1)+v2/norm(v2);
end

function theta = midAngle(v1,v2)    
    % Find costheta of vectors

    CosTheta = dot(v1,v2)/(norm(v1)*norm(v2));
    % Inv cos of that value in degrees
    theta = acos(CosTheta);
end

function [q,theta] = midVector(v1,v2)
    % MUST GIVE V1=STATIONARY VECTOR AND V2=MOVING VECTOR
    theta = midAngle(v1,v2)/2; % Half for simplicity outside fn

    % Angle between vector v2 and x-axis
    deltaTheta = midAngle(v2,[1 0]);

    % Vector between the two original vectors
    newTheta = theta*2+deltaTheta;

    q = [cos(newTheta/2) sin(newTheta/2)];
end

function R = Rz(q)
R = [cos(q) -sin(q); sin(q) cos(q)];
end

