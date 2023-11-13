clc; clear; close all;

%% Determine angle between two vectors

% Method 1 - DOES NOT WORK AS INTENDED
u = [0.5;-0.5];
v = [1;0];
q = [0;1];
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
degrees = CosTheta * 180 / pi;

% Plot
quiver(0,0,u(1),u(2),1)
hold on
quiver(0,0,v(1),v(2),1)
quiver(0,0,q(1),q(2),1)
hold off

% Method 2 - WORKS PROPERLY BUT ONLY RETURNS VALUES UNDER 180 DEGREES
u3 = [u;0]
v3 = [v;0]
q3 = [q;0]
atanTheta = atan2(norm(cross(u3,v3)),dot(u3,v3))
degrees = atanTheta * 180 / pi

% Plot
quiver(0,0,u3(1),u3(2),1)
hold on
quiver(0,0,v3(1),v3(2),1)
quiver(0,0,q3(1),q3(2),1)
hold off