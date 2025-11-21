clear, cla, close all;
    
% Simulation of double-crank mechanism moving

% Define link lengths
l1 = 15;
l2 = 35;
l3 = 35;
l4 = 15;
l5 = 30;

% First and Last points
Ax = 0;
Ay = 0;
Ex = l5;
Ey = 0;

% Configurations for base angles
n  = 300; % Number of configurations to try
theta1 = linspace(0,4.5*pi,n);
theta2 = linspace(0,2*pi,n);

thetaArray = [theta1; theta2]';

% Calculate coords of joints 2 and 4
Bx = l1*cos(theta1);
By = l1*sin(theta1);
Dx = l5 + l4*cos(theta2);
Dy = l4*sin(theta2);

% Distance between joints 2 and 4
d  = sqrt((Dx - Bx).^2 + (Dy - By).^2);

% if any(abs(l3-l4) > d | d > l3+l4)
%     msgbox('Simulation is impossible. Check dimensions');
%     return
% end

% Calculate angle between d and l2
alpha = acos((l2.^2 + d.^2 - l3.^2)./(2*l2.*d));

% Calculate "V" vector with length l2 along d
vx = l2*(Dx - Bx)./d;
vy = l2*(Dy - By)./d;

% CALCULATE E.E. X;Y by rotating V vector

Cx =  (vx.*cos(alpha) - vy.*sin(alpha) + Bx)';
Cy =  (vx.*sin(alpha) + vy.*cos(alpha) + By)';

xlim([min(Bx)-5 max(l5)+5])
ylim([min(Dy)-5 max(Cy)+5])

axis equal
hold on

for i = 2:n
    j = i-1:i;
    plot(Cx(j),Cy(j),'Color','r')
	h = plot([Ax Bx(i) Cx(i) Dx(i) Ex], ...
             [Ay By(i) Cy(i) Dy(i) Ey], '-ok');
%     if i==3
%         input('enter')
%     end
	pause(0.05);
    delete(h)
end
hold off