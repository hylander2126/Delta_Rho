function [f] = controller(X,J)

global mo Io
global Pi_o Rz

global Xd

Kp = 2;
Kd = 2;

e = [Rz(X(3))'*(Xd(1:2)-X(1:2)); Xd(3)-X(3)];

F = pinv(J)*[ Kp*e(1:2)-Kd*X(4:5) ; Kp*e(3)-Kd*X(6) ];

%f = saturate([F(1);F(2)],[-5 5]);
f =  [F(1);F(2)];



