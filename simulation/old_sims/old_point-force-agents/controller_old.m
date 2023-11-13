function [f] = controller(X,J)

global mo Io Pi_o Rz Xd fL

Kp = 20;
Kd = 5;

e = [Rz(X(3))'*(Xd(1:2)-X(1:2)); Xd(3)-X(3)];

F = pinv(J)*[ Kp*e(1:2)-Kd*X(4:5) ; Kp*e(3)-Kd*X(6) ];

f = saturate([F(1);F(2)],fL);

end
