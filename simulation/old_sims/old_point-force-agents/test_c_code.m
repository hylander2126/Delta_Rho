clc; clear; close all;


Pi_o = [-10 -100  50;
         60  10   -46];



N = size(Pi_o,2);

J = construct_jacobian(Pi_o);

for i=1:N
    J_(:,:,i) = construct_jacobian(Pi_o(:,i));
end





% Rz = @(q) [cos(q) -sin(q); sin(q) cos(q)];
% 
% x =[0;0;0];
% xd = [10;10;0];
% xO = [0;0;0];
% 
% 
% Kp = 10;
% Kd = 0;
% 
% e = [Rz(xO(3))'*(xd(1:2)-xO(1:2)); xd(3)-xO(3)];
% 
% F = pinv(J_(:,:,1))*[ Kp*e(1:2); Kp*e(3) ];
% 
% f = [F(1);F(2)];
% 
% 
% fR = Rz(x(3))'*Rz(xO(3))*f;