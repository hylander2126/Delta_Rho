% X(1 : 6)       ->     Position and velocity of the main object
%                       X(1:3) = [xo   yo  qo]
%                       X(4:6) = [dxo dyo dqo]
%
function dX = system_dynamics(~,X)

global mo Io Pi_o

global J J_

dynamics = @(Y,m,I,F) [Y(4)*cos(Y(3)) - Y(5)*sin(Y(3));
                       Y(4)*sin(Y(3)) + Y(5)*cos(Y(3));
                       Y(6);
                       F(1)/m + Y(6)*Y(5);
                       F(2)/m - Y(6)*Y(4);
                       F(3)/I];



N = size(Pi_o,2);

f = [0;0];
M = 0;

for i=1:N
    [f_] = controller(X,J_(:,:,i));
    f = f + f_;
    M = M + J(3,1:2,i)*f_;
end


dX = dynamics(X,mo,Io,[f;M]);
end

