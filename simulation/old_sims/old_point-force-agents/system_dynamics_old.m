% X(1 : 6)       ->     Position and velocity of the main object
%                       X(1:3) = [xo   yo  qo]
%                       X(4:6) = [dxo dyo dqo]
%
function dX = system_dynamics(~,X)

global mo Io us uk J J_ N W


eps = 0.01;


dynamics = @(Y,m,I,F) [Y(4)*cos(Y(3)) - Y(5)*sin(Y(3));
                       Y(4)*sin(Y(3)) + Y(5)*cos(Y(3));
                       Y(6);
                       F(1)/m + Y(6)*Y(5);
                       F(2)/m - Y(6)*Y(4);
                       F(3)/I];


f = [0;0];
M = 0;

for i=1:N
    [f_] = controller(X,J_(:,:,i));
    f = f + f_;
    M = M + J(3,1:2,i)*f_;
end


%--------------------------------------------------------------------------
%   Friction effect
if(norm(X(4:5)) <= eps)
    if(norm(f) <= us*W)
        f = [0;0];
    end
else
    f = f -uk*W*sign(X(4:5));
end

if(abs(X(6)) <= eps)
    if(M <= us*W)
        M = 0;
    end
else
    M = M -uk*W*sign(X(6));
end


%--------------------------------------------------------------------------
%   Updating dX

dX = dynamics(X,mo,Io,[f;M]);
end

