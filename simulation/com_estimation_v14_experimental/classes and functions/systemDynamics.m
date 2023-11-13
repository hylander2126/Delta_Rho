function [dy, bounds, desired_direc]= systemDynamics(t,y,pd,A,O,Q)

%__________________________________________________________________________
% y vector decomposition
%
% y(1:6) = [po' dpo']';
% y(6*i+1 : 6*i+3) = pa_i';

dy = zeros(length(y),1);

po = y(1:3);
dpo = y(4:6);

O.move(po,dpo);

F = [0;0];
M = 0;

% Agent and Payload dynamics
for i=1:length(A)

    A(i).move(y( 6*i+1 : 6*i+3 ));
        
    [f,m] = A(i).agentForce(t,O);
 
    F = F + f;
    M = M + m;
    
    dy(6*i+1:6*i+3) = A(i).kinematics(t,O);
end

dy(1:6) = O.dynamics(F,M);

global Bounds
Bounds = [Bounds; t, A.negBound', A.posBound'];


% bounds = [A.negBound', A.posBound'];
% desired_direc = A.actual_direc'; % A.desired_direc';

% drawnow();