function dy = systemDynamics(t,y,pd,A,O)

%__________________________________________________________________________
% y vector decomposition
%
% y(1:6) = [po' dpo']';
% y(6*i+1 : 6*i+3) = pi';

dy = zeros(length(y),1);


po = y(1:3);
dpo = y(4:6);

O.move(po,dpo);

F = [0;0];
M = 0;

for i=1:length(A)

    A(i).move(y( 6*i+1 : 6*i+3 ));
    
    dy(6*i+1:6*i+3) = A(i).kinematics(O);
    A = A(i);

    [f,m] = A(i).agentForce(O,dy(7:9));
 
    F = F + f;
    M = M + m;
end


dy(1:6) = O.dynamics(F,M,A,dy(7:9));

drawnow();
end