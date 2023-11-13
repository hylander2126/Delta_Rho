function dydt = new_systemDynamics(t,y,pd,A,O,Q)
% v1 - This function takes in y, Object, Agent, & t span
% Uses the com function in robot file to calculate force needed to minimize
% angle between force vector and object's velocity vector. TODO change
% function to instead control robot's velocity (not force)
%__________________________________________________________________________
% y vector decomposition
%
% y(1:6) = [po' dpo']';
% y(6*i+1 : 6*i+3) = [pi'];

dydt = zeros(length(y),1);

po = y(1:3);
dpo = y(4:6);

O.move(po,dpo); % Update object's position

F = [0;0];
M = 0;

for i=1:length(A) % For each agent
    
    % Update agent's position
    A(i).move(y(6*i+1 : 6*i+3));
    
    % Add agent velocity to output y
    dydt(6*i+1:6*i+3) = A(i).kinematics(O); % Robot kinematics fn.

    % Update agent force
    [F,M] = A(i).com(O);
end

% Add object velocity, acceleration to output y
dydt(1:6) = O.dynamics(F,M); % Payload dynamics fn.


drawnow();