%% This function gets the previous O pos & vel, and A pos, vel, & acc, and 
%% returns the corresponding derivatives for each value. Uses robot and obj
%% classes to compute these values.

%% System Dynamics
%__________________________________________________________________________
function dy = systemDynamics(t,y,A,O)

    dy = zeros(length(y),1);
    po = y(1:3);
    dpo = y(4:6);
    O.move(po,dpo); % Move OBJECT to position and velocity (set O.p & O.dp)

    for i=1:length(A)
        % Move AGENT to next position, velocity (set A.p and A.dp)
        A(i).move(y(6*i+1 : 6*i+3), y(9*i+1 : 9*i+3));

        % Get new AGENT velocity and acceleration
        dy(6*i+1 : 9*i+3) = A(i).dynamics(O,t);
        
        % Move AGENT to same pos, vel, but new acc (set A(i).a)
        A(i).move(y(6*i+1 : 6*i+3), y(9*i+1 : 9*i+3), dy(9*i+1 : 9*i+3));
    end

    % Get new OBJECT velocity   
    dy(1:6) = O.dynamics(A);
    % Move OBJECT to same pos, vel, but new acc (set O.a)
    O.move(po,dpo,dy(4:6));
    drawnow();
end