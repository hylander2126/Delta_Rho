% Basic automated dance for two robots. Just using time-based actions.

time = toc;
K = K_max;
initial_pause = 1;

if time < initial_pause
    u_r = [0;0;0];  % Stop
end
if time > initial_pause
    u_r = [K;0;0];  % Forward
end
if time > initial_pause + 2
    u_r = [-K;0;0]; % Backward
end
if time > initial_pause + 4
    u_r = [0;-K;0]; % Left
end
if time > initial_pause + 6
    u_r = [0;K;0];  % Right
end
if time > initial_pause + 8
    u_r = [0;0;0];
    agent(1).stop = 1;
end


%% Set robot motor torques | k=1 front left; k=2 rear; k=3 front right
for n=1:N

    % Calculate required force using u_r direction and Jacobian
    f = agent(n).J * u_r;

    % Assign each agent's 'u' data for serial-sending in parent function
    for k=1:3
        if(f(k) >= 0)
            agent(n).u.data(2*k-1,1) = 0;
            agent(n).u.data(2*k,1) = uint8(abs(f(k)));
        else
            agent(n).u.data(2*k-1,1) = uint8(abs(f(k)));
            agent(n).u.data(2*k,1) = 0;
        end
    end
end