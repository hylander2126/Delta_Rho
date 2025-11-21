%% Callback function to get keystroke

function getKey(~,event)

global agent
global s1

N = size(agent,2);
% u_r = [0;0;0]; % agent.u_r; % <this keeps the robots moving when not pressing a key

K = agent(N).K;


% J_r = [1.7322, 1, 1;
%        0, -2, 1;
%        -1.7322, 1, 1]; % Original ~rotate about robot centroid
% J_r = [-1.7321,-1.0000,0.4559;
%           0,2.0000,2.0000;
%           1.7321,-1.0000,0.4559]; % Updated 02/06/2023


c = event.Key;

    %% Method to change agent we are controlling - I ELIMINATED THIS
%         if(~isempty(str2num(c))) %#ok<*ST2NM> 
%             if(str2num(c) <= N  && str2num(c) > 0)
%                 agent(num).u_data = zeros(6,1);
%                 for i=1:5
%                     agent(i).num = str2num(c);
%                 end
%                 fprintf('Changing agent to #%d\n',str2num(c));
%             end
%         end
    %%

switch(c)
    case 'leftarrow'
        u_r = [0;-K;0];
    case 'rightarrow'
        u_r = [0;K;0];
    case 'uparrow'
        u_r = [K;0;0];
    case 'downarrow'
        u_r = [-K;0;0];
    case 'z'
        u_r = [0;0;K];
    case 'x'
        u_r = [0;0;-K];
    case 's'    
        u_r = [0;0;0];
    case 'hyphen'
        K = K - 1;
        if(K < 1)
            K = 0;
        end
        u_r(u_r ~= 0) = K;
        fprintf('K changed to %d \n',K);
    case 'equal'
        K = K + 1;
        fprintf('K changed to %d \n',K);
        u_r(u_r ~= 0) = K;
    case 'escape'
        fprintf('Exiting control...\n');
        for i=1:N
            agent(i).stop = 1; % Set an 'exit' condition to the first agent.
        end
        return
end


% Update all agent's gains
for i=1:N
    agent(i).Kp = K;
end
    

% Trying to fix 'sticky' motors by sending a momentary high pulse to the active motors:
% TEMP 02/06/2023 - front two wheels don't get enough gain to actually
% spin, testing by amplifying this gain
% if u_r(3) ~= 0
%     f(1) = f(1)*1.85;
%     f(3) = f(3)*1.85;
% end


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
   
end

