%% Get joystick force feedback from controller and convert to robot motor
%% values. ***** MUST BE RUN IN A LOOP *****

%% RECALL AGENT FRAME:
%                      _         O   <- Marker 1
%                      |        / \
%                      |       /   \
%                   2d |      /  x  \
%                      |     /   ^   \
%                      |    /    |    \
%   Marker 3 (2) ->    _   O y<--*     O  <- Marker 2 (3)
%
%                          |<--- d --->|
%

%% Get Joystick values
[analog,button,dPad] = read(joy);

K = agent.K;

% Analog decomposition
% (1:2) Left  joystick x and y values
% (3)   Trigger values (left is - right is +)
% (4:5) Right joystick x and y values
c = [-analog(2), analog(1), -analog(3)];

% Set analog deadzone
dz = .05;
c(find(c>-dz & c<dz)) = 0;


% Get D-Pad input
switch(dPad)
    case 0
        c = [-1, 0, 0];
    case 90
        c = [0, 1, 0];
    case 180
        c = [1, 0, 0];
    case 270
        c = [0, -1, 0];
end


% Set desired direction
u_r = K*c';

% Set stop condition
if any(button)

    fprintf('Exiting control...\n');
    for n=1:N
        agent(n).stop = 1; % Set an 'exit' condition to the first agent.
    end
    return
end

% Set robot motor torques | k=1 front left; k=2 rear; k=3 front right
for n=1:N
    f = agent(n).J * u_r
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