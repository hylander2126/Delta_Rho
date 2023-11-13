function [value,isterminal,direction] = instability(t,x)

% Locate the time when y passes through 0.111 in all 
% directions and stop integration.
value = norm(x(1:2)) - 20;  % Detect y = 0.111
isterminal = 1;        % Stop the integration
direction = 0;         % All direction