function [robots] = update_robots(s1, N, robots)

% RECALL AGENT AND SENSOR FRAMES:
%                           
%                      ^ x
%                      |
%         _      O     *-->y O  <- Marker 3 (2)
%         |       \         / 
%         |        \       /
%      2d |         \     /
%         |          \   /
%         |           \ /
%         _            O        <- Marker 1 (1)
%                |<--------->|
%                      d
%
%                      O p5
%                     / \
%                    /   \
%                l3 /     \ l4
%                  /       \
%                 /    ^ y  \
%                 \    |    /
%                  O   *---O--> x

% u_r = [z; x; y]
    u_r = [80; 0; 0];

    %% Set new robot motor values
    for i=1:N
        robots(i) = robots(i).setTorques(u_r);
    end
end
