%% Helper function to get keystroke
function getKey(~, event, A)
%     global robot n N exit K u_r
    
    K = A.K;
    num = A.num;
    N = A.N;
    u_r = A.u_r;

    map = 1; % Turn on if you want to control
    c = event.Key

    %% Method to change robot we are controlling - I ELIMINATED THIS
    if(~isempty(str2num(c))) %#ok<*ST2NM> 
        if(str2num(c) <= N  && str2num(c) > 0)
            A(num).u_data = zeros(6,1);
            for i=1:5
                A(i).num = str2num(c);
            end
            fprintf('Changing robot to #%d\n',str2num(c));
        end
    end
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
            for i=1:5
                A(i).exit = 1; % Set an 'exit' condition to the first robot.
            end
    end
    
    for i=1:N
        A(i).K = K;
    end
    
    if(map == 1)
        matrix = [1.7322, 1, 1;
                0, -2, 1;
                -1.7322, 1, 1];
        f = matrix*u_r; % f = [3x1]
        for i=1:length(f) % Loop through f ~ motor values [front L, rear, front R]'
            for ii=1:N % Loop through each robot
                if(f(i) >= 0) % Set positive registry to our gain
                    A(ii).u_data(2*i-1,1) = 0;
                    A(ii).u_data(2*i,1) = uint8(abs(f(i)));
                else
                    A(ii).u_data(2*i-1,1) = uint8(abs(f(i)));
                    A(ii).u_data(2*i,1) = 0;
                end
            end
        end
    end
end