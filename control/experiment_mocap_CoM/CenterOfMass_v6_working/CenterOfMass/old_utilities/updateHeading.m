function updateHeading(A, time)
%updateHeading Function to update the robot's bounds and its heading upon
%every rotation check every T time steps. Should create motor values and
%set the *robots* u-data to the values, which is then used in top-level for
%SerialCommunication to send to robot(s). See *getKey* fn for example

    K = A.K;
    num = A.num;
    N = A.N;
    u_r = A.u_r;

    tInt = 10;
%     u_r = [0;0;K];

%     endEffectorMat = [-1.7, 1.7322, 1;
%                       -4.7, 0, 1;
%                       -1.7, -1.7322, 1];
% 
%     u_r = endEffectorMat*u_r;

    if rem(time,tInt)==0
        fprintf('time is: %d \n',time);
        [negBound, posBound] = getBounds(A,time);
        newDirect = negBound + posBound;
        normNewDirect = newDirect./norm(newDirect);
        
        u_r = K*normNewDirect;
        A.u_r = u_r;
    end

    matrix = [1, 1.7322, 1;
              -2, 0, 1;
             1, -1.7322, 1];


%     matrix = [1.7322, 1, 1;
%               0, -2, 1;
%               -1.7322, 1, 1];

    f = matrix*u_r; % f = [3x1]

    % Loop through f ~ motor values [front L, rear, front R]'
    for i=1:length(f)

        % Loop through each robot
        for ii=1:N

            % Set corresponding registry to our gain "K"
            if(f(i) >= 0)
                A(ii).u_data(2*i-1,1) = 0;
                A(ii).u_data(2*i,1) = uint8(abs(f(i)));
            else
                A(ii).u_data(2*i-1,1) = uint8(abs(f(i)));
                A(ii).u_data(2*i,1) = 0;
            end
        end
    end
end