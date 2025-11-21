%% Get initial bounds

hardWay = false;

% Rotation Matrix Function
Rz2d = @(q)[cosd(q),-sind(q);sind(q),cosd(q)];

if hardWay

    % Get nearest points
    bodyPoints = Markers{1}*1000;
    A_pos = robot(N).x.data(1:2)';
    O_pos = object.x.data(1:2)';
    r = O_pos(1:2) - A_pos(1:2); % 1x3 Vector from A to O
    
    localEE = [-.070; 0; 0]; % End effector location wrt Robot frame
    EE_pos = [A_pos'; 0] + Rzd(rad2deg(qa_0))*localEE;
    EE_pos = EE_pos(1:2)';
    
    k = dsearchn(bodyPoints,EE_pos); % Index of nearest point
    
    % Get first bound and make wrt A_pos
    firstBound = bodyPoints(k,:) - A_pos;
    firstBound = (firstBound/norm(firstBound))'; % 2x1
    
    % Check direction and angle of first bound and set the second bound
    c = cross([r';0], [firstBound;0]);
    theta = sign(c(3))*180/pi*atan2(norm(c),dot(r,firstBound'));
    
    %% MUST CHANGE FROM 180 TO 2*THETA FOR DIFFERENT OBJECTS
    % Mirror firstBound ALMOST to 180 degrees
    % secondBound = (Rzd(sign(c(3))*180)*firstBound')';
    secondBound = (Rz2d(sign(c(3))*theta*2)*firstBound);
    
    if sign(c(3)) > 0
        robot(N).posBound = secondBound; % 2x1
        robot(N).negBound = firstBound; % 2x1
    else
        robot(N).posBound = firstBound;
        robot(N).negBound = secondBound;
    end

else
    posBound = Rz2d(robot(N).x.data(3)) * [-1;1000];
    negBound = Rz2d(robot(N).x.data(3)) * [-1;-1000];
    robot(N).posBound = posBound/norm(posBound);
    robot(N).negBound = negBound/norm(negBound);

end
