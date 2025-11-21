%% Get rigidBody, marker location, and quaternion Q of N rigid bodies

% _________ NOTE: _______________
%
% !! RUN init_optitrack.m FIRST !!
% (or Matlab may crash)
% _______________________________

if ~exist('theClient', 'var')
    disp('OPTITRACK NOT INITIALIZED!!!')
    optitrack_init
    pause(2)
    return;
end

%% Information from OptiTrack
frameOfData = theClient.GetLastFrameOfData();

%% Assign rigid body objects from OptiTrack
nRigidBodies = frameOfData.nRigidBodies;
for n = 1:nRigidBodies
    rigidBody{n} = frameOfData.RigidBodies(n); % Getting rigid body 1 information
end

%% rigidBody Marker Locations
% for n=1:nRigidBodies
%     nMarkers = rigidBody{n}.nMarkers; % Number of markers per rigid body
%     for i=1:nMarkers
%         X(i,:) = rigidBody{n}.Markers(i).x;
%         Y(i,:) = rigidBody{n}.Markers(i).y;
%     end
%     Markers{n} = [X Y];
%     X = [];
%     Y = [];
% end


%% RigidBody translation and rotation
global opti_T opti_theta

for n = 1:nRigidBodies
    body = rigidBody{n};
    
    x = body.x;
    y = body.y;
    z = body.z;
    qw = body.qw;
    qx = body.qx;
    qy = body.qy;
    qz = body.qz;

    % Translation
    opti_p = double([body.x; body.y; body.z]); %double([x(n); y(n); z(n)]);
    % Quaternion
    opti_q = double([body.qw; body.qx; body.qy; body.qz]); % double([qw(n) qx(n) qy(n) qz(n)]');
    % Rotation Matrix
    opti_R = quat2rot(opti_q);
    % Transformation matrix
    opti_T(:,:,n) = [opti_R opti_p; 0 0 0 1];
    % Rigidbody rotation: Wrapped -180 to 180 degrees
    [roll, pitch, yaw] = quat2euler(opti_q);
    opti_theta(n) = pitch; % atan2(opti_R(2,1), opti_R(1,1)); % theta = atan(sin(theta)/cos(theta))
end


%% UNUSED OLD CODE
% Q(1) = rigidBody.qw;
% Q(2) = rigidBody.qx;
% Q(3) = rigidBody.qy;
% Q(4) = rigidBody.qz;

% Rotation = quat2rotm(Q);
% 
% T_rigidbody1 = [Rotation Translation];
% 
% T_rigidbody1 = [T_rigidbody1;
%               0 0 0 1]

% %% rigidbody 2          
% RigidBody_2 = frameOfData.RigidBodies(2);
% 
% x = RigidBody_2.x;
% y = RigidBody_2.y;
% z = RigidBody_2.z;
% 
% Translation = [x;y;z];
% 
% Q(1) = RigidBody_2.qw;
% Q(2) = RigidBody_2.qx;
% Q(3) = RigidBody_2.qy;
% Q(4) = RigidBody_2.qz;
% 
% Rotation = quat2rotm(Q);
% 
% T_rigidbody2 = [Rotation Translation];
% 
% T_rigidbody2 = [T_rigidbody2;
%               0 0 0 1]

%%
%     RigidBody_3 = frameOfData.RigidBodies(3);
%     RigidBody_4 = frameOfData.RigidBodies(4);
%     RigidBody_5 = frameOfData.RigidBodies(5);

          
%     Q_2(1) = RigidBody_2.qw;
%     Q_2(2) = RigidBody_2.qx;
%     Q_2(3) = RigidBody_2.qy;
%     Q_2(4) = RigidBody_2.qz;

%     Q_3(1) = RigidBody_3.qw;
%     Q_3(2) = RigidBody_3.qx;
%     Q_3(3) = RigidBody_3.qy;
%     Q_3(4) = RigidBody_3.qz;
%     
%     Q_4(1) = RigidBody_4.qw;
%     Q_4(2) = RigidBody_4.qx;
%     Q_4(3) = RigidBody_4.qy;
%     Q_4(4) = RigidBody_4.qz;
%     
%     Q_5(1) = RigidBody_5.qw;
%     Q_5(2) = RigidBody_5.qx;
%     Q_5(3) = RigidBody_5.qy;
%     Q_5(4) = RigidBody_5.qz;

%     R_1 = quat2rotm(Q); % Coneverts quaternions to rotation matrix
%     R_2 = quaternions2R(Q_2);
%     R_2 = quaternions2R(Q_2);
%     R_3 = quaternions2R(Q_3);
%     R_4 = quaternions2R(Q_4);
%     R_5 = quaternions2R(Q_5);
    %[q,~,~] = quaternions2euler(Q); % Converts quaternions to Euler (z,y,x)
    
%     v_1 = R_1(1,:);
%     v_2 = R_2(1,:);
%     v_3 = R_3(3,:);
%     v_4 = R_4(3,:);
%     v_5 = R_5(3,:);
    
%      theta_1 = acosd(v_1*v_2'/(norm(v_1)*norm(v_2)));
%     theta_2 = acosd(v_2*v_3'/(norm(v_2)*norm(v_3)));
%     theta_3 = acosd(v_3*v_4'/(norm(v_3)*norm(v_4)));
%     theta_4 = acosd(v_4*v_5'/(norm(v_4)*norm(v_5)));
%    theta_1 = atan2(norm(cross(v_1,v_2)), dot(v_1,v_2))*180/pi;
%     theta_2 = atan2(norm(cross(v_2,v_3)), dot(v_2,v_3))*180/pi;
%     theta_3 = atan2(norm(cross(v_3,v_4)), dot(v_3,v_4))*180/pi;
%     theta_4 = atan2(norm(cross(v_4,v_5)), dot(v_4,v_5))*180/pi;
%      fwrite(s,'A');
%      fwrite(s,'a');
%      fwrite(s,num2str(theta_1));
%      fwrite(s,'b');
%      pause(0.001);
%      fwrite(s,'-10.11');
%      fwrite(s,'&');
%      pause(0.001);
%      fwrite(s,'-30.22');
%      fwrite(s,'&');
%      pause(0.001);
%      fwrite(s,'-90.98');
%      %fwrite(s,'800.12');
%      fwrite(s,'a');
%      pause(0.001);
    
%     Theta1=[Theta1;theta_1];
%     Theta2=[Theta2;theta_2];
%     Theta3=[Theta3;theta_3];
%     Theta4=[Theta4;theta_4];
    
%     v_2 = R_2(3,:);
%     theta = acosd(v_1*v_2'/(norm(v_1)*norm(v_2)));
    
    % Updating plots (You may ignore this part since it is not part of
    % getting data from optiTrack)
%Data Output
% fid=fopen('Results.csv','wt');
% X=[X;x];
% Y=[Y;y];
% Z=[Z;z];
% Theta=[Theta;theta];
% fprintf(fid,'%d,%f\n',X,Y,Z,Theta);
% % fprintf(fid,X,Y,Z,Theta);
% fclose(fid);    
% 
% fid=fopen('Thetas.csv','wt');
% fprintf(fid,'%f,%f,%f,%f\n',Theta1,Theta2,Theta3,Theta4);
% fclose(fid);


 %    o = [x;y;z]*ones(1,3);
%      c_ = R_1*c;
    
%     set(coordinate,'xdata',o(1,:),'ydata',o(2,:),'zdata',o(3,:),...
%                    'udata',c_(1,:),'vdata',c_(2,:),'wdata',c_(3,:));
   
%     set(onplaneCoord,'xdata',o(1,1:2),'ydata',o(2,1:2),...
%                      'udata',c_(1,1:2),'vdata', c_(2,1:2));



% drawnow();
