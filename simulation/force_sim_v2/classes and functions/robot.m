% Author: Siamak Ghorbani Faal
% Date: 3/19/2017
% Copyright @ Siamak Ghorbani Faal. All rights reserved
classdef robot < matlab.mixin.SetGet
    properties
        Name = 'Robot';
        ID              % Agent ID
        delta           % body radius
        gamma           % vision range (radius)
        Kp              % Proportional gain of the PD controller (Manipulation)
        Kd              % Derivative gain of the PD controller (Manipulation)
        fmax            % Maximum force of the robot
        
        p               % robot position
        r               % robots attachment point w.r.t object's center defined in {O}

        % I ADDED THESE PROPERTIES
        f = [0 0];      % robot force
        m = 0;          % robot moment
        p_attach        % robots INTENDED attachment point
        com_mode = 0;   % CoM search mode initialized as false
        prev_meas % Previously measured payload rotation
        desired_direc   % Unit vector of INTENDED travel direction
        max_theta       % Threshold for max rotation measured
        min_theta       % Threshold to STOP pushing due to minimal rotation recorded
        measure_t=0     % Time-keeper for when rotation is below threshold - in conjunction with min_theta

        f_read=0        % Force reading from force sensor

        negBound        % Clockwise-most bound of obj wrt agent pos
        posBound        % C.Clockwise-most bound of obj wrt agent pos
        
        prev_direc      % Previous desired direction vector
        stop = 0
        noise_pregen
        counter
        
        AttachedColor
        DetachedColor
        AvoidColor
        FmanColor
        FavoidColor
        
        handle          % Graphical handles of the robot (body, vision and cs)
        
        attached = 0;   % Convert to default = 0;
    end
    
    properties (Access = private)
       J = @(r) [1 0 1 0; 0 1 0 1; -r(2) r(1) r(2) -r(1)]; % LOCAL Jacobian (not robot jacobian)
       
       bodyPoints % Points which define the perimeter of the robots
       visionPoints % Points which visually define the vision range
       csRadius % radius of the coordinate system
    end
        
    methods
        %__________________________________________________________________
        % robot constructor('Property',value,...)
        function [obj] = robot(varargin)
            for i=1:2:nargin-1
%                 disp(varargin{i})
                obj.(varargin{i}) = varargin{i+1};
            end
        end
        

        %__________________________________________________________________
        % display robot
        function plot(obj,ax)
            if(nargin < 2)
                ax = gca;
                hold(ax,'on');
                axis(ax,'equal');
            end 
            
            obj.csRadius = 0.9*obj.delta;
            
            qq = linspace(0,2*pi,20); qq(end) = [];
            obj.bodyPoints = obj.delta*[cos(qq)' sin(qq)'];
            obj.visionPoints = obj.gamma*[cos(qq)' sin(qq)'];
            
            R = Rz(obj.p(3));            
            p_ = ones(size(obj.bodyPoints,1),1)*obj.p(1:2)' + obj.bodyPoints*R'; % Body Points of Agent
            pvision = ones(size(obj.visionPoints,1),1)*obj.p(1:2)' + obj.visionPoints*R';
            
            % Set/update the agent body vertices to the agent starting point
            obj.handle(1) = patch('Vertices',p_,'Faces',1:size(obj.bodyPoints,1),'FaceColor',obj.AttachedColor, ...
                'EdgeColor','k','Parent',ax);

            % Set (plot) the agent vision vertices to the center of the agent
            obj.handle(2) = patch('Vertices',pvision,'Faces',1:size(obj.bodyPoints,1),'FaceColor',obj.AttachedColor,...
                'FaceAlpha',0.07,'EdgeColor','k','LineStyle','--','Parent',ax);

            % Display vector corresponding to F towards the desired point
%             obj.handle(3) = quiver([obj.p(1) obj.p(1)],[obj.p(2) obj.p(2)],0.9*obj.csRadius*R(1,:),...
%                 0.9*obj.csRadius*R(2,:),0,'r','LineWidth',1,'MaxHeadSize',.3,'Parent',ax);
            
            % Set the force arrow location and size
            obj.handle(4) = quiver(obj.p(1),obj.p(2),0,0,0,'Color', obj.FmanColor,'LineWidth',1,'MaxHeadSize',.3,...
                'Parent',ax);
           
            % Set the bounds arrows locations and sizes
            obj.handle(5) = quiver(obj.p(1),obj.p(2),0,0,1,'Color','b','LineWidth',1,'MaxHeadSize',1,...
                'Parent',ax,'DisplayName','Bounds'); % negBound
            obj.handle(6) = quiver(obj.p(1),obj.p(2),0,0,1,'Color','b','LineWidth',1,'MaxHeadSize',1,...
                'Parent',ax); % posBound

            % Force reading arrow
%             obj.handle(7) = quiver(obj.p(1),obj.p(2),0,0,1,'Color','r','LineWidth',1,'MaxHeadSize',1,...
%                 'Parent',ax,'DisplayName','Force Reading');

            % Payload velocity a.k.a. force reading v2
            obj.handle(7) = quiver(obj.p(1),obj.p(2),0,0,5,'Color','r','LineWidth',1,'MaxHeadSize',1,...
                'Parent',ax,'DisplayName','Payload Velocity');
        end
        

        %__________________________________________________________________
        % set the value of r {O} based on the connection point xc {W} and the object position Po {W}.
        function setr(obj,xc,Po)
            xc = reshape(xc,[2,1]);
            obj.r = Rz(Po(3))'*(xc - Po(1:2));
        end
        

        %__________________________________________________________________
        % move center of the robot to p = [x, y, q]
        function [varargout] = move(obj,p,O,desired_direc,O_dp)
            p = reshape(p,[1,3]);
            obj.p = p';

            if(nargout > 0 || ~isempty(obj.handle))
                R = Rz(p(3));

                % p_ and pvision should just be points of a circle
                p_ = ones(size(obj.bodyPoints,1),1)*p(1:2) + obj.bodyPoints*R';
                pvision = ones(size(obj.visionPoints,1),1)*p(1:2) + obj.visionPoints*R';

                if(~isempty(obj.handle))
%                     [f,m,alpha] = agentForce(obj,pd,O,Q,1);
%                     [f,m] = com(obj,O);

                    set(obj.handle(1),'vertices', p_)
                    set(obj.handle(2),'vertices', pvision)

%                     set(obj.handle(3),'XData',[p(1) p(1)],'YData',[p(2) p(2)],'UData',0.9*obj.csRadius*R(1,:),...
%                         'VData',0.9*obj.csRadius*R(2,:));
                    
%                     set(obj.handle(4),'XData',p(1),'YData',p(2),'UData',f(1),'VData',f(2));
                    set(obj.handle(4),'XData',p(1),'YData',p(2),'UData',desired_direc(1),'VData',desired_direc(2));
                    set(obj.handle(7),'XData',p(1),'YData',p(2),'UData',O_dp(1),'VData',O_dp(2));
                end
                
                if(nargout > 0)
                    varargout{1} = p_;
                end
            end
        end
        

        %__________________________________________________________________
        % The force exerted by the agent to the object
        function [f,m] = agentForce(obj,t,O)
            % f = [fx;fy] agent force in {W}; (column)
            % m = moment of f applied to the object in z direction
            %______________________________________________________________
            if t < 1
                direction = [0;1];
            else
                direction = com_search(obj,t,O);
                obj.prev_direc = direction;
            end
            R = Rz(O.p(3));
            f = obj.fmax * direction;
%             m = f(2)*R(1,:)*(obj.r - O.com) - f(1)*R(2,:)*(obj.r - O.com);
            m = (f(2)*R(1,:)*obj.r - f(1)*R(2,:)*obj.r);

            obj.f = f;
        end


        %__________________________________________________________________
        % Agent velocity
        function [dp] = kinematics(obj,t,O)
            % O = an object of the "payload" class;      dp = agent velocity; (3x1 column)
            delta_dp = [O.dp(3)*Rz(O.p(3))*[-obj.r(2);obj.r(1)] ;0];
            dp = O.dp + delta_dp;
        end


        % _________________________________________________________________
        % Routine to compare force vectors and decide new heading
        function new_direction = com_search(obj,t,O)
            % Set refresh rate of force sensor

            force_reading = obj.read_force(O);
            if norm(force_reading) ~= 0
                force_reading = -(force_reading/norm(force_reading));
            end

            e = -atan2d(force_reading(1)*obj.prev_direc(2)-force_reading(2)*obj.prev_direc(1), ...
                force_reading(1)*obj.prev_direc(1)+force_reading(2)*obj.prev_direc(2));            
            de = -rad2deg(O.dp(3));

            phi = 1.1*e + 1*de;
            new_direction = Rz(deg2rad(phi)) * obj.prev_direc;

%             new_direction = [0;1];
        end


        % _________________________________________________________________
        % Measure reactive force from payload
        function force_reading = read_force(obj,O)
            if norm(O.dp) ~= 0
%                 % Need to calculate normal component, tangential component, and account for friction.
%                 % Normal:
%                 % Lever arm is vector from com to robot
%                 lever_arm = Rz(O.p(3))*(-obj.r/norm(obj.r));
%                 normal_force = dot(F, lever_arm);
%                 normal_force = normal_force * lever_arm;
%                 % Tangent:
%                 tan_direc = Rz(pi/2)*lever_arm;
%                 tan_force = alpha * norm(obj.r);
%                 tan_force = tan_force * tan_direc;
%                 % Sum:
%                 force_read = -(normal_force + tan_force); % Not sure why need to negate... wip

                
                % NEW METHOD: just use the friction force aka velocity
                force_reading_raw = -obj.f-(O.uk*O.mass*9.81*O.dp(1:2))/O.mass;
                temp = randn(1);            % Normal distribution variance = 1

%                 if obj.counter == length(obj.noise_pregen)
%                     obj.counter = 1;
%                 end
                
%                 temp = obj.noise_pregen(obj.counter,1);
%                 obj.counter = obj.counter + 1;
                
                sensor_noise = Rzd(temp);
                force_reading = sensor_noise*force_reading_raw;                
            else
                force_reading = [0;0];
            end
%             disp('Sensor Reading:')
%             disp(force_read')
        end
    end
end


%__________________________________________________________________________
% Avoidance
%   inputs:
%       A: agent of class "robot"
%       O: object of class "payload"
%       Q: an array of objects of class "obstacle"
%   output:
%       alpha: {0,1} 0 if there is no obstacle
%       favoid: avoidance force
function [alpha,favoid] = Avoidance(A,O,Q,varargin)
    if nargin > 3
        dx = varargin{1};
    else
        dx = A.kinematics(O,Q);
    end
%     v = reshape(dx(1:2),[2 1])/norm(dx(1:2));
    b = [0;0];
    N = 0;
    
    for j=1:length(Q)
        
        [alpha, ~, cone] = Q(j).sphereXo(A.p(1:2),A.gamma);
        
        if(alpha)
            l1 = cone(1,:)';
            l2 = cone(2,:)';
            
            % find if v is in the cone
            n = (l1 + l2)/norm(l1+l2);
            
    %         if( v'*n >= l1'*n )
    %             N = N + 1;
    %             B = v - 2*n*(v'*n);
    %             b = b + B/norm(B);
    %         end
    
            N = N + 1;
            b = b - n;
        end
    end
    
    if(N~=0)
        b = b/N;    %fmax
    end

    favoid = A.fmax*b;
end

%__________________________________________________________________________
% Saturates vector U based on the norm of (V)
%   inputs:
%       v:  a row or column vector
%       limits: [limit_low, limit_high]
%   output:
%       vs: saturated vector
function U = normSat(U,V)
    Vnorm = norm(V); % Magnitude of vector V
    Unorm = norm(U); % Magnitude of vector U
    if(Unorm > Vnorm)
        U = Vnorm/Unorm*U; % Ratio of magnitudes times the original vector U 
    end
end

%__________________________________________________________________________
% Saturates vector (v) based on the (limits)
%   inputs:
%       v:  a row or column vector
%       limits: [limit_low, limit_high]
%   output:
%       vs: saturated vector
function vs = saturate(v,limits)
    vs = zeros(size(v));
    for i=1:length(v)
        vs(i) = min(max(v(i),limits(1)),limits(2));
    end
end

%__________________________________________________________________________
% Rotation matrix about z axis
function R = Rz(q)
R = [cos(q) -sin(q); sin(q) cos(q)];
end

%__________________________________________________________________________
% Rotation matrix about z axis
function R = Rzd(q)
R = [cosd(q) -sind(q); sind(q) cosd(q)];
end

%% MY FUNCTIONS
function theta = midAngle(v1,v2)    
    % Find costheta of vectors
    cosTheta = dot(v1,v2)/(norm(v1)*norm(v2));
    % Inv cos of that value in degrees
    theta = real(acos(cosTheta))/2;
end

