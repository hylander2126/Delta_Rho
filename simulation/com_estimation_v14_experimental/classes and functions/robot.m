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

        negBound        % Clockwise-most bound of obj wrt agent pos
        posBound        % C.Clockwise-most bound of obj wrt agent pos
        
        stop = 0;
        
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
        end
        

        %__________________________________________________________________
        % set the value of r {O} based on the connection point xc {W} and the object position Po {W}.
        function setr(obj,xc,Po)
            xc = reshape(xc,[2,1]);
            obj.r = Rz(Po(3))'*(xc - Po(1:2));
        end
        

        %__________________________________________________________________
        % move center of the robot to p = [x, y, q]
        function [varargout] = move(obj,p,O,negBound,posBound,desired_direc)
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

                    % Update object bounds arrows (don't have list of bounds for simulation section rn. 
                    % Need to 'move' while giving live bound information

                    set(obj.handle(5),'XData',p(1),'YData',p(2),'UData',negBound(1),'VData',negBound(2));
                    set(obj.handle(6),'XData',p(1),'YData',p(2),'UData',posBound(1),'VData',posBound(2));
%                     set(obj.handle(5),'XData',p(1),'YData',p(2),'UData',bounds(1),'VData',bounds(2));
%                     set(obj.handle(6),'XData',p(1),'YData',p(2),'UData',bounds(3),'VData',bounds(4));


%                     if(alpha == 1)
%                         set(obj.handle(1),'FaceColor', obj.AvoidColor);
%                         set(obj.handle(4),'Color', obj.FavoidColor);
%                     else
                    set(obj.handle(1),'FaceColor', obj.AttachedColor);
                    set(obj.handle(4),'Color','m'); % Used to be 'Color',obj.FmanColor);
%                     end
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
            
            R = Rz(O.p(3));
            direction = comSearch(obj,t,O);
            
            f = obj.fmax * direction;
%             m = f(2)*R(1,:)*(obj.r - O.com) - f(1)*R(2,:)*(obj.r - O.com);
            m = (f(2)*R(1,:)*obj.r - f(1)*R(2,:)*obj.r);

%             f = obj.fmax*[0; -1];
        end


        %__________________________________________________________________
        % Agent velocity
        function [dp] = kinematics(obj,t,O)
            % O = an object of the "payload" class;      dp = agent velocity; (3x1 column)
            delta_dp = [O.dp(3)*Rz(O.p(3))*[-obj.r(2);obj.r(1)] ;0];
            dp = O.dp + delta_dp;
        end


        % _________________________________________________________________
        % Routine to measure rotation and decide new heading
        function new_direction = comSearch(obj,t,O)
   
            % Update bound rotation every time step
            updateBounds(obj,O)

            % Record payload rotation from previous measurement
            theta_meas = O.p(3) - obj.prev_meas;

            % IF stop condition active
            if obj.stop
                new_direction = [0;0]; 
                return

            % IF measured rotation BELOW threshold
            elseif abs(theta_meas) < deg2rad(obj.max_theta)
%                 new_direction = Rz(O.p(3) - O.prev_theta) * obj.desired_direc; % Rotate desired heading by change since last measurement
                new_direction = obj.desired_direc;
                obj.desired_direc = new_direction;

            % IF GREATER than threshold
            else
                obj.prev_meas = O.p(3);

                if sign(theta_meas) < 0
                    obj.posBound = Rz(theta_meas) * obj.desired_direc; % Update positive (c.clock-most bound)
                else
                    obj.negBound = Rz(theta_meas) * obj.desired_direc; % Update negative (clock-most bound)
                end
                
                % Get new direction - bisect bounds w/ unit vector, then multiply by desired acceleration
                pseudo_bisect = obj.negBound + obj.posBound;
                new_direction = pseudo_bisect./norm(pseudo_bisect);

                obj.desired_direc = new_direction;
                obj.measure_t = t;
            end
            
            % Checking for min rotation threshold. Activate global stop condition
            if t-obj.measure_t > 3
                obj.measure_t = t;
                if abs(theta_meas) < deg2rad(obj.min_theta)
                    fprintf("STOPURU at time %.2f with measured theta: %.2f\n\n", round(t,2), rad2deg(theta_meas))
                    obj.stop = 1; %%%%%% STOP CONDITION
                    % Get final push vector, bisecting final bounds
                    pseudo_bisect = obj.negBound + obj.posBound;
                    obj.desired_direc = pseudo_bisect./norm(pseudo_bisect);
                    obj.measure_t = t; % record final run time
                end
            end
        end


        %__________________________________________________________________
        % Update bounds every time step for graphics
        function updateBounds(obj,O)
            obj.negBound = Rz(O.p(3) - O.prev_theta) * obj.negBound;
            obj.posBound = Rz(O.p(3) - O.prev_theta) * obj.posBound;
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

