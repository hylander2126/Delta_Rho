% Author: Siamak Ghorbani Faal
% Date: 3/19/2017
% Copyright @ Siamak Ghorbani Faal. All rights reserved
classdef robot < matlab.mixin.SetGet
    properties
        Name = 'Robot'
        delta % body radius
        gamma % vision range (radius)
        Kp % Proportional gain of the PD controller (Manipulation)
        Kd % Derivative gain of the PD controller (Manipulation)
        fmax % Maximum force of the robot
        
        p %robot position
        r %robots attachment point w.r.t object's center defined in {O}
        
        
        AttachedColor
        DetachedColor
        AvoidColor
        FmanColor
        FavoidColor
        
        handle % Graphical handles of the robot (body, vision and cs)
        
        attached = 1;                                                      % Conevrt to default = 0;
    end
    
    properties (Access = private)
       J = @(r) [1 0 1 0; 0 1 0 1; -r(2) r(1) r(2) -r(1)];
       
       bodyPoints % Points which define the perimeter of the robots
       visionPoints % Points which visually define the vision range
       csRadius % radius of the coordinate system)
    end
        
    methods
        %__________________________________________________________________
        % robot constructor('Property','value,...)
        function [obj] = robot(varargin)
            for i=1:2:nargin-1
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
             p_ = ones(size(obj.bodyPoints,1),1)*obj.p(1:2)' + obj.bodyPoints*R';
             pvision = ones(size(obj.visionPoints,1),1)*obj.p(1:2)' + obj.visionPoints*R';
                
            obj.handle(2) = patch('Vertices',pvision,...
                'Faces',1:size(obj.bodyPoints,1),...
                'FaceColor',obj.AttachedColor,'FaceAlpha',0.07,...
                'EdgeColor','k','LineStyle','--','Parent',ax);
            
            obj.handle(1) = patch('Vertices',p_,...
                'Faces',1:size(obj.bodyPoints,1),...
                'FaceColor',obj.AttachedColor,'EdgeColor','k','Parent',ax);

            obj.handle(3) = quiver([obj.p(1) obj.p(1)],[obj.p(2) obj.p(2)],...
                0.9*obj.csRadius*R(1,:),0.9*obj.csRadius*R(2,:),0,'k',...
                'LineWidth',1,'MaxHeadSize',.3,'Parent',ax);
            
            obj.handle(4) = quiver(obj.p(1),obj.p(2),0,0,0.1,...
                'Color', obj.FmanColor,'LineWidth',1,'MaxHeadSize',.3,'Parent',ax);
            
            
        end
        
        %__________________________________________________________________
        % set the value of r {O} based on the connection point xc {W} and
        % the object position Po {W}.
        function setr(obj,xc,Po)
            xc = reshape(xc,[2,1]);
            obj.r = Rz(Po(3))'*(xc - Po(1:2));
        end
        
        %__________________________________________________________________
        % move center of the robot to p = [x, y, q]
        function [varargout] = move(obj,p,pd,O,Q)
            p = reshape(p,[1,3]);
            obj.p = p';

            if(nargout > 0 || ~isempty(obj.handle))
                R = Rz(p(3));
                p_ = ones(size(obj.bodyPoints,1),1)*p(1:2) + obj.bodyPoints*R';
                pvision = ones(size(obj.visionPoints,1),1)*p(1:2) + obj.visionPoints*R';
                
                if(~isempty(obj.handle))
                    
                    [f,m,alpha] = agentForce(obj,pd,O,Q);
                    
                    set(obj.handle(1),'vertices', p_)
                    set(obj.handle(2),'vertices', pvision)
                    set(obj.handle(3),'XData',[p(1) p(1)],'YData',[p(2) p(2)],...
                        'UData',0.9*obj.csRadius*R(1,:),...
                        'VData',0.9*obj.csRadius*R(2,:));
                    
                    set(obj.handle(4),'XData',p(1),'YData',p(2),...
                        'UData',f(1),'VData',f(2));
                    
                    if(alpha == 1)
                        set(obj.handle(1),'FaceColor', obj.AvoidColor);
                        set(obj.handle(4),'Color', obj.FavoidColor);
                    else
                        set(obj.handle(1),'FaceColor', obj.AttachedColor);
                        set(obj.handle(4),'Color',obj.FmanColor);
                    end
                end
                
                if(nargout > 0)
                    varargout{1} = p_;
                end
            end
        end
        
        %__________________________________________________________________
        % The force exerted by the agent to the object
        function [f,m,alpha] = agentForce(obj,pd,O,Q)
            % pd = desired position of the object; (column)
            %  O = an object of the "payload" class
            %  Q = an array of all obstacles (objects of "obstacle" class)
            %--------------------------------------------------------------
            % f = [fx;fy] agent force in {W}; (column)
            % m = moment of f applied to the object in z direction
            % alpha
            %______________________________________________________________
            if(obj.attached)
                
                R = Rz(O.p(3));
                
                Jacobian = obj.J(obj.r);
                
                e = pd - O.p;
                de = -O.dp;
                
                phi = obj.Kp*e + obj.Kd*de;
                
                %__________________________________________________________
                %# inefficient section
                R3 = [R [0;0]; [0 0 1]];
                H = R3*[eye(2) zeros(2); zeros(1,4)]*pinv(Jacobian)*R3';
                Fman = H*phi;
                %__________________________________________________________
                
                fmanipulation = normSat(Fman(1:2),obj.fmax);
                [alpha,favoidance] = Avoidance(obj,O,Q);
                
                f = (1-alpha)*fmanipulation + alpha*favoidance;
                m = f(2)*R(1,:)*obj.r - f(1)*R(2,:)*obj.r;
                
            else
                f = [0;0];
                m = 0;
            end
            
        end
        %__________________________________________________________________
        % Agent velocity
        function [dp] = kinematics(obj,O)
            %  O = an object of the "payload" class
            %--------------------------------------------------------------
            % dp = agent velocity; (column)
            %______________________________________________________________

            if(obj.attached)    % if the agent is attached to the object;
                dp = O.dp + [O.dp(3)*Rz(O.p(3))*[-obj.r(2);obj.r(1)] ;0];
            else % if the agent is seperate from the object
                dp = [0;0;0];
                %dp = approachObject(obj,Q);                               % <-- Complete
            end
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
function [alpha,favoid] = Avoidance(A,O,Q)

dx = A.kinematics(O);
v = reshape(dx(1:2),[2 1])/norm(dx(1:2));

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

Vnorm = norm(V);
Unorm = norm(U);

if(Unorm > Vnorm)
    U = Vnorm/Unorm*U;
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



