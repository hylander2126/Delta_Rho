% Author: Steven M Hyland
% Date: 09/02/2022
% Copyright @ Steven M Hyland. All rights reserved
classdef payload < matlab.mixin.SetGet
    properties
        Name = 'object';
        mass = 0.5;     % mass of the object in kg
        inertia = 1;    % object inertia in kgm^2
        uk = 0.1;       % coefficient of kinetic friction
        p = [0;0;0];    % object position [x;y,q];
        dp = [0;0;0];   % object velocity [dx;dy,dq];
        bodyPoints = [];
        bodyRadius = 1;
        bodyColor = 'y';
        handle = [];
        g = 9.81;

        % ADDING MY OWN PROPERTIES 07/06/22
        friction = [0;0;0];   % obj acc due to fric - this is updated AFTER the agent and doesnt work right now 08/11/22
        a               % acceleration of the object
        com;            % com of the object - where dynamics are calculated

    end
    
    properties (Access = private)
        csRadius = [];
    end
    
    methods
        %__________________________________________________________________
        % object constructor('Property','value,...)
        function [obj] = payload(varargin)
            for i=1:2:nargin-1
                obj.(varargin{i}) = varargin{i+1};
            end

            % If no body points defined, make a circle with body points
            % defined as the body radius times (cos, sin) of the span from
            % 0 to 2pi (20 points in the span)
            if(isempty(obj.bodyPoints))
                qq = linspace(0,2*pi,20); qq(end) = [];
                obj.bodyPoints = obj.bodyRadius*[cos(qq)' sin(qq)'];
            end
        end
        %__________________________________________________________________
        % display payload
        function plot(obj,ax)
            if(nargin < 2)
                ax = gca;
                hold(ax,'on');
                axis(ax,'equal');
            end
            
            obj.csRadius = 0.9*obj.bodyRadius;            
            R = Rzd(obj.p(3)*180/pi);
            p_ = ones(size(obj.bodyPoints,1),1)*obj.p(1:2)' + obj.bodyPoints*R';

            % Object shape
            obj.handle(1) = patch('Vertices',p_,...
                'Faces',1:size(obj.bodyPoints,1),...
                'FaceColor',obj.bodyColor,'EdgeColor','k','Parent',ax);
            % Object frame arrows (x;y)
            obj.handle(2) = quiver([obj.p(1) obj.p(1)],[obj.p(2) obj.p(2)],...
                0.5*obj.csRadius*R(1,:),0.5*obj.csRadius*R(2,:),0.3,'k',...
                'LineWidth',1,'MaxHeadSize',.3,'Parent',ax);
    
            % Object velocity arrow
            obj.handle(3) = quiver(obj.p(1),obj.p(2),0,0,0.5,...
                'Color','b','LineWidth',1,'MaxHeadSize',.3,'Parent',ax);
        end
        %__________________________________________________________________
        % return points on the surface of the payload;
        function p = getBodyPoint(obj,N)
            if(length(N) == 1)
                q = linspace(0,2*pi,N+1);% q(end) = [];
            else
                q = N;
            end
            I = [1:size(obj.bodyPoints,1),1];
            p = [];
            R = Rzd(obj.p(3)*180/pi);
            for i=1:length(q)
                n = [cos(q(i));sin(q(i))];
                for j=1:size(obj.bodyPoints,1)-1
                    v1 = obj.bodyPoints(I(j),:)';
                    v2 = obj.bodyPoints(I(j+1),:)';
                    x = intersectionPoint([0;0],n,v1,v2);
                    if(~isempty(x))
                        p(end+1,:) = (obj.p(1:2) + R*x)';
                    end
                end
            end 
        end           
        
        %__________________________________________________________________
        % MOVE center of the object to p = [x, y, q] and set its velocity
        % to dp = [dx, dy, dz];
        function [varargout] = move(obj,p,dp,a,alpha,full_sim)
            p = reshape(p,[1,3]);
            obj.p = p';

            if(exist('dp','var'))
                obj.dp = reshape(dp,[3,1]);
            end
            
            if(exist('a','var'))
                obj.a = reshape(a,[3,1]);               
            end

            if(~isempty(obj.handle))
                R = Rzd(p(3)*180/pi);
                p_ = ones(size(obj.bodyPoints,1),1)*p(1:2) + obj.bodyPoints*R';

                if ~full_sim
                    % Object center
                    obj.handle(1) = patch('Vertices',p_,...
                        'Faces',1:size(obj.bodyPoints,1),...
                        'FaceColor',obj.bodyColor,'EdgeColor','k','FaceAlpha',alpha);
                    % Object frame
                    obj.handle(2) = quiver([obj.p(1) obj.p(1)],[obj.p(2) obj.p(2)],...
                        0.5*obj.csRadius*R(1,:),0.5*obj.csRadius*R(2,:),0.3,'k',...
                        'LineWidth',1,'MaxHeadSize',.3);
                else
                    % Object center
                    set(obj.handle(1),'vertices', p_)
                    % Object frame
%                     Update object axes position/direction
                    set(obj.handle(2),'XData',[p(1) p(1)],'YData',[p(2) p(2)],...
                        'UData',0.9*obj.csRadius*R(1,:),...
                        'VData',0.9*obj.csRadius*R(2,:));
                end
                
                % Update object velocity arrow
%                 set(obj.handle(3),'XData',p(1),'YData',p(2),'UData',obj.dp(1),...
%                     'VData',obj.dp(2));

                if(nargout ~= 0)
                    varargout{1} = p_;
                end                
            end
        end
        %__________________________________________________________________
        % Payload DYNAMICS
        function dx = dynamics(obj,Agent)
            %% Get relative vel/acc AT COM
            R = Rzd(obj.p(3)*180/pi);
            r = [(R * Agent.r); 0]; % r from object center to agent

            A_vel = Agent.dp;
            omega = cross(r, A_vel)/norm(r)^2; % Omega calculated with r from obj to agent
            vel = A_vel + cross(omega, -r); % velocity calculated with r from agent to obj

            com_vel = [vel(1:2); omega(3)]; % Relative velocity of the CoM
            
            A_acc = Agent.a;
            alpha = cross(r, A_acc)/norm(r)^2;
            acc = A_acc + cross(alpha, -r) - omega(3)^2*-r;
            
            com_acc = [acc(1:2); alpha(3)]; % Relative acc of the CoM

            %% Get relative vel/acc AT CENTROID
            new_r = [R*obj.com;0];
            newVel = com_vel + cross(omega, -new_r);
            dx(1:3,1) = [newVel(1:2); omega(3)];

            newAcc = com_acc + cross(alpha, -new_r) - omega(3)^2*-new_r;
            dx(4:6,1) = [newAcc(1:2); alpha(3)];
            
%             Friction = frictionForce(obj,dx(1:3,1))
%             dx(4:5,1) = acc(1:2) + Friction(1:2);
%             dx(6,1) = alpha(3) + Friction(3);
        end
        %__________________________________________________________________
        % Object FRICTION FORCE
        function [fric] = frictionForce(obj,vel)
            % Object dp is w.r.t the object's frame
%             dpo = obj.dp/norm(obj.dp); % Payload velocity property
%             dpo = vel/norm(vel) % Supplied payload velocity
%             if isnan(dpo)
%                 disp('dpo is nan')
%                 dpo = [0;0;0];
%             end

            dpo = obj.dp./norm(obj.dp);
            fric(1:2,1) = - (obj.uk*obj.mass*obj.g*dpo(1:2))/obj.mass;
            fric(3,1) = - (obj.uk*obj.mass*obj.g*dpo(3))/obj.mass;

            obj.friction = fric;
        end
    end
end

%__________________________________________________________________________
% Rotation matrix about z axis
function R = Rz(q)
    R = [cos(q) -sin(q); sin(q) cos(q)];
end
% Rotation matrix about z axis IN DEGREES
function R = Rzd(q)
    R = [cosd(q) -sind(q); sind(q) cosd(q)];
end

%__________________________________________________________________________
% intersection of a ray (p,n) and line segment (v1,v2)
function x = intersectionPoint(p,n,v1,v2)
    dv = v2 - v1;
    a = [0 -1;1 0]*dv;
    b = -a'*v1;
    
    k = (a'*p - b)/(a'*n);
    y = p + k*n;
    
    x = [];    
    alpha = (dv'*(y-v1))/(dv'*dv);
    
    if( k >= 0 && 0 <= alpha && alpha <= 1)
        x = y;
    end
end



