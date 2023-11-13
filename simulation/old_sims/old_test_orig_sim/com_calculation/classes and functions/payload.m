% Author: Siamak Ghorbani Faal
% Date: 3/19/2017
% Copyright @ Siamak Ghorbani Faal. All rights reserved
classdef payload < matlab.mixin.SetGet
    properties
        Name = 'object';
        mass = 1; % mass of the object in kg
        inertia = 1; % object inertia in kgm^2
        uk = 0.1; % coefficient of kinetic friction
        p = [0;0;0]; %object position [x;y,q];
        dp = [0;0;0]; %object velocity [dx;dy,dq];
        bodyPoints = [];
        bodyRadius = 1;
        bodyColor = 'y';
        handle = [];
    end
    
    properties (Access = private)
        csRadius = [];
        g = 9.81;
    end
    
    methods
        %__________________________________________________________________
        % object constructor('Property','value,...)
        function [obj] = payload(varargin)
            % iter thru num args and set variables to user-def values
            for i=1:2:nargin-1 
                obj.(varargin{i}) = varargin{i+1};
            end
            % If no bodyPoints defined, create a circle with 20 points and
            % radius = bodyRadius
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

            % csRadius is the coordinate system radius
            % set the cs radius close to the supplied body radius
            obj.csRadius = 0.9*obj.bodyRadius;
            
            % Rz is rotation about the z axis defined below. P is the 
            % object position [x,y,q]
            R = Rz(obj.p(3));
            % create temp points that rotate bodyPoints by R
            p_ = ones(size(obj.bodyPoints,1),1)*obj.p(1:2)' + obj.bodyPoints*R';
            
            % now plot (patch) polygon with vertices p_ and connected faces
            obj.handle(1) = patch('Vertices',p_,...
                'Faces',1:size(obj.bodyPoints,1),...
                'FaceColor',obj.bodyColor,'EdgeColor','k','Parent',ax);
            obj.handle(2) = quiver([obj.p(1) obj.p(1)],[obj.p(2) obj.p(2)],...
                0.9*obj.csRadius*R(1,:),0.9*obj.csRadius*R(2,:),0,'k',...
                'LineWidth',1,'MaxHeadSize',.3,'Parent',ax);

            % Custom adding an arrow for velocity
%             obj.handle(3) = quiver(obj.p(1),obj.p(2),0,0,0,'Color','c', ...
%                 'LineWidth',1,'Parent',ax);
        end
        
        %__________________________________________________________________
        % return points on the surface of the payload;
        % basically run thru each object edge and find the intersectio b/w
        % this and ___ 
        function p = getBodyPoint(obj,N)
            % If one agent, create array with one entry (0) Why not do [0]?
            if(length(N) == 1)
                q = linspace(0,2*pi,N+1); q(end) = [];
            else
                q = N;
            end
            % array of length of # vertices, with 1 at the end
            I = [1:size(obj.bodyPoints,1),1];
            p = [];
            R = Rz(obj.p(3));
            % for every agent
            for i=1:length(q)
                % generate points for cos and sin of given agent
                n = [cos(q(i));sin(q(i))];
                % for every object vertex (point) till length-1
                for j=1:size(obj.bodyPoints,1)-1
                    v1 = obj.bodyPoints(I(j),:)';
                    v2 = obj.bodyPoints(I(j+1),:)';
                    % intersect point of ray (p,n) and line segment (v1,v2)
                    x = intersectionPoint([0;0],n,v1,v2);
                    if(~isempty(x))
                        p(end+1,:) = (obj.p(1:2) + R*x)';
                    end
                end
            end 
        end
            
        
        %__________________________________________________________________
        % move center of the object to p = [x, y, q] and set its velocity
        % to dp = [dx, dy, dz];
        function [varargout] = move(obj,p,dp)
            % reshape from 3x1 to 1x3 of the p provided
            p = reshape(p,[1,3]);
            obj.p = p';
            % if dp exists, set object dp to 3x1 of the dp provided
            if(exist('dp','var'))
                obj.dp = reshape(dp,[3,1]);
            end
            
            % if object handle is NOT empty:
            if(~isempty(obj.handle) || ~isempty(obj.handle))
                R = Rz(p(3));
                p_ = ones(size(obj.bodyPoints,1),1)*p(1:2) + obj.bodyPoints*R';
                
                % again, if object handle NOT empty, set new vertices to 
                % our calculated p_ from supplied p
                if(~isempty(obj.handle))
                    % set the 'vertices' label to our new p_
                    set(obj.handle(1),'vertices', p_)
                    % also set some other labels for quiver force vectors
                    set(obj.handle(2),'XData',[p(1) p(1)],'YData',[p(2) p(2)],...
                        'UData',0.9*obj.csRadius*R(1,:),...
                        'VData',0.9*obj.csRadius*R(2,:));
                   
                    % Custom changing the velocity quiver:
%                     set(obj.handle(3),'XData',p(1),'YData',p(2),'UData',...
%                         obj.dp(1),'VData',obj.dp(2));

                end
                if(nargout ~= 0)
                    varargout{1} = p_;
                end
                
            end
        end
        
        %__________________________________________________________________
        function dx = dynamics(obj,F,M)
            dx(1:3,1) = obj.dp;  % Current object velocity
            dx(4:5,1) = (F - obj.uk*obj.mass*obj.g*obj.dp(1:2))/obj.mass; % Originally multiply numerator by object velocity
            dx(6,1) = (M - obj.uk*obj.mass*obj.g*obj.dp(3))/obj.mass; % Originally multiply numerator by object velocity
        end

        
    end
end

%__________________________________________________________________________
% Rotation matrix about z axis
function R = Rz(q)
R = [cos(q) -sin(q); sin(q) cos(q)];
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



