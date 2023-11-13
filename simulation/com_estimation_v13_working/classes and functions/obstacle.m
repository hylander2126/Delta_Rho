% Author: Siamak Ghorbani Faal
% Date: 3/24/2017
% Copyright @ Siamak Ghorbani Faal. All rights reserved
classdef obstacle < matlab.mixin.SetGet
    properties
        Name = '2D Obstacle'
        Vertices = [];
        Faces = [];
        
        Lines = [];
        nLines = 0;
        
        bodyColor % Obstacle color
        handle % Graphical handles of the robot (body, vision and cs)
        
    end
    
    methods
        %__________________________________________________________________
        % Constructor('Property','value,...)
        function obj = obstacle(varargin)
            for i=1:2:nargin-1
                obj.(varargin{i}) = varargin{i+1};
            end

            if(isempty(obj.Faces))
                obj.Faces = 1:size(obj.Vertices,1);
            end
            
            obj.nLines = length(obj.Faces);
            I = [1:obj.nLines,1];
            a = zeros(obj.nLines,2);
            b = zeros(obj.nLines,1);
            for i=1:obj.nLines-1
                v1 = obj.Vertices(obj.Faces(I(i)),:)';
                v2 = obj.Vertices(obj.Faces(I(i+1)),:)';
                n = [0 -1;1 0]*(v2 - v1);
                a(i,:) = n'/norm(n);
                b(i,1) = -a(i,:)*v1;
            end
            obj.Lines = [a,b];
        end
        %__________________________________________________________________
        % display obstacle region
        function plot(obj,ax)
            if(nargin < 2)
                ax = gca;
                hold(ax,'on');
                axis(ax,'equal');
            end
            
            if(isempty(obj.handle))
                obj.handle = patch('Vertices',obj.Vertices,'Faces',obj.Faces,...
                'FaceColor',obj.bodyColor,'EdgeColor','k','Parent',ax);
            else
                set(obj.handle,'Vertices',obj.Vertices,'Faces',obj.Faces,...
                'FaceColor',obj.bodyColor);
            end
        end
        
        %__________________________________________________________________
        % Intersection between a circle(s0,r) with obstacle
        function [inter, x, cone] = sphereXo(obj,s0,r)
            % s0 = circle center; (column)
            % r = circle radius
            %--------------------------------------------------------------
            % inter = {0,1}; 0 if there is no intersection
            % x = intersecting points [ --x1-- ; ... ; --xN--] 
            % cone = two cone edges covering x [ --c1-- ; --c2-- ]
            %______________________________________________________________
            s0 = reshape(s0,[2,1]);
            cone = [];
            I = [1:obj.nLines,1];
            x = [];
            % Detecting intersecting points
            for i=1:obj.nLines
                if(norm(obj.Vertices(i,:)'-s0) <= r)
                    x(end+1,:) = obj.Vertices(i,:);
                end
                y = circleXline(s0,r,obj.Vertices(I(i),:)',obj.Vertices(I(i+1),:)');
                if(~isempty(y))
                    x(end+1:end+size(y,1),:) = y;
                end
            end
            inter = ~isempty(x);
            % Finding cone vertices
            if(nargout > 2 && inter)
                v = vectors(s0,x);
                cone = findCone(v);
            end
            
        end
        
    end
end

%__________________________________________________________________________
% Find points on the line segment (v1,v2) intersecting with the 
% circle(s0,r)
%   inputs:
%       circle: (s0,r); s0 (column)
%       line segment: (v1,v2); (column)
%   output:
%       intersecting points: x = [ --x1-- ; (--x2--) ];
function x = circleXline(s0,r,v1,v2)

n = (v2 - v1)/norm(v2 - v1);

t1 = n'*(v1-s0);
t2 = sqrt( t1^2 - norm(v1-s0)^2 + r^2 );

x = [];

if(isreal(t2))
    k(1) = -t1 + t2;
    k(2) = -t1 - t2;
    
    for i=1:2
        if(k(i) > 0)
            y = v1+k(i)*n;
            if(norm(y-v1)<= norm(v2-v1))
                x(end+1,:) = transpose(y);
            end
        end
    end
end

end

%__________________________________________________________________________
% Vectors from a point (origin) to a list of points (x)
%   inputs:
%       point: origin (row or column)
%       list of points: [ --x1--; ... ; --xN-- ];
%   output:
%       vectors: v = [ --v1-- ; ... ; --vN-- ];
function v = vectors(origin,x)

origin = reshape(origin,[1,2]);

v = zeros(size(x));

for i=1:size(x,1)
    v(i,:) = x(i,:) - origin;
    v(i,:) = v(i,:)/norm(v(i,:));
end
end

%__________________________________________________________________________
% find the two cone edges (cone) covering vectors (v)
%   inputs:
%       vectors: v = [ --v1-- ; ... ; --vN-- ];
%   output:
%       cone edges: cone = [ --c1-- ; --c2-- ];
function cone = findCone(v)

cone(1,:) = v(1,:);
cone(2,:) = v(1,:);

q = 1;

for i=1:size(v,1);
    for j=i+1:size(v,1);
        q_ = v(i,:)*v(j,:)';
        if(q_ < q)
            q = q_;
            cone(1,:) = v(i,:);
            cone(2,:) = v(j,:);
        end
    end
end
end


