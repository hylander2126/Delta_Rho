% Author: Steven Hyland
% Date: 04/22/2025
classdef body < matlab.mixin.SetGet
    properties
        Name        = 'object';
        mass        = 1;        % mass of the object in kg
        inertia     = 1;        % object inertia in kgm^2
        p           = [0;0;0];  % object position [q;x;y];
        v           = [0;0;0];  % object twist [omega;vx;vy];
        a           = [0;0;0];  % object 
        bodyColor   = 'y';      % color of the rigid body
        handle      = [];
        width       = 2;        % box width
        height      = 4;        % box height
        Pf          = [0;0;0];  % point on object of external force application
        Fp          = [0;0;0];  % magnitude of applied force
        verts       = [];       % point cloud of object vertices
        com         = [0;0;0];  % com wrt origin
    end
    
    properties (Access = private)
        g           = 9.81;     % gravity
        mu          = 0.5;      % coefficient of kinetic friction
        faces       = [];       % faces of the object
        origin      = [0;0;0];  % NOT SURE YET HOW THIS AFFECTS STUFF
        T           = eye(4);   % Transformation matrix for tracking plot
    end
    
    methods
        %% Class constructor('Property','value,...)
        function [obj] = body(varargin)
            % iter thru num args and set variables to user-def values
            for i=1:2:nargin-1 
                obj.(varargin{i}) = varargin{i+1};
            end
            % % If no bodyPoints defined, create a circle with 20 points and
            % % radius = bodyRadius
            % if(isempty(obj.bodyPoints))
            %     qq = linspace(0,2*pi,20); qq(end) = [];
            %     obj.bodyPoints = obj.bodyRadius*[cos(qq)' sin(qq)'];
            % end
            % obj.prev_theta = obj.p(3);


            % Parameters for the prism
            origin = reshape(obj.origin, [1 3]);
            dx = obj.width;
            dy = obj.width;
            dz = obj.height;

            % Define the 8 vertices of the prism
            verts = [origin;                      % 1
                    origin + [dx, 0, 0];          % 2
                    origin + [dx, dy, 0];         % 3
                    origin + [0, dy, 0];          % 4
                    origin + [0, 0, dz];          % 5
                    origin + [dx, 0, dz];         % 6
                    origin + [dx, dy, dz];        % 7
                    origin + [0, dy, dz];         % 8
            ];
            
            % Define the six faces by listing the four corner‑indices of each
            faces = [ ...
                1 2 3 4;   % bottom
                5 6 7 8;   % top
                1 2 6 5;   % front
                2 3 7 6;   % right
                3 4 8 7;   % back
                4 1 5 8;   % left
            ];
            
            % Store the vertices as the 'point cloud' representation that we would have in real life
            obj.verts = verts;
            obj.faces = faces; % and i guess faces too...
        end


        %% Display payload
        function plot(obj,ax,viewpoint)
            if nargin < 2, ax = gca;        end
            if nargin < 3, viewpoint = 3;   end
            % if nargin < 4, T = eye(4);      end
                % hold(ax,'on');
                % axis(ax,'equal');
            
            % % Build homogeneous coords
            % V_h   = [obj.verts, ones(size(obj.verts,1),1)];
            % Pf_h  = [obj.Pf; 1];
            % com_h = [obj.com; 1];
            % 
            % % Transform
            % V_t   = (T * V_h')';
            % Pf_t  = [eye(3) [0;0;0]] * (T * Pf_h); % Make it 3 elements
            % com_t = T * com_h;
            % 
            % % Extract and Plot the object
            % verts_plot = V_t(:, 1:3);

            patch('Vertices', obj.verts, ...
                  'Faces', obj.faces, ...
                  'FaceColor', obj.bodyColor, ...
                  'FaceAlpha', 0.4, ...
                  'EdgeColor', 'k', ...
                  'LineWidth', 1.2);
              
            % Enhance the axes
            axis equal;
            grid on;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Object Visualization');
            view(viewpoint);
            xlim([obj.origin(1)-2, obj.width+2]); ylim([obj.origin(2)-2, obj.width+2]); zlim([-0.1, obj.height+2])
        
            % Draw controlled force arrow
            hold on;
            quiv3(obj.Pf(1),obj.Pf(2),obj.Pf(3), obj.Fp(1),obj.Fp(2),obj.Fp(3), 'r')

            % With contact normals as shown in magenta
            cll = 0.5; % contact normal arrow length
            for i=1:4 % for each vertex (4 for now)
                for j=1:4 % for each side of friction polyhedral approximation
                    switch j
                        case 1
                            fn_hat = [cll*obj.mu ; 0; cll];
                        case 2
                            fn_hat = [0 ; cll*obj.mu; cll];
                        case 3
                            fn_hat = [0 ; -cll*obj.mu; cll];
                        case 4
                            fn_hat = [-cll*obj.mu ; 0; cll];
                    end
                    fn_hat_t = obj.T(1:3,1:3)*fn_hat;
                    quiv3(obj.verts(i,1),obj.verts(i,2),obj.verts(i,3), fn_hat_t(1),fn_hat_t(2),fn_hat_t(3), 'm')
                end
            end

            % Draw the CoM
            scatter3(obj.com(1),obj.com(2),obj.com(3), 'red', 'filled', 'MarkerEdgeColor','w', 'LineWidth',1)
        end
            
        
        %__________________________________________________________________
        function [varargout] = move(obj, T)
            % Build homogeneous coords
            V_h   = [obj.verts, ones(size(obj.verts,1),1)];
            Pf_h  = [obj.Pf; 1];
            com_h = [obj.com; 1];

            % Transform
            V_t   = (T * V_h')';
            Pf_t  = T * Pf_h;
            com_t = T * com_h;

            % Extract the elements
            verts_plot = V_t(:, 1:3);
            Pf_plot = Pf_t(1:3); % Make it 3 elements
            com_plot = com_t(1:3);

            obj.verts = verts_plot;
            obj.Pf = Pf_plot;
            obj.com = com_plot;
            obj.T = T;

        end
        

        %__________________________________________________________________
        function dx = dynamics(obj, F, M)
            obj.prev_theta = obj.p(3); % Note previous payload rotation
            
            R = Rz(obj.p(3));   % Rotation is consistant thru a rigid body
            omega = [0;0;obj.dp(3)];  % Angular velocity is consistant too

            %% Incoming force - Contact model uses normal/tangent
            com_to_cent = R*-obj.com;                       % Unit: CoM TO centroid
            n_hat = R*((obj.r-obj.com)/norm(obj.r-obj.com));% Unit: r TO CoM

            F_new = dot(F,n_hat)*n_hat;                       % Force norm towards CoM
%             F_new = F;

            M = cross([n_hat;0],[F;0]);            

            %% NEXT get agent acceleration ACTING ON CoM
            a_com = (F_new - obj.uk*obj.mass*obj.g*obj.dp(1:2))/obj.mass;
            alpha = (M(3) - obj.uk*0.1*obj.mass*obj.g*obj.dp(3))/obj.mass; % Angular accel THOUGHOUT PAYLOAD

            %% FINALLY, get relative force acting AT CENTROID:
            a_centr_n = cross(omega,cross(omega,[com_to_cent;0]));  a_centr_n(end) = [];
            a_centr_t = cross([0;0;alpha],[com_to_cent;0]);         a_centr_t(end) = [];
            

            dx(1:3,1) = obj.dp;                         % Current object velocity
            dx(4:5,1) = a_com + a_centr_n + a_centr_t;  % Relative acc = a_com + a_com/centr_norm + a_com/centr_tan
            dx(6,1) = alpha;
        end
    end
end

%__________________________________________________________________________
% Rotation matrix about z axis
function R = Rz(q)
    R = [cos(q) -sin(q); sin(q) cos(q)];
end

%__________________________________________________________________________
% Rotation matrix about z axis in DEGREES
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

%__________________________________________________________________________
% Plot 3D quiver (makes the linewidth and color and arrow size the same)
function quiv3(x,y,z,u,v,w, color)
            quiver3(x, y, z, u, v, w, ...
                'LineWidth', 2, 'Color',color, 'AutoScale','off', 'MaxHeadSize',2)
end



