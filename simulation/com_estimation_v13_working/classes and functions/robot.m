% Author: Steven M Hyland
% Date: 09/02/2022
% Copyright @ Steven M Hyland. All rights reserved
classdef robot < matlab.mixin.SetGet
    properties
        Name = 'Robot'
        delta          % body radius
        fmax           % Maximum force of the robot        
        p              % robot position
        r              % robots attachment point w.r.t object's center defined in {O}

        % ADDING ADDITIONAL PROPERTIES
        dp             % Current robot velocity
        a              % Robot current acceleration
        a_prev         % Previous robot acceleration for deceleration
        a_desired      % Desired acceleration

        method         % Whether to use optical method or force sensing
        deltaT = 0     % Counter for Delta-T calculation

        negBound       % Counter-clockwise-most bound of obj wrt agent pos
        posBound       % Clockwise-most bound of obj wrt agent pos

        prevTheta      % Record previous rotation amount for bounds change
        measuredTheta = 0 % Recorded rotation during measurement step

        measuredCoM    % Final CoM line prediction wrt space frame

        mode = 'accel' % Mode to check for accel/decel/measure robot modes     
        
        arrowSize      % Arrow size only for bounds arrows
        handle          % Graphical handles of the robot (body, vision and cs)     
        attached = 1;                                                      % Convert to default = 0;
    end
    
    properties (Access = private)
        J = @(r) [1 0 1 0; 0 1 0 1; -r(2) r(1) r(2) -r(1)];
       
        bodyPoints % Points which define the perimeter of the robots
        csRadius % radius of the coordinate system
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
            
            R = Rzd(obj.p(3)*180/pi);
            p_ = ones(size(obj.bodyPoints,1),1)*obj.p(1:2)' + obj.bodyPoints*R';                
            
            % Agent body
            obj.handle(1) = patch('Vertices',p_,'Faces',...
                1:size(obj.bodyPoints,1),'FaceColor',[0.6549, 0.8353, 0.5412],...
                'EdgeColor','k','Parent',ax);    
            % Object bounds arrows
            obj.handle(7) = quiver(obj.p(1),obj.p(2),0,0,1,'Color','b',...
                'LineWidth',1,'MaxHeadSize',obj.arrowSize,'Parent',ax,'DisplayName','Bounds');
            obj.handle(8) = quiver(obj.p(1),obj.p(2),0,0,1,'Color','b',...
                'LineWidth',1,'MaxHeadSize',obj.arrowSize,'Parent',ax);
            
            % Agent velocity arrow (keep showing regardless of showSteps)
            obj.handle(6) = quiver(obj.p(1),obj.p(2),0,0,0.5,...
                'Color', 'r','LineWidth',1,'MaxHeadSize',.3,'Parent',ax);
            % Applied force arrow
%             obj.handle(2) = quiver(obj.p(1),obj.p(2),0,0,0.5,...
%                 'Color', 'k','LineWidth',1,'MaxHeadSize',.3,'Parent',ax);

            if contains(obj.method, 'force')
                % Normal force reading arrow
                obj.handle(3) = quiver(obj.p(1),obj.p(2),0,0,0.1,...
                    'Color', 'm','LineWidth',1,'MaxHeadSize',.3,'Parent',ax);
                % Tangent force reading arrow
                obj.handle(4) = quiver(obj.p(1),obj.p(2),0,0,0.5,...
                    'Color', 'g','LineWidth',1,'MaxHeadSize',.3,'Parent',ax);
                % Friction force arrow
                obj.handle(5) = quiver(obj.p(1),obj.p(2),0,0,0.5,...
                    'Color', '','LineWidth',1,'MaxHeadSize',.3,'Parent',ax);
            end
        end
        
        %__________________________________________________________________
        % set the value of r {O} based on the connection point xc {W} and
        % the object position Po {W}. Takes into account desired CoM posit.
        function setr(obj,xc,Po,com)
            xc = reshape(xc,[2,1]);
            R = Rzd(Po(3)*180/pi); % Get current object rotation theta
            r_com = R*com; % Rotate com wrt current object theta
            obj.r = R'*(xc - (Po(1:2) + r_com));
        end
        
        %__________________________________________________________________
        % MOVE center of the robot to p=[x,y,q] dp=[dx,dy,dq] a=[ax,ay,aq]
        function [varargout] = move(obj,p,dp,a,negBound,posBound,O,alpha,full_sim)
            p = reshape(p,[1,3]);
            obj.p = p';
            
            if(exist('dp','var'))
                obj.dp = reshape(dp,[3,1]);               
            end
            
            if(exist('a','var'))
                obj.a = reshape(a,[3,1]);               
            end

            if(nargout > 0 || ~isempty(obj.handle))
                R = Rzd(p(3)*180/pi);
                p_ = ones(size(obj.bodyPoints,1),1)*p(1:2) + obj.bodyPoints*R';

                if ~full_sim
                    % Agent body
                    obj.handle(1) = patch('Vertices',p_,'Faces',...
                        1:size(obj.bodyPoints,1),'FaceColor',[0.6549, 0.8353, 0.5412],...
                        'EdgeColor','k','FaceAlpha',alpha);
                    % Agent bounds
                    obj.handle(7) = quiver(obj.p(1),obj.p(2),negBound(1),negBound(2),.5,'Color','b',...
                        'LineWidth',1,'MaxHeadSize',obj.arrowSize,'DisplayName','Bounds');
                    obj.handle(8) = quiver(obj.p(1),obj.p(2),posBound(1),posBound(2),.5,'Color','b',...
                        'LineWidth',1,'MaxHeadSize',obj.arrowSize);
                else
                    set(obj.handle(1),'vertices', p_)
                % Update object bounds arrows (don't have list of bounds
                % for simulation section rn. Need to 'move' while giving live
                % bound information'
                    set(obj.handle(7),'XData',p(1),'YData',p(2),'UData',...
                        negBound(1),'VData',negBound(2));
                    set(obj.handle(8),'XData',p(1),'YData',p(2),'UData',...
                        posBound(1),'VData',posBound(2));
                end

                % Update applied force arrow
%                 fap = agentForce(obj,O);
%                 set(obj.handle(2),'XData',p(1),'YData',p(2),'UData',fap(1),...
%                     'VData',fap(2));  
                
                % Update agent velocity arrow
                set(obj.handle(6),'XData',p(1),'YData',p(2),'UData',dp(1),...
                    'VData',dp(2));
                
                if contains(obj.method, 'force')
                    % Update normal force arrow
                    [fn,ft] = forceReading(obj,O);
                    set(obj.handle(3),'XData',p(1),'YData',p(2),'UData',fn(1),...
                        'VData',fn(2));                
                    % Update tangent force arrow
                    set(obj.handle(4),'XData',p(1),'YData',p(2),'UData',ft(1),...
                        'VData',ft(2)); 
                    % Update friction force arrow
                    fric = aFrictionForce(obj,O);
                    set(obj.handle(5),'XData',p(1),'YData',p(2),'UData',fric(1),...
                        'VData',fric(2)); 
                end

                if(nargout > 0)
                    varargout{1} = p_;
                end
            end
        end
       
        %__________________________________________________________________
        % Agent VELOCITY and ACCELERATION
        function dx = dynamics(obj,O,t)
            %  O = an object of the "payload" class
            vel = obj.dp;

            if t == 0 % Initially accelerate @ centroid (defined as O.p)
                direc = (O.p-obj.p); % Directiom towards centroid
                acc = obj.a_desired*(direc./norm(direc));
            else
                if contains(obj.method,'optical')
%                     friction_term = aFrictionForce(obj,O,vel);
                    acc = comSearch(obj,O,t); % Use comSearch to change direction
                elseif contains(obj.method,'force')
                    acc = forceReading(obj,O);
                end
            end
            obj.deltaT = obj.deltaT + 1; % Create some delta-t to allow system time to respond

            dx(1:3,1) = vel;
            dx(4:6,1) = acc;
            %% Set CoM Prediction Line Vector
            if ~isempty(obj.a_prev)
                obj.measuredCoM = Rzd(rad2deg(O.p(3)))'*obj.a_prev(1:2);
            end
        end

        %__________________________________________________________________
        % AGENT COM SEARCH ALGORITHM - get new direc/accel
        function acc = comSearch(obj,O,t)
            N = 10; % Delta-T ~time steps between checking rotation

            % Every N time steps switch to DECEL mode, begin stopping obj
            if rem(obj.deltaT,N) == 0
                obj.mode = 'decel';
            end
            
            % Update bounds for graphics
            updateBounds(obj,O)

            % Based on robot state, take certain actions
            if contains(obj.mode, 'measure')
                acc = takeMeasurement(obj,O);
            elseif contains(obj.mode, 'decel')
                acc = decelerate(obj);
            elseif contains(obj.mode, 'stop')
                acc = [0;0;0];
                return
            else % If not decelerating, keep constant accel
                acc = obj.a;
                obj.a_prev = acc;
            end
            obj.prevTheta = O.p(3); % Record current payload rotation
        end
       
        %__________________________________________________________________
        % Update bounds every time step for graphics
        function updateBounds(obj,O)
            dTheta = (O.p(3) - obj.prevTheta); % Rot. from prev time-step
            R = Rzd(dTheta*180/pi);
            obj.negBound = R * obj.negBound;
            obj.posBound = R * obj.posBound;
        end
       
        %__________________________________________________________________
        % Agent MEASUREMENT of object rotation
        function acc = takeMeasurement(obj,O)
            measured_dTheta = (O.p(3) - obj.measuredTheta) * (180/pi);
            % If negligible rotation measured
            if abs(measured_dTheta) < 1
                obj.mode = 'stop';
                acc = [0; 0; 0];
                return
            end
            % If rotating counter-clockwise
            if measured_dTheta > 0
                newPosBound = Rzd(measured_dTheta) * obj.a_prev(1:2);
                obj.posBound = newPosBound./norm(newPosBound);
            % If rotating clockwise
            elseif measured_dTheta < 0
                newNegBound = Rzd(measured_dTheta) * obj.a_prev(1:2);
                obj.negBound = newNegBound./norm(newNegBound);
            end
            % Get new direction - bisect bounds w/ unit vector, then
            % multiply by original acceleration
            newDirection = obj.negBound + obj.posBound;
            norm_newDirection = newDirection./norm(newDirection);
            acc = norm(obj.a_desired) * [norm_newDirection;0];
    
            obj.mode = 'accel';
            obj.measuredTheta = O.p(3);
        end
       
        %__________________________________________________________________
        % Agent CANCEL OUT current acceleration
        function acc = decelerate(obj)
            % If negligible vel, set to measure mode
            if norm(obj.dp) < 1e-15
                obj.mode = 'measure';
                acc = [0; 0; 0];
            else
                acc = -60*obj.a_prev;
                if norm(acc) > norm(obj.dp)
                    acc = -obj.a_prev;
                end
            end
        end

        %__________________________________________________________________
        % The FORCE exerted by the agent to the object
        function [f,m] = agentForce(obj,O)
            % O = an object of the "payload" class
            % f = [fx;fy] agent force in {W}; (column)
            % m = moment of f applied to the object in z direction
            %______________________________________________________________
            if(obj.attached)             
                R = Rzd(O.p(3)*180/pi);                             
                % Force now is relative to the direction of acceleration
                f = 1 * obj.a/norm(obj.a);               
                f = f(1:2);
                
                if norm(obj.a) == 0
                    f = [0;0];
                end
                % Now find moment
                m = f(2)*R(1,:)*obj.r - f(1)*R(2,:)*obj.r;
                
            else
                f = [0;0];
                m = 0;
            end
        end
      
        %__________________________________________________________________
        % Agent FORCE READING
        function [fn,ft] = forceReading(obj,O)
            % f = [fNormal; fTangent]; 
            %______________________________________________________________            
            % Calculate friction force             
            F_fric = aFrictionForce(obj,O);

            % Get agent's applied force ~ resistive force
            agent_f = - agentForce(obj,O);
            
            % Get net forces acting on agent
            Fnet = F_fric + agent_f;

            % Project applied force to normal and tangent
            % Make unit vectors normal to and tangent to object.
            n = Rzd(O.p(3)*180/pi) * obj.r;
            unit_n = n/norm(n);
            t = Rzd((O.p(3) + pi/2)*180/pi) * obj.r;
            unit_t = t/norm(t);

            if ~isempty(agent_f)
                fn = (dot(Fnet,unit_n)/norm(unit_n)^2)*unit_n;
                ft = (dot(Fnet,unit_t)/norm(unit_t)^2)*unit_t;
            else
                fn = [0;0];
                ft = [0;0];
            end
        end

        %__________________________________________________________________
        % Agent FRICTION FORCE
        function [fric] = aFrictionForce(obj,O,vel)
            % Object dp is w.r.t the object's frame
            r_o = [O.p(1:2) - obj.p(1:2); 0]; % Vector from obj TO agent
            objAccel = O.frictionForce(vel);

            omega = cross(r_o, O.dp)/norm(r_o)^2; % Omega calculated with r from obj to agent
            alpha = cross(r_o, objAccel)/norm(r_o)^2;
            fricAcc = objAccel + cross(alpha, -r_o) - omega(3)^2*-r_o;
            agentFric = fricAcc(1:2);
            fric = [objAccel(1:2); 0];
%             TODO fix friction it doesnt work right now

%             dpo = O.dp; % O.dp/norm(O.dp);
%             ddpo = O.a;
%             if isnan(dpo)
%                 disp('dpo is nan')
%                 dpo = [0;0;0];
%             end
%             if isnan(ddpo)
%                 disp('ddpo is nan')
%                 ddpo = [0;0;0];
%             end
% 
%             % Making object's friction acceleration relative to agent
%             O_fric = O.friction;
%             omega = O.dp; % Omega calculated with r from obj to agent
% 
%             alpha = cross(r_o, O_fric)/norm(r_o)^2;
%             acc = O_fric + cross(alpha, -r_o) - omega(3)^2*-r_o;
% %             dx(4:6,1) = [acc(1:2); alpha(3)];
% 
%             fric(1:2,1) = acc(1:2); % (O.uk*O.mass*O.g*dpo(1:2))/O.mass;
%             fric(3,1) = 0; %- (O.uk*O.mass*O.g*dpo(3))/O.mass;
        end
    end
end

%_________________________________________________________________________
% Saturate U vector based on norm of V
function U = normSat(U,V)
    Vnorm = norm(V);
    Unorm = norm(U);
    if(Unorm > Vnorm)
        U = Vnorm/Unorm*U;
    end
end

%__________________________________________________________________________
% Rotation matrix about z axis
function R = Rz(q)
    R = [cos(q) -sin(q); sin(q) cos(q)];
end
% Rotation matrix about z axis FOR DEGREES
function R = Rzd(q)
    R = [cosd(q) -sind(q); sind(q) cosd(q)];
end
% Rotation about z 3x3
function R = Rz3(q)
    R = [cos(q) -sin(q) 0; sin(q) cos(q) 0; 0 0 0;];
end

%__________________________________________________________________________
% Find angle between two vectors
function theta = getAngle(u,v)
    if length(u) < 3
        disp('U and V must be length of 3')
    end
    theta = atan2(norm(cross(u,v)),dot(u,v));
end
