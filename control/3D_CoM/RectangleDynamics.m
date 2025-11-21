function RectangleDynamics()
    % Initialize the figure and graphics
    fig = figure('Position', [100, 100, 900, 600], 'Name', 'Rectangle Dynamics Simulation');
    
    % Create main axes for simulation
    ax = axes('Parent', fig, 'Position', [0.1, 0.3, 0.8, 0.6]);
    hold(ax, 'on');
    axis(ax, 'equal');
    title('Rectangle Dynamics: Slipping vs. Tipping');
    xlabel('Distance (m)');
    ylabel('Height (m)');
    
    % Create sliders panel
    sliderPanel = uipanel('Title', 'Force Controls', 'Position', [0.1, 0.05, 0.8, 0.2]);
    
    % Create sliders
    forceMagSlider = uicontrol('Parent', sliderPanel, 'Style', 'slider', ...
        'Position', [20, 80, 300, 20], 'Min', 0, 'Max', 20, 'Value', 0, ...
        'Callback', @updateForce);
    uicontrol('Parent', sliderPanel, 'Style', 'text', ...
        'Position', [330, 80, 100, 20], 'String', 'Force: 0 N');
    
    forceHeightSlider = uicontrol('Parent', sliderPanel, 'Style', 'slider', ...
        'Position', [20, 50, 300, 20], 'Min', 0, 'Max', 1, 'Value', 0.5, ...
        'Callback', @updateForce);
    uicontrol('Parent', sliderPanel, 'Style', 'text', ...
        'Position', [330, 50, 100, 20], 'String', 'Height: 50%');
    
    forceAngleSlider = uicontrol('Parent', sliderPanel, 'Style', 'slider', ...
        'Position', [20, 20, 300, 20], 'Min', -90, 'Max', 90, 'Value', 0, ...
        'Callback', @updateForce);
    uicontrol('Parent', sliderPanel, 'Style', 'text', ...
        'Position', [330, 20, 100, 20], 'String', 'Angle: 0°');
    
    % Reset button
    uicontrol('Parent', sliderPanel, 'Style', 'pushbutton', ...
        'Position', [500, 50, 100, 30], 'String', 'Reset', ...
        'Callback', @resetSimulation);
    
    % Physical parameters
    params.width = 0.5;        % Width of rectangle (m)
    params.height = 0.3;       % Height of rectangle (m)
    params.mass = 1;          % Mass of rectangle (kg)
    params.g = 9.81;          % Gravity (m/s^2)
    params.mu_s = 0.5;        % Static friction coefficient
    params.mu_k = 0.3;        % Kinetic friction coefficient
    
    % Initialize force parameters
    force.magnitude = 0;      % Force magnitude (N)
    force.height = 0.5;       % Relative height on left edge (0-1)
    force.angle = 0;          % Force angle (degrees)
    
    % Initialize state variables
    state.position = [0; params.height/2];  % Center position [x; y]
    state.velocity = [0; 0];                % Linear velocity [vx; vy]
    state.angle = 0;                        % Rotation angle (rad)
    state.angularVelocity = 0;              % Angular velocity (rad/s)
    state.onGround = true;                  % Is block on ground?
    state.tipping = false;                  % Is block tipping?
    state.slipping = false;                 % Is block slipping?
    
    % Calculate moment of inertia (for rectangular plate)
    params.I = params.mass * (params.width^2 + params.height^2) / 12;
    
    % Store global data
    data.params = params;
    data.force = force;
    data.state = state;
    data.forceArrow = [];
    data.textInfo = [];
    data.rectangle = [];
    data.tipPoint = [];
    data.dt = 0.02;  % Time step (s)
    data.timeElapsed = 0;
    
    % Set up timer for simulation
    simTimer = timer('ExecutionMode', 'fixedRate', ...
        'Period', data.dt, ...
        'TimerFcn', @(~,~) updateSimulation(data));
    
    % Store data in figure
    guidata(fig, data);
    
    % Set up clean-up function when figure is closed
    set(fig, 'CloseRequestFcn', @(src,~) closeFigure(src, simTimer));
    
    % Initialize visualization
    updateVisualization(data);
    
    % Start simulation
    start(simTimer);
    
    % Function to update force parameters from sliders
    function updateForce(src, ~)
        data = guidata(gcf);
        
        % Update force parameters based on which slider was changed
        switch src.Position(2)
            case 80  % Force magnitude slider
                data.force.magnitude = src.Value;
                src.Parent.Children(5).String = ['Force: ', num2str(src.Value, '%.1f'), ' N'];
            case 50  % Force height slider
                data.force.height = src.Value;
                src.Parent.Children(3).String = ['Height: ', num2str(src.Value*100, '%.0f'), '%'];
            case 20  % Force angle slider
                data.force.angle = src.Value;
                src.Parent.Children(1).String = ['Angle: ', num2str(src.Value, '%.0f'), '°'];
        end
        
        guidata(gcf, data);
        
        % Update visualization with new force
        updateVisualization(data);
    end
    
    % Function to reset simulation
    function resetSimulation(~, ~)
        data = guidata(gcf);
        
        % Reset state
        data.state.position = [0; data.params.height/2];
        data.state.velocity = [0; 0];
        data.state.angle = 0;
        data.state.angularVelocity = 0;
        data.state.onGround = true;
        data.state.tipping = false;
        data.state.slipping = false;
        data.timeElapsed = 0;
        
        guidata(gcf, data);
        
        % Update visualization
        updateVisualization(data);
    end
    
    % Function to properly close figure and timer
    function closeFigure(src, t)
        if isvalid(t)
            stop(t);
            delete(t);
        end
        delete(src);
    end
end

function updateSimulation(data)
    % Get latest data
    fig = gcf;
    data = guidata(fig);
    
    % Calculate new state based on physics
    data = updatePhysics(data);
    
    % Update visualization
    updateVisualization(data);
    
    % Store updated data
    guidata(fig, data);
end

function data = updatePhysics(data)
    % Unpack data
    params = data.params;
    state = data.state;
    force = data.force;
    dt = data.dt;
    
    % Convert rectangle corners to global coordinates
    corners = getRectangleCorners(state.position, params.width, params.height, state.angle);
    
    % Find corners in contact with ground (y=0)
    groundCorners = corners(:, corners(2,:) <= 1e-6);
    
    % If no corners are on ground, check if we need to handle collision
    if isempty(groundCorners) && state.onGround
        state.onGround = false;
    elseif ~isempty(groundCorners) && ~state.onGround
        % Handle collision with ground
        state.velocity(2) = -0.5 * state.velocity(2);  % Simple bounce with damping
        state.onGround = true;
    end
    
    % Calculate force vector
    forceAngleRad = force.magnitude * deg2rad(force.angle);
    forceVector = force.magnitude * [cos(forceAngleRad); sin(forceAngleRad)];
    
    % Calculate force application point on left edge
    forceHeight = force.height * params.height;
    
    % Calculate weight force
    weightForce = [0; -params.mass * params.g];
    
    % Check if block should tip or slip
    if state.onGround && ~state.tipping && ~state.slipping
        % Find bottom-left corner (will be the tipping point)
        cornerDistances = sqrt(sum((corners - repmat([min(corners(1,:)); 0], 1, 4)).^2));
        [~, cornerIndex] = min(cornerDistances);
        tipPoint = corners(:, cornerIndex);
        
        % Calculate lever arm from tipping point to center of mass
        leverArmCM = state.position - tipPoint;
        
        % Calculate lever arm from tipping point to force application point
        forcePoint = [min(corners(1,:)); forceHeight];
        leverArmForce = forcePoint - tipPoint;
        
        % Calculate torques
        torqueWeight = cross2D(leverArmCM, weightForce);
        torqueForce = cross2D(leverArmForce, forceVector);
        netTorque = torqueWeight + torqueForce;
        
        % Calculate horizontal forces for friction
        horizontalForce = forceVector(1);
        normalForce = params.mass * params.g - forceVector(2);
        
        % Friction force
        frictionForce = params.mu_s * normalForce;
        
        % Determine if block tips or slips
        if netTorque > 0 && normalForce > 0  % Only tip if force is trying to rotate counter-clockwise
            state.tipping = true;
            state.tipPoint = tipPoint;
        elseif abs(horizontalForce) > frictionForce && normalForce > 0
            state.slipping = true;
        end
    end
    
    % Update state based on physics
    if state.tipping
        % For tipping, we rotate around the tipping point
        % Calculate moment of inertia around tipping point (parallel axis theorem)
        r = norm(state.position - state.tipPoint);
        I_tip = params.I + params.mass * r^2;
        
        % Calculate torque around tip point
        leverArmCM = state.position - state.tipPoint;
        torqueWeight = cross2D(leverArmCM, weightForce);
        
        % Calculate force application point in global coordinates
        forcePoint = [min(corners(1,:)); forceHeight];
        leverArmForce = forcePoint - state.tipPoint;
        torqueForce = cross2D(leverArmForce, forceVector);
        
        netTorque = torqueWeight + torqueForce;
        
        % Angular acceleration
        alpha = netTorque / I_tip;
        
        % Update angular velocity and angle
        state.angularVelocity = state.angularVelocity + alpha * dt;
        state.angle = state.angle + state.angularVelocity * dt;
        
        % Update position based on rotation around tip point
        R = [cos(state.angularVelocity * dt), -sin(state.angularVelocity * dt);
             sin(state.angularVelocity * dt), cos(state.angularVelocity * dt)];
        state.position = state.tipPoint + R * (state.position - state.tipPoint);
        
        % Check if tipping changes back to normal state
        if state.angle < 0
            state.tipping = false;
            state.angle = 0;
            state.angularVelocity = 0;
        end
    else
        % For slipping or normal state
        netForce = weightForce;
        
        if state.slipping
            % Calculate kinetic friction direction
            frictionDirection = -sign(state.velocity(1));
            if state.velocity(1) == 0
                frictionDirection = -sign(forceVector(1));
            end
            
            % Calculate normal force
            normalForce = params.mass * params.g - forceVector(2);
            
            % Add applied force and kinetic friction
            if normalForce > 0  % Only if block is on ground
                netForce = netForce + forceVector + [frictionDirection * params.mu_k * normalForce; 0];
            else
                netForce = netForce + forceVector;
            end
        else
            % Add applied force
            netForce = netForce + forceVector;
        end
        
        % Calculate acceleration
        acceleration = netForce / params.mass;
        
        % Update velocity and position
        state.velocity = state.velocity + acceleration * dt;
        state.position = state.position + state.velocity * dt;
        
        % Check ground constraint
        if state.position(2) < params.height/2
            state.position(2) = params.height/2;
            state.velocity(2) = 0;
        end
        
        % Check if block stops slipping
        if state.slipping && abs(state.velocity(1)) < 1e-6
            state.slipping = false;
            state.velocity(1) = 0;
        end
    end
    
    % Update data
    data.state = state;
    data.timeElapsed = data.timeElapsed + dt;
end

function updateVisualization(data)
    % Get current axes
    ax = gca;
    
    % Unpack data
    params = data.params;
    state = data.state;
    force = data.force;
    
    % Calculate rectangle corners for visualization
    corners = getRectangleCorners(state.position, params.width, params.height, state.angle);
    
    % Draw or update rectangle
    if isempty(data.rectangle) || ~isvalid(data.rectangle)
        data.rectangle = patch(corners(1,:), corners(2,:), 'b', 'FaceAlpha', 0.5);
    else
        set(data.rectangle, 'XData', corners(1,:), 'YData', corners(2,:));
    end
    
    % Draw or update force vector if magnitude > 0
    if force.magnitude > 0
        % Calculate force vector components
        forceAngleRad = deg2rad(force.angle);
        fx = force.magnitude * cos(forceAngleRad) * 0.02; % Scale for visualization
        fy = force.magnitude * sin(forceAngleRad) * 0.02;
        
        % Calculate force application point
        minX = min(corners(1,:));
        forceStartPoint = [minX; force.height * params.height];
        forceEndPoint = forceStartPoint + [fx; fy];
        
        % Draw or update arrow
        if isempty(data.forceArrow) || ~isvalid(data.forceArrow)
            data.forceArrow = quiver(forceStartPoint(1), forceStartPoint(2), ...
                fx, fy, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
        else
            set(data.forceArrow, 'XData', forceStartPoint(1), 'YData', forceStartPoint(2), ...
                'UData', fx, 'VData', fy);
        end
    elseif ~isempty(data.forceArrow) && isvalid(data.forceArrow)
        delete(data.forceArrow);
        data.forceArrow = [];
    end
    
    % Draw tipping point if block is tipping
    if state.tipping
        if isempty(data.tipPoint) || ~isvalid(data.tipPoint)
            data.tipPoint = plot(state.tipPoint(1), state.tipPoint(2), 'ro', 'MarkerSize', 8);
        else
            set(data.tipPoint, 'XData', state.tipPoint(1), 'YData', state.tipPoint(2));
        end
    elseif ~isempty(data.tipPoint) && isvalid(data.tipPoint)
        delete(data.tipPoint);
        data.tipPoint = [];
    end
    
    % Display state information
    stateText = sprintf('Position: (%.2f, %.2f) m\nAngle: %.1f°\nTime: %.1f s', ...
        state.position(1), state.position(2), rad2deg(state.angle), data.timeElapsed);
    
    if state.tipping
        stateText = [stateText, 'State: Tipping'];
    elseif state.slipping
        stateText = [stateText, 'State: Slipping'];
    else
        stateText = [stateText, 'State: Stable'];
    end
    
    if isempty(data.textInfo) || ~isvalid(data.textInfo)
        data.textInfo = text(0.02, 0.98, stateText, ...
            'Units', 'normalized', 'VerticalAlignment', 'top', ...
            'BackgroundColor', [1 1 1 0.7]);
    else
        set(data.textInfo, 'String', stateText);
    end
    
    % Update axis limits to follow the rectangle if it moves
    axisMargin = 0.5;  % margin around rectangle
    xlim([min(min(corners(1,:))-axisMargin, -axisMargin), ...
          max(max(corners(1,:))+axisMargin, params.width+axisMargin)]);
    ylim([0, max(max(corners(2,:))+axisMargin, params.height+axisMargin)]);
    
    % Draw ground line
    hold on;
    line(xlim, [0, 0], 'Color', 'k', 'LineWidth', 2);
    
    % Force redraw
    drawnow;
end

function corners = getRectangleCorners(center, width, height, angle)
    % Calculate rectangle corners in local coordinates
    local_corners = [
        -width/2, -height/2;  % bottom-left
        width/2, -height/2;   % bottom-right
        width/2, height/2;    % top-right
        -width/2, height/2    % top-left
    ]';
    
    % Create rotation matrix
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    
    % Rotate corners
    rotated_corners = R * local_corners;
    
    % Translate to center position
    corners = rotated_corners + repmat(center, 1, 4);
end

function result = cross2D(a, b)
    % 2D cross product (returns scalar)
    result = a(1)*b(2) - a(2)*b(1);
end