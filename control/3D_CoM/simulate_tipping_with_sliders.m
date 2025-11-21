%% Program to simulate tipping vs sliding. 



%% BROKEN CURRENTLY

function simulate_tipping_with_sliders()
    % --- Parameters ---
    params.m = 1.0;
    params.w = 0.4;
    params.h = 0.8;
    params.mu_s = 0.9;
    params.mu_k = 0.7;
    params.g = 9.81;
    params.dt = 0.01;
    params.corner_radius = 0;
    
    % Initial state [x; y; theta; vx; vy; omega]
    % Start with the rectangle resting on the ground.
    state = [0; params.h/2; 0; 0; 0; 0];
    
    % --- UI Setup ---
    fig = figure('Name', 'Tipping vs Slipping (Slider Controlled)', ...
                 'CloseRequestFcn', @exitSim, 'Position', [10, 10, 1000, 750]);
    movegui(fig, 'center');
    
    % Axes for visualization
    ax = axes('Parent', fig, 'Units', 'normalized', ...
              'Position', [0.05 0.35 0.6 0.6]);
    axis(ax, [-1 1 -0.1 1.5]); axis equal; hold on;
    fill([-20 20 20 -20], [0 0 -0.1 -0.1], [0.6 0.6 0.6]);  % ground
    rect = rectangle('Position', getRectPosition(state, params), ...
                     'Curvature', [params.corner_radius, params.corner_radius], ...
                     'FaceColor', [0.1 0.5 0.9]);
    
    % Initial force arrow (will be updated each frame)
    force_arrow = quiver(0, 0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);

    % --- Sliders and Value Displays ---
    % Force Magnitude
    uicontrol('Style', 'text', 'String', 'Force Magnitude (N)', ...
              'Units', 'normalized', 'Position', [0.7 0.85 0.25 0.05]);
    sMag = uicontrol('Style', 'slider', 'Min', 0, 'Max', 20, 'Value', 5, ...
              'Units', 'normalized', 'Position', [0.7 0.8 0.25 0.05]);
    sMagText = uicontrol('Style', 'text', 'String', sprintf('Force Magnitude (N): %.2f', sMag.Value), ...
              'Units', 'normalized', 'Position', [0.7 0.9 0.25 0.05]);
    
    % Force Height
    uicontrol('Style', 'text', 'String', 'Force Height (m)', ...
              'Units', 'normalized', 'Position', [0.7 0.7 0.25 0.05]);
    sHeight = uicontrol('Style', 'slider', 'Min', 0, 'Max', params.h, 'Value', params.h/2, ...
              'Units', 'normalized', 'Position', [0.7 0.65 0.25 0.05]);
    sHeightText = uicontrol('Style', 'text', 'String', sprintf('Force Height (m): %.2f', sHeight.Value), ...
              'Units', 'normalized', 'Position', [0.7 0.75 0.25 0.05]);
    
    running = true;

    while running && ishandle(fig)
        % --- Update slider value displays ---
        sMagText.String = sprintf('Force Magnitude (N): %.2f', sMag.Value);
        sHeightText.String = sprintf('Force Height (m): %.2f', sHeight.Value);
        
        % --- Force computation from sliders ---
        F_mag = sMag.Value;
        h = sHeight.Value;
        % Force is always horizontal (along x-axis)
        F = [F_mag; 0];

        % Apply force on the left side at the specified height.
        Fp = [state(1) - params.w/2; state(2) - params.h/2 + h];
        
        % Update arrow to show force visually.
        scale = 0.05;  % Arrow length scale per Newton
        arrow_vec = scale * F;
        force_arrow.XData = Fp(1);
        force_arrow.YData = Fp(2);
        force_arrow.UData = arrow_vec(1);
        force_arrow.VData = arrow_vec(2);
        
        % Integrate dynamics.
        [~, s_out] = ode45(@(t, y) dynamics(t, y, params, F, Fp), [0 params.dt], state);
        state = s_out(end, :)';
        
        % --- Ground collision: Clamp the rectangle's bottom to y = 0 ---
        bottom_y = state(2) - (params.h/2)*cos(state(3));
        state(2) = state(2) - bottom_y;  % Adjust vertical position so bottom is at 0.
        state(5) = 0;                    % Zero vertical velocity.
        
        % Update rectangle display.
        rect.Position = getRectPosition(state, params);

        % --- Auto-expand x-axis to follow rectangle ---
        margin = 0.5;  % extra space to right
        current_center_x = state(1);
        xlim_now = xlim(gca);
        if current_center_x + margin > xlim_now(2)
            new_xlim = [xlim_now(1), current_center_x + margin];
            xlim(gca, new_xlim);
        end

        drawnow;
        pause(params.dt);
    end

    % --- Dynamics Function ---
    function dydt = dynamics(~, y, p, F, Fp)
        x = y(1); y_pos = y(2); theta = y(3);
        vx = y(4); vy = y(5); omega = y(6);
        CoM = [x; y_pos];
    
        % Torque induced by the force.
        r = Fp - CoM;
        tau = r(1)*F(2) - r(2)*F(1);
    
        N = p.m * p.g;
        f_static_max = p.mu_s * N;
        f_kinetic = p.mu_k * N;
    
        tipping_threshold = N * (p.w/2);
        tipping = abs(tau) > tipping_threshold;
    
        % Ground contact (we enforce ground contact via clamping).
        bottom_y = y_pos - (p.h/2)*cos(theta);
        on_ground = abs(bottom_y) < 1e-4;
    
        % Check if external force is negligible.
        very_small_force = norm(F) < 1e-3;
        nearly_static = norm([vx, vy, omega]) < 1e-4;
    
        % Freeze if fully static on ground.
        if very_small_force && nearly_static && on_ground
            dydt = zeros(6,1);
            return;
        end
    
        % --- Friction and Motion ---
        if tipping
            ax = 0;
            alpha = tau / getInertia(p);
        elseif abs(vx) > 1e-3
            % Apply kinetic friction.
            fx_friction = -f_kinetic * sign(vx);
            ax = (F(1) + fx_friction) / p.m;
            alpha = 0;
        elseif ~very_small_force && abs(F(1)) < f_static_max
            % Static friction holds it.
            ax = 0;
            alpha = tau / getInertia(p);
        else
            % Force overcomes static friction.
            fx_friction = -f_kinetic * sign(F(1));
            ax = (F(1) + fx_friction) / p.m;
            alpha = 0;
        end
    
        % Force vertical acceleration to zero (block remains on ground).
        ay = 0;
    
        dydt = [vx; vy; omega; ax; ay; alpha];
    end

    function I = getInertia(p)
        I = (1/12) * p.m * (p.w^2 + p.h^2);
    end

    function pos = getRectPosition(state, p)
        x = state(1); y = state(2); theta = state(3);
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        corner = R * [-p.w/2; -p.h/2];
        pos = [x + corner(1), y + corner(2), p.w, p.h];
    end

    function exitSim(~, ~)
        running = false;
        delete(gcf);
    end
end
