close all; clc; clear;

%% Second simulation for tipping vs sliding

%% Parameter Definitions
m = 1;          % mass of rectangle [kg]
L = 2.0;        % width (base length) of rectangle [m]
H = 4.0;        % height of rectangle [m]
g = 9.81;       % gravitational acceleration [m/s^2]
mu = 0.5;       % friction coefficient
F = 5.45;         % applied horizontal force [N]
h_force = 0.1;  % height (from bottom) at which force is applied [m]
stopAtEquilibrium = true;  % Option: stop tipping simulation at equilibrium

%% Compute Thresholds
F_slide = mu * m * g;            % Minimum force to overcome friction (sliding threshold)
F_tip = (m * g * (L/2)) / h_force; % Minimum force to tip (tipping threshold)
fprintf('F_slide = %.2f N, F_tip = %.2f N\n', F_slide, F_tip);

%% Decision: No Motion, Sliding, or Tipping
if F < F_slide
    disp('No motion: Applied force is insufficient to overcome static friction.');
    
elseif F < F_tip
    %% Sliding Animation
    disp('Sliding occurs.');
    % Compute net acceleration (assuming kinetic friction)
    a_slide = (F - F_slide) / m;
    
    % Time parameters for animation
    t_end = 2;      % simulation time [s]
    dt = 0.02;      % time step [s]
    t = 0:dt:t_end;
    
    % Compute horizontal displacement (starting from rest)
    x_disp = 0.5 * a_slide * t.^2;
    
    % Animate sliding (rectangle remains horizontal)
    figure;
    for k = 1:length(t)
        clf;
        % Bottom-left corner of the rectangle is at (x_disp, 0)
        x0 = x_disp(k);
        % Define rectangle corners (clockwise): 
        % bottom left, bottom right, top right, top left, and back to bottom left.
        rect_x = [x0, x0+L, x0+L, x0, x0];
        rect_y = [0, 0, H, H, 0];
        
        % Plot the rectangle and ground
        plot(rect_x, rect_y, 'b-', 'LineWidth', 2);
        hold on;
        plot([x0-1, x0+L+1], [0, 0], 'k-', 'LineWidth', 2);
        axis equal;
        xlim([x0-0.5, x0+L+1]);
        ylim([-0.1, H+0.5]);
        title(sprintf('Sliding Animation: t = %.2f s', t(k)));
        xlabel('X (m)'); ylabel('Y (m)');
        drawnow;
        pause(dt);
    end
    
else
    %% Tipping Animation about Bottom-Right Pivot with Equilibrium Detection
    disp('Tipping occurs.');
    
    % Moment of inertia about the bottom-right pivot.
    % Using the parallel axis theorem for a rectangle: I_p = m*(L^2+H^2)/3.
    I_p = m*(L^2+H^2)/3;
    
    % Define the ODE for rotational motion about the bottom-right pivot.
    % y(1) = theta (rotation angle, in radians; note: clockwise rotation gives negative theta)
    % y(2) = theta_dot (angular velocity)
    % Net moment = gravitational moment + moment due to applied force:
    tipODE = @(t, y) [y(2); (- m*g*((L/2)*cos(y(1)) + (H/2)*sin(y(1))) + F*( L*sin(y(1)) - h_force*cos(y(1)) )) / I_p];
    
    % Set up event function to stop when COM is directly above the pivot.
    % The condition is: L*cos(theta) + H*sin(theta) = 0.
    eventFunction = @(t, y) equilibriumEvent(t, y, L, H);
    options = odeset('Events', eventFunction);
    
    % Time span and initial conditions (starting from rest with theta = 0)
    tspan = [0 2];
    y0 = [0; 0];
    
    if stopAtEquilibrium
        [t_sol, y_sol, te, ye, ie] = ode45(tipODE, tspan, y0, options);
    else
        [t_sol, y_sol] = ode45(tipODE, tspan, y0);
        te = []; ye = [];
    end
    theta = y_sol(:,1);
    
    % Compute required force at each time step based on moment equilibrium:
    % To maintain zero angular acceleration, required force is:
    %   F_required(theta) = m*g*((L/2)*cos(theta)+(H/2)*sin(theta)) / (L*sin(theta)-h_force*cos(theta))
    F_required = (m*g*((L/2)*cos(theta) + (H/2)*sin(theta))) ./ (L*sin(theta) - h_force*cos(theta));
    
    % Animate tipping: The rectangle rotates about its bottom-right pivot (pivot at [L,0])
    figure;
    for k = 1:length(t_sol)
        clf;
        theta_k = theta(k);
        % Define the pivot (bottom-right corner) in the global frame
        pivot = [L; 0];
        % Original rectangle corners (from bottom left): (0,0), (L,0), (L,H), (0,H), (0,0)
        corners = [0, 0;
                   L, 0;
                   L, H;
                   0, H;
                   0, 0]';
        % Shift coordinates so that the pivot is at the origin.
        corners_shifted = corners - repmat(pivot, 1, size(corners,2));
        % Rotation matrix for angle theta_k
        R = [cos(theta_k), -sin(theta_k); sin(theta_k), cos(theta_k)];
        % Rotate the shifted corners
        rotated_corners = R * corners_shifted;
        % Shift back to the global coordinate system
        final_corners = rotated_corners + repmat(pivot, 1, size(corners,2));
        
        % Plot the rotated rectangle and ground
        plot(final_corners(1, :), final_corners(2, :), 'r-', 'LineWidth', 2);
        hold on;
        ground_x = [pivot(1)-L-1, pivot(1)+L+1];
        plot(ground_x, [0,0], 'k-', 'LineWidth', 2);
        % Mark the pivot
        plot(pivot(1), pivot(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
        axis equal;
        xlim([pivot(1)-L-1, pivot(1)+L+1]);
        ylim([-0.5, H+1]);
        title(sprintf('Tipping Animation (Pivot at Bottom Right): t = %.2f s', t_sol(k)));
        xlabel('X (m)'); ylabel('Y (m)');
        drawnow;
        pause(0.02);
    end
    
    % Plot the computed required force vs tipping angle.
    figure;
    plot(-rad2deg(theta), -F_required, 'LineWidth', 2); % Swap angle (abs value)
    xlabel('Tipping Angle (deg)');
    ylabel('Required Force (N)');
    title('Force Required vs Tipping Angle');
    grid on;
    
    % If an equilibrium event was detected, record the equilibrium angle.
    if ~isempty(te)
        equilibriumAngle = ye(1);
        fprintf('Equilibrium reached at theta = %.4f radians (%.2f degrees).\n', equilibriumAngle, equilibriumAngle*180/pi);
    else
        fprintf('No equilibrium event was detected within the simulation time.\n');
    end
end

%% Nested Function: Event Detection for Equilibrium
function [value, isterminal, direction] = equilibriumEvent(~, y, L, H)
    % The event is triggered when the center-of-mass is directly above the pivot.
    
    % For a rectangle rotating about its bottom-right corner, the COM relative to the pivot (in unrotated state) 
    % is (-L/2, H/2). After rotation by theta = y(1), the horizontal (x) coordinate 
    % becomes: x_cm = -L/2*cos(theta) - H/2*sin(theta).
    
    % Equilibrium (vertical COM over pivot) is reached when x_cm = 0, i.e.,
    %   L*cos(theta) + H*sin(theta) = 0.
    theta = y(1);
    value = L*cos(theta) + H*sin(theta); % when this crosses zero, equilibrium is reached.
    isterminal = 1;   % Stop the integration when the event occurs.
    direction = -1;   % Only detect when the value is decreasing through zero.
end
