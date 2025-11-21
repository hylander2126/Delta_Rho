function [five_bar] = sensor_kinematics(five_bar)
    
    % Deconstruct five_bar class
    k           = five_bar.spring_k;
    p           = five_bar.p;
    l           = five_bar.links;
    alpha       = five_bar.alpha;
    home        = five_bar.home;
    phi         = five_bar.phi;
    ctr         = five_bar.ctr;
    window_size = five_bar.window_size;
    calib       = five_bar.calib;

    %% Calculate coordinates of all other joints
    % Coordinates of joint 3 and 4
    for i=[1,2]
        p(:,i+2) = p(:,i) + l(i)*[cos(alpha(i)); sin(alpha(i))];
    end
    
    % Get vector between joints 3 and 4
    lambda = p(:,4)-p(:,3);
    
    % Calculate angle between lambda and l3 using law of cosines
    xi = acos((l(3).^2 + norm(lambda).^2 - l(4).^2)./(2*l(3).*norm(lambda)));
    
    % Find EE (p5): multiply unit vector along lambda by l3, rotating by xi, then moving by p3
    p(:,5) = p(:,3) + Rz2(xi) * (l(3) * (lambda/norm(lambda)));
    
    % Home config of EE calculated with resting alpha=(60,120) - run script with these values to generate    
    delta = p(:,5) - home;
    % ROTATING DELTA DUE TO WEIRD SKEW IN FORCE SENSOR
    % delta = Rz2(0.1974) * delta;
    
    %% Now calculate force
    gamma = [0;0];
    for i=[1,2]
        % Shift EE vector to origin
        p_e = p(:,5) - p(:,i+2);
        % Shift l1,l2 to origin
        p_l = p(:,i+2) - p(:,i);
        % Calculate gamma
        gamma(i) = acos(dot(p_l, p_e) / (norm(p_l)*norm(p_e)));
    end
    
    % Get difference between current reading and resting angle - UPDATED with NOMINAL VALUES
    dAlpha = alpha - phi;
    
    % Calculate force transmitted thru l3 and l4
    f3 = abs(k*dAlpha(1) / (l(1)*sin(gamma(1))))  * (1/9.807) * (1000/1);
    f4 = abs(k*dAlpha(2) / (l(2)*sin(gamma(2)))) * (1/9.807) * (1000/1);
   
    %% LAST WAY - STUPID BUT IT MIGHT WORK. SIMPLY SCALE DELTA BY A FACTOR DEPENDING ON ITS ANGLE
    delta_angle = acos(dot(-delta, [0,1]) / (norm(-delta) * norm([0,1])));

    % Linear interpolation (from 0 degrees to 30 degrees)
    scaling_fn = @(x) interp1([0 deg2rad(30)], [2.7 3.2], x, 'linear','extrap');
    scaling_factor = scaling_fn(delta_angle);

    force = scaling_factor*norm(delta);

    if abs(force) < five_bar.deadzone
        force = 0;
        delta = [0; 0];
    end

    %% Update class force, direction, other plotting
    % Store current direc measurement in array of prev direcs
    % five_bar.prev_direc(ctr,:) = delta;

     % five_bar.direction = mean(five_bar.prev_direc); % Rolling average of past n samples
    five_bar.direction = delta;
    five_bar.force = force;
    five_bar.p = p;             % ONLY USED FOR 'LIVE' PLOT

    % TEMP
    five_bar.gamma = gamma;

    % if ctr < window_size
    %     five_bar.ctr = ctr + 1;
    % else
    %     five_bar.ctr = 1;
    % end
end