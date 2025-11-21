function angle = getAngle(u,v,spec)
    % Compute angle between vectors using atan2 method.
    if contains(spec,'d')
        angle = atan2d(v(2), v(1)) - atan2d(u(2), u(1));
    else
        angle = atan2(v(2), v(1)) - atan2(u(2), u(1));  
    end

end