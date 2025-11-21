function angle = getAngle2(v1, v2)
    %GETANGLE2 Calculate the angle from the reference of the second vector to the first vector.
    %
    % Syntax:
    %   angle = getAngle2(v1, v2)
    %
    % Description:
    %   Calculates the angle from the second vector (v2) to the first vector (v1) 
    %   using the cross product and dot product. The function returns the angle in radians
    %   between the two vectors, with positive angles indicating a counter-clockwise rotation
    %   from the second vector to the first vector.
    %
    % Inputs:
    %   v1 - A 2-element vector (can be row or column) representing the first vector.
    %   v2 - A 2-element vector (can be row or column) representing the second vector.
    %
    % Outputs:
    %   angle - The angle between the two vectors in radians, measured from v2 to v1.
    %
    % Example:
    %   v1 = [1, 0];
    %   v2 = [0, 1];
    %   theta = getAngle2(v1, v2);
    %   disp(rad2deg(theta)); % Displays -90 degrees
    %
    % Note:
    %   Both vectors must have the same orientation (either both row or both column vectors)
    %   for the function to work correctly.
    %
    % See also: ATAN2, DOT, CROSS

    % Ensure the vectors are both row or column vectors of the same type
    if length(v1) ~= 2 || length(v2) ~= 2
        error('Both vectors must be 2-element vectors.');
    end

    % Calculate the angle using the formula for the angle between vectors
    % The order of v2 and v1 is swapped in both the cross and dot products
    angle = atan2(v2(1) * v1(2) - v2(2) * v1(1), v2(1) * v1(1) + v2(2) * v1(2));
end
