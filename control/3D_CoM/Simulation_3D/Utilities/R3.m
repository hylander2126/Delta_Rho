function R = R3(angle, axisVec, unitMode)
% R3  Rotation matrix for a rotation of ‘angle’ about ‘axisVec’
%
%   R = R3(angle)                   % rotates about +Z by ‘angle’ radians
%   R = R3(angle, axisVec)          % rotates about axisVec (any nonzero 3×1)
%   R = R3(angle, axisVec, 'deg')   % same, but ‘angle’ is in degrees
%
%   Examples:
%     R3(pi/4)                      % 45° about Z
%     R3(90, [0;1;0], 'deg')        % 90° about +Y
%     R3(30, [0;0;-1], 'deg')       % –30° about Z

    if nargin < 2
        axisVec = [0;0;1];
    end
    if nargin < 3
        unitMode = 'rad';
    end

    %— normalize axis and angle ------------------------------------------
    u = axisVec(:) / norm(axisVec);  
    q = angle;
    if strcmpi(unitMode,'deg')
        q = deg2rad(q);
    end

    %— Rodrigues formula ------------------------------------------------
    x = u(1);  y = u(2);  z = u(3);
    c = cos(q);
    s = sin(q);
    C = 1 - c;

    R = [ c + x^2*C,    x*y*C - z*s,  x*z*C + y*s;
          y*x*C + z*s,  c + y^2*C,    y*z*C - x*s;
          z*x*C - y*s,  z*y*C + x*s,  c + z^2*C ];
end
