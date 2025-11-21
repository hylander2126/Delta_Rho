function R = Rz2(q, d)
    if nargin < 2, d = 0; end
    
    if strcmp(d,'d') || d==1
        R = [cosd(q),-sind(q);sind(q),cosd(q)];
    elseif d==0
        R = [cos(q),-sin(q);sin(q),cos(q)];
    else
        fprintf('\narg2 must be 0, 1, or "d"\n');
        return
    end
end