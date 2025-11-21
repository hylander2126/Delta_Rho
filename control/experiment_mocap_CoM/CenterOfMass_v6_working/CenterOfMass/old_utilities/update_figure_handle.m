% Intermediate call - inside loop. To update agent, object, bounds positions.

if generateFigure
        % Object shape
        R_temp = Rz2d(O_pos(3));
        p_ = ones(size(circlePoints,1),1)*O_pos(1:2)' + 0.085*circlePoints*R_temp';
        set(circle(1), 'Vertices',p_);
        
        % Object Frame
        fRadius = 0.1; % Radius of diplayed frame
        set(circle(2),'xdata',x(1),'ydata',y(1),'udata',fRadius*cos(qo), ...
            'vdata',fRadius*sin(qo))
        set(circle(3),'xdata',x(1),'ydata',y(1),'udata',fRadius*cos(qo+pi/2), ...
            'vdata',fRadius*sin(qo+pi/2))
    
        % Agent Frame
        set(circle(4),'xdata',x(2),'ydata',y(2),'udata',fRadius*cos(qa), ...
            'vdata',fRadius*sin(qa))
        set(circle(5),'xdata',x(2),'ydata',y(2),'udata',fRadius*cos(qa+pi/2), ...
            'vdata',fRadius*sin(qa+pi/2))
    
        dxd = (Robot.dxd/norm(Robot.dxd));
        set(arrow(3),'xdata',x(2),'ydata',y(2),'udata',dxd(1)/5,'vdata',dxd(2)/5)
        
        % Bounds (Update object edge position))
        posBound = Robot.posBound;
        negBound = Robot.negBound;
        O_edge = Rz2d(rad2deg(qo))*[0.073; 0] + O_pos(1:2);
        
        set(arrow(4),'xdata',O_edge(1),'ydata',O_edge(2),'udata',posBound(1)/10,'vdata', ...
        posBound(2)/10,'Color','g');
        set(arrow(5),'xdata',O_edge(1),'ydata',O_edge(2),'udata',negBound(1)/10,'vdata', ...
        negBound(2)/10,'Color','b');
    end