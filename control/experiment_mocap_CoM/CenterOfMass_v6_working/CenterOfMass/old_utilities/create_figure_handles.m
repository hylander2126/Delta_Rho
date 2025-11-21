% Initial call to generate live plot of agent, object, bounds, etc.

%% Initialize quiver and object patch for live plot
if generateFigure
    arrow = zeros(5);
    for n=1:length(arrow)
        arrow(n) = quiver(0,0,.25,0,0,'MaxHeadSize',10,'LineWidth',2);
    end
    
    % Initialize object patch handle
    qq = linspace(0,2*pi,20);
    circlePoints(:,1) = 0.85*cos(qq);
    circlePoints(:,2) = 0.85*sin(qq);
    circle(1) = patch('Vertices',circlePoints,'Faces',1:size(circlePoints,1),'FaceColor', ...
        [0.6549, 0.8353, 0.5412],'EdgeColor','k','FaceAlpha',0.5);
end

%% Update object initial frame arrows
if generateFigure
    u = 0.2*cos(theta_p0);
    v = 0.2*sin(theta_p0);
    set(arrow(1),'xdata',x(1),'ydata',y(1),'udata',.25,'vdata',0,'Color','k');
    set(arrow(2),'xdata',x(1),'ydata',y(1),'udata',0,'vdata',.25,'Color','k');
    % Update object continuous frame arrows
    circle(2) = quiver(x(1),y(1),0,0.1,0.3,'k','LineWidth',1,'MaxHeadSize',.3);
    circle(3) = quiver(x(1),y(1),0.1,0,0.3,'k','LineWidth',1,'MaxHeadSize',.3);
    % Update agent continuous frame arrows
    circle(4) = quiver(x(2),y(2),0,0.1,0.3,'k','LineWidth',1,'MaxHeadSize',.3);
    circle(5) = quiver(x(2),y(2),0.1,0,0.3,'k','LineWidth',1,'MaxHeadSize',.3);
end