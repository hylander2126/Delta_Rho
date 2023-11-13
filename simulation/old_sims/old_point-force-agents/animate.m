function [frames] = animate(t,X,stop)

global R Pi_o Rz
global X0 Xd

global J J_


shapeN = 15;
Q = linspace(0,2*pi,shapeN);

shape = R*[cos(Q);sin(Q)];

coordinate = @(x,axis) [x(1:2), x(1:2)+ R*Rz(x(3))*axis];

O2W_x = @(X,region) X(1) + [cos(X(3)) -sin(X(3))]*region;
O2W_y = @(X,region) X(2) + [sin(X(3))  cos(X(3))]*region;


N = size(Pi_o,2);
Pi_W = zeros(2,N);
f = zeros(2,N);

L.x = [ min([X0(1) Xd(1)])-3*R max([X0(1) Xd(1)])+3*R ];
L.y = [ min([X0(2) Xd(2)])-3*R max([X0(2) Xd(2)])+3*R ];


if(stop ~= 3)
    
    for n=1:length(t)
        
        fill(O2W_x(X(n,1:3),shape),O2W_y(X(n,1:3),shape),'c')
        
        hold on
        
        
        plot([L.x],[Xd(2) Xd(2)],'--k');
        plot([Xd(1) Xd(1)],[L.y],'--k');
        plot([Xd(1) Xd(1)+5*cos(Xd(3))],[Xd(2) Xd(2)+5*sin(Xd(3))],'-.k');
        
        for i=1:N
            Pi_W(:,i) = [O2W_x(X(n,1:3),Pi_o(:,i)) ; O2W_y(X(n,1:3),Pi_o(:,i))];
            f(:,i) = Rz(X(n,3))*controller(X(n,:)',J_(:,:,i));
        end
        
        plot(Pi_W(1,:),Pi_W(2,:),'Ob');
        quiver(Pi_W(1,:),Pi_W(2,:),f(1,:),f(2,:),0);
        
        Cx = coordinate(X(n,1:3)',[1;0]);
        Cy = coordinate(X(n,1:3)',[0;1]);
        
        plot(Cx(1,:),Cx(2,:),'r','LineWidth',2);
        plot(Cy(1,:),Cy(2,:),'b','LineWidth',2);
        
        text(L.x(1)+R,L.y(1)+R,['Time = ',num2str(t(n))]);
        
        axis equal
        axis([L.x L.y]);
        xlabel('x [m]');
        ylabel('y [m]');
        
        if(n ==1 && stop == 1)
            pause();
        elseif(stop == 2)
            pause();
        end
        
        
        %drawnow();
        frames(n) = getframe();
        hold off
    end
    
else
    hold on
    
    cm = colormap('cool');
    jump = round(length(t)/10);
    
    cmL = length(1:jump:length(t));
    
    color = @(n) cm(round(63/(cmL-1)*(n-1)+1),:);
    
    plot([L.x],[Xd(2) Xd(2)],'--k');
    plot([Xd(1) Xd(1)],[L.y],'--k');
    plot([Xd(1) Xd(1)+5*cos(Xd(3))],[Xd(2) Xd(2)+5*sin(Xd(3))],'-.k');
    
    n_ = 0;
    for n=1:jump:length(t)
        n_ = n_ + 1;
        fill(O2W_x(X(n,1:3),shape),O2W_y(X(n,1:3),shape),color(n_),'FaceAlpha', 0.2)
        
        Cx = coordinate(X(n,1:3)',[1;0]);
        Cy = coordinate(X(n,1:3)',[0;1]);
        
        plot(Cx(1,:),Cx(2,:),'r','LineWidth',2);
        plot(Cy(1,:),Cy(2,:),'b','LineWidth',2);
    end
    
    axis equal
    axis([L.x L.y]);
    xlabel('x [m]');
    ylabel('y [m]');
    frames = getframe();
end