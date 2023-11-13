clc; clear; close all
xTemp = [1 2 3];
yTemp = [1 2 3];
handle = [];

Plot(xTemp,yTemp)

function Plot(xTemp,yTemp)
    for n = 1:size(xTemp,2)
        x = xTemp(n);
        y = yTemp(n);
    
        ax = gca;
        hold(ax,'on');
        axis(ax,'equal');
        handle(1) = plot(x,y);
        handle(2) = plot(y,x);
    end
end