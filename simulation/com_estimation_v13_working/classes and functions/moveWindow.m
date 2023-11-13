%% Sub-routine to move the window if desired. Follows the object/agent 
%% trajectory by changing the axes limits.

xLimits = get(gca,'XLim');
yLimits = get(gca,'YLim');
new_xLimits = y(n,1);
new_yLimits = y(n,2);
% If new_xLimits out of bounds, set to the new XLim
if new_xLimits - 2*Ro < xLimits(1)
    set(gca,'XLim',[new_xLimits - 5, new_xLimits + 5]);
elseif new_xLimits + 2*Ro > xLimits(2)
    set(gca,'XLim',[new_xLimits - 5, new_xLimits + 5]);
end
% If new_yLimits out of bounds, set to the new YLim
if new_yLimits - 2*Ro < yLimits(1)
    set(gca,'YLim',[new_yLimits - 4, new_yLimits + 4]);
elseif new_yLimits +2*Ro > yLimits(2)
    set(gca,'YLim',[new_yLimits - 4, new_yLimits + 4]);
end
