%% Sub-routine to move the window if desired. Follows the object/agent 
%% trajectory by changing the axes limits.

xLimits = get(gca,'XLim');
yLimits = get(gca,'YLim');
new_xLimits = y(n,1);
new_yLimits = y(n,2);
% If new_xLimits out of bounds, set to the new XLim
if new_xLimits - Ro < xLimits(1)
    set(gca,'XLim',[new_xLimits - 0.8, new_xLimits + 0.8]); % used to be +- 5
elseif new_xLimits + Ro > xLimits(2)
    set(gca,'XLim',[new_xLimits - 0.8, new_xLimits + 0.8]);
end
% If new_yLimits out of bounds, set to the new YLim
if new_yLimits - Ro < yLimits(1)
    set(gca,'YLim',[new_yLimits - 0.8, new_yLimits + 0.8]); % Used to be +- 4
elseif new_yLimits + Ro > yLimits(2)
    set(gca,'YLim',[new_yLimits - 0.8, new_yLimits + 0.8]);
end
