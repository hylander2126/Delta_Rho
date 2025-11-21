%% Function to generate a 'scene' - a plot with axes limits [xmin, xmax, ymin, ymax]
function scene = generateEnvironment(limits,opts)
% Check if optional args exists
if ~exist('opts','var')
    opts = '';
end

% If dual monitor is desired, set figure position to 2nd monitor (@ work)
if contains(opts, 'dual')
    figure(Position=[800,377,900,550]);
%     figure(Position=[1700, 177, 800, 600]);
elseif contains(opts, 'big')
    figure(Position=[400,100, 800,600])
else
    figure
end
ax = axes; hold(ax,'on'); box(ax,'on');
axis(ax,'equal');

% title('Center of Mass Estimation'); % | Elapsed Time: ' num2str(round(toc,2)) 's'])
xlabel('X Position (m)')
ylabel('Y Position (m)')

set(gca,'FontSize',20)
set(gcf,'Color','w')

if(nargin > 0)
    set(ax,'xlim',limits(1:2),'ylim',limits(3:4));
end

% grid(ax,'on');
scene.ax = ax;


scene.Q = [1 0.8 0.8];
scene.O = [0.9020, 0.7333, 0.9725];

scene.Aattched = [0.6549, 0.8353, 0.5412];
scene.Adetached = [1, 0.6, 0.2];
scene.Aavoid = [1, 0.6, 0.2];
scene.Fman = [0 0.7 0.4];
scene.Favoid = [1 0.4 0.4];

scene.pd = [0.4941, 0.1843, 0.5569];


