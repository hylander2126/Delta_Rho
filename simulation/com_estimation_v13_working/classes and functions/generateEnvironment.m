function scene = generateEnvironment(limits,~,~)

figure(); ax = axes; hold(ax,'on'); box(ax,'on');
% title('Center of Mass Estimation Simulation')
xlabel('X Position')
ylabel('Y Position')
axis(ax,'equal');

set(gcf,'Position',[100 100 100+1200 100+700]);
% set(gcf,'WindowState','maximized')
set(gca,'FontSize',20)
if(nargin > 0)
    set(ax,'xlim',limits(1:2),'ylim',limits(3:4));
    if nargin > 2
        set(gcf,'Position',[3600 290 700 500]);
    elseif nargin > 1
        set(gcf,'Position',[500 80 1000 700]);
    end
end
set(0,'DefaultLegendAutoUpdate','off')
grid(ax,'on');

scene.ax = ax;


scene.Q = [1 0.8 0.8];
scene.O = [0.9020, 0.7333, 0.9725];

scene.Aattched = [0.6549, 0.8353, 0.5412];
scene.Adetached = [1, 0.6, 0.2];
scene.Aavoid = [1, 0.6, 0.2];
scene.Fman = [0 0.7 0.4];
scene.Favoid = [1 0.4 0.4];

scene.pd = [0.4941, 0.1843, 0.5569];


