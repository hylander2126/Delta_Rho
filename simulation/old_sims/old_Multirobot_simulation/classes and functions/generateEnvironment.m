function scene = generateEnvironment(limits)

figure(); ax = axes; hold(ax,'on'); box(ax,'on');
axis(ax,'equal'); 

if(nargin > 0)
    set(ax,'xlim',limits(1:2),'ylim',limits(3:4));
end

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


