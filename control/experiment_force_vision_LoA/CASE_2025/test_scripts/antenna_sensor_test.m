clc; clear; close all;
addpath('Utilities');

sensor = ForceSensor(1);

f1 = figure(); hold on; xlim([-40, 40]); ylim([-10, 40]);
q1 = quiver(0, 0, 0, 0);
l1 = plot([-11.42 cos(2*pi/3)*25-11.42], [0 sin(2*pi/3)*25], 'r');
l2 = plot([11.42 cos(pi/3)*25+11.42], [0 sin(pi/3)*25], 'r');

alpha_dynamic = deg2rad([120; 60]);

for i=1:3
    alpha_dynamic = alpha_dynamic + [0.2; -0.2]
    sensor.alpha = alpha_dynamic;
    sensor = sensor.kinematics();
    
    F = sensor.force;
    D = sensor.direction;

    set(l1, 'XData', [-11.42 sensor.p(1,1)], 'YData', [0 sensor.p(1,2)])
    set(l2, 'XData', [11.42 sensor.p(2,1)], 'YData', [0 sensor.p(2,2)])

    temp = (sensor.p(:,1) + sensor.p(:,2))/2;
    set(q1, 'XData', temp(1), 'YData', temp(2), 'UData', D(1), 'VData', D(2))
    
    pause(0.5)
end