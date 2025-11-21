%% Plot force data to visualize incoming force vector

fig = figure();
grid on;
xlabel('x'); ylabel('y');
hold on;
axis([-0.5 0.5 -0.5 0.5]);
axis equal;

h = [];
for i=1:size(force_data)
    h = quiver(0,0,force_data(i,2),force_data(i,3),'r');
    drawnow
    title(sprintf('Time: %4.2f s', force_data(i,1)))
    if i ==1
        input('enter')
    end
    pause(0.2)
    delete(h)
end