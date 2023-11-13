% Plot the obstacles:
for j=1:length(Q)
    Q(j).plot(scene.ax);
end

% Plot the object:
O.plot(scene.ax);

% Plot the agent(s):
for i=1:N
    A(i).plot(scene.ax);
end

% Initialize handle for CoM location point
c = scatter(0,0);
