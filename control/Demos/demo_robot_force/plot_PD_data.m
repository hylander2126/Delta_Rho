%% Plot PD data to visualize robot response

fig = figure();
grid on;
xlabel('x'); ylabel('y');
hold on;
% axis([-0.5 0.5 -0.5 0.5]);
% axis equal;


temp_1 = [];
temp_2 = [];
for i = 1:length(force_data)
    sign_1 = sign(force_data(i,2)) * sign(force_data(i,3));
    temp_1 = [temp_1; sign_1*norm(force_data(i,2:3))];
    sign_2 = sign(PD_data(i,1)) * sign(PD_data(i,2));
    temp_2 = [temp_2; sign_2*norm(PD_data(i,1:2))];
end

plot(temp_1, 'b')
plot(temp_2, 'r')
legend('force data','Response')

% h = [];
% for i=1:size(force_data)
%     h = quiver(0,0,force_data(i,2),force_data(i,3),'r');
%     drawnow
%     title(sprintf('Time: %4.2f s', force_data(i,1)))
%     if i ==1
%         input('enter')
%     end
%     pause(0.2)
%     delete(h)
% end