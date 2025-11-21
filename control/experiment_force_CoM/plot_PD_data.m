%% Plot PD data to visualize robot response

%% Whether to plot traditional (1) or animation (0)
plot_style = 1;

fig = figure();
% grid on;
xlabel('x'); ylabel('y');
hold on;
fontsize(fig,18,"points")


closed = 0;
totalSteps = size(PD_data,1);

if plot_style
    xlabel('Time (s)'); ylabel('Angle (\circ)');
    % temp_1 = [];
    % temp_2 = [];
    % for i = 1:totalSteps
    %     sign_1 = sign(force_data(i,2)) * sign(force_data(i,3));
    %     temp_1 = [temp_1; sign_1*norm(force_data(i,2:3))];
    %     sign_2 = sign(PD_data(i,1)) * sign(PD_data(i,2));
    %     temp_2 = [temp_2; sign_2*norm(PD_data(i,1:2))];
    % end
    
    % plot(temp_1, 'b')
    % plot(temp_2, 'r')

    plot(PD_data(:,1), PD_data(:,2), 'b--', 'LineWidth',1);
    hold on;
    plot(PD_data(:,1), PD_data(:,3), 'g');
    plot(PD_data(:,1), PD_data(:,4), 'r');

    legend('Force Sensor','Control Response', 'Attitude Response')
    drawnow

    fig2 = figure();
    quiver(0, 150, Robot.d_prev(1), Robot.d_prev(2))

else
    axis ([-160 160 -160 160])
    g = quiver(0,0,0,0, 'b', 'LineWidth',1);
    h = quiver(0,0,0,0, 'r', 'LineWidth',1);
    legend('Response', 'force data')
    for i=1:size(force_data)
        if ~isgraphics(fig)
            closed = 1;
            break
        elseif i == 0 % Set to 2 to pause for recording
            input('enter')
        end
        set(h,'udata', -force_data(i,2), 'vdata', -force_data(i,3))
        set(g,'udata', PD_data(i,1), 'vdata', PD_data(i,2))
        drawnow
        title(sprintf('Time: %4.2f s', force_data(i,1)))
%         pause(0.07)
    end
end
if closed
    close all;
end