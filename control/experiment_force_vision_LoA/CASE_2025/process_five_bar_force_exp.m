%% Process MoCap Five-Bar Experiment
clc; close all; clear;

% DATE = '08-27-24';
% DATE = '09-10-24';
% addpath('Utilities\', ['CASE_2025/exp_', DATE]);
DATE = '03-06-25';
addpath('Utilities\', ['CASE_2025/baseline_exp_', DATE]);

numPos      = 3;    % Number of CoM positions tested
numTrials   = 10;   % Number of trials for this experiment
divFactor   = 10;   % Every nth step for text and plot shapes

GETSTATS = 1; % Whether to do statistics calculations
PLOTSERIES  = 0;    % Whether to plot time series of sensor vs payload orientation
PLOTFOOT    = 0;    % Whether to plot combined footprint plot

% c           = mat2cell(lines(numTrials), ones(1, numTrials), 3); % Colormap
c = {'c', 'm', 'g'};

box_exp         = ''; % _box % % Box payload or not


% Get GLOBAL CoM Location for each position
circ_com0 = ((50/225)*[0;85]) + [0;85];
circ_com1 = ((50/225)*[85; 0]) + [0;85];
circ_com2 = ((50/225)*[-85; 0]) + [0;85];
box_com0 = ((145/220)*[0;85]) + [0;85];
box_com1 = ((145/220)*[85; 85]) + [0; 85];
box_com2 = ((145/220)*[-85; 85]) + [0;85];

%% 2025-03-06 Get accuracy of LoA estimate
if GETSTATS
final_guess = cell(numPos, numTrials);

for i = 0:numPos-1
    for j =1:numTrials
        data_parent_path = [DATE, '_com', num2str(i), '_trial', num2str(j), box_exp];
        load([data_parent_path, '.mat']);

        % This is global final push
        final_direction = diff(data_payload_pose(:,1:2));
        % Average across final few 'steps'
        final_direction = mean(final_direction(end-3:end,:))';
        % 'Unrotate' final push vector by the final payload orientation
        final_guess{i+1}(j,:) = reshape(unitize(Rz2(-(data_payload_pose(end,3)))*final_direction), [1 2]);
    end
end

% If any guesses are nan (missing mocap data point) replace with zero
for i=1:3
    for j=1:length(final_guess{i})
        if isnan(final_guess{i}(j,:))
                final_guess{i}(j,:) = final_guess{i}(j-1,:); %[0,0];
        end
    end
end

%% Now get angular error and plot each shape
if strcmp(box_exp, '_box')
    ground_truth = {unitize(box_com0), unitize(box_com1), unitize(box_com2)};
    x = [1 1 -1 -1 1]; y = [0 2 2 0 0];  
else
    ground_truth = {unitize(circ_com0), unitize(circ_com1), unitize(circ_com2)};
    x = 1*cos(linspace(0,2*pi)); y = 1*sin(linspace(0,2*pi))+1;
end

final_estimate = {mean(final_guess{1}), mean(final_guess{2}), mean(final_guess{3})};

% Print ground truths
for i=1:3
    fprintf("\n Average error for trial %i: %.2f\n", i, rad2deg(getAngle2(ground_truth{i}, final_estimate{i})));
    fprintf("ground truth: %.2f , %.2f \n estimate: %.2f , %.2f \n", ground_truth{i}(1), ground_truth{i}(2), ...
        final_estimate{i}(1), final_estimate{i}(2))
end

% Plot each shape and guess
f2 = figure("Position",[600 300 800 500]); hold on;
plot(x,y, 'b')
hold on; axis equal;
xlim([-1.25 1.25]); ylim([-.25 2.25])
title("LoA Estimate vs Ground Truth")



% Process errors and plot
for pos = 1:3
    % Calc errors per trial
    trial_errors = zeros(numTrials, 1);
    for t = 1:10
        trial_errors(t) = rad2deg(getAngle2(ground_truth{pos}, final_guess{pos}(t,:)));
        
        % quiver(0,-1, final_guess{pos}(t,1), final_guess{pos}(t,2), 'Color', c{pos})
        % pause(0.2)
    end

    CI95 = (1.96*std(trial_errors,'omitmissing') / sqrt(numTrials));
    
    % Store statistics
    angular_errors{pos} = trial_errors;
    fprintf("\nMean | StDv | 95 CI | for pos %i:\n", pos);
    fprintf("%.2f | %.2f | %.2f |\n", [mean(trial_errors, 'omitmissing'), std(trial_errors,'omitmissing'), CI95])
    
    % Plot final guesses
    final_guess_avg = unitize(mean(final_guess{pos}));

    quiver(0,0, final_guess_avg(1), final_guess_avg(2), 'Color', c{pos})
    quiver(0,0, 2*ground_truth{pos}(1), 2*ground_truth{pos}(2), 'Color', c{pos}, 'LineWidth',2, 'LineStyle','--')
end

% legend('payload','guess 0','CoM 0','guess 1','CoM 1','guess 2','CoM 2');
legend('','Estimate','Ground Truth')

end

return

%% Now AVERAGE each of the runs for each CoM Position
seriesFig = figure; grid;
for i = 0:numPos-1
    times={}; robot_poses={}; payload_poses={}; sensor_forces={}; sensor_angles={}; payload_angles={};
    % figure; grid;
    for j = 1:numTrials
        data_parent_path = [DATE, '_com', num2str(i), '_trial', num2str(j), box_exp];
        load([data_parent_path, '.mat']);

        %% FIRST clean and average the data as needed

        % Get index of 'flat spot' at the start of sensor data
        flatIdx     = 5;
        for k = 5:length(data_time)
            if data_sensor(k,:) == data_sensor(5,:)
                flatIdx = flatIdx + 1;
            end
        end
        data_sensor(1:flatIdx,:) = zeros([flatIdx, 2]);
        data_robot_pose(1:flatIdx, 1:2) = zeros([flatIdx, 2]);

        % Get the velocity (push) vector of the robot
        push_global = [0 0; diff(data_robot_pose(:,1:2))];
        push_local  = zeros(size(push_global));
        for k = 1:length(data_time)
            push_local(k,:) = reshape(Rz2(-data_robot_pose(k,3)) * push_global(k,:)', [1 2]);
        end

        % Get our desired data for plotting
        sensor_angle = [];
        for k = 1:length(data_time)
            if i==0 && ~strcmp(box_exp, '_box')
                tempt = Rz2(deg2rad(-30))*(-data_sensor(k,:)');
            else
                tempt = -data_sensor(k,:);
            end
            sensor_angle(k) = getAngle2(tempt, [0 1]); % [0; -1]);
            push_angle(k)   = getAngle2(push_local(k,:),  [0 1]); % [0; -1]);
        end
        payload_angle = (data_payload_pose(:,3) - data_robot_pose(:,3))';

        % Assign Cells for access when plotting
        sensor_angles{j} = sensor_angle*180/pi;
        payload_angles{j} = payload_angle*180/pi;
            % Also apply smoothing to this diff 
        push_angles{j} = sgolayfilt(push_angle*180/pi, 3, 11);

        times{j} = data_time;
        % robot_poses{j} = data_robot_pose;
        % figure; plot(data_time, data_payload_pose(:,3)); 
        payload_angles_global{j} = (180/pi)*data_payload_pose(:,3)';
        % sensor_forces{j} = data_sensor;
    end

    % Pad Arrays with 0 to match longest array
    [maxLength, cellIdxOfLongestTime] = max(cellfun(@length, sensor_angles));
    padForce = cellfun(@(x) [x, NaN(1, maxLength-length(x))], sensor_angles, 'UniformOutput', false);
    avgForce = mean(vertcat(padForce{:}), 1, 'omitnan')';
    padPayl  = cellfun(@(x) [x, NaN(1, maxLength-length(x))], payload_angles, 'UniformOutput', false);
    avgPayl  = mean(vertcat(padPayl{:}), 1, 'omitnan')';
    padPush  = cellfun(@(x) [x, NaN(1, maxLength-length(x))], push_angles, 'UniformOutput', false);
    avgPush  = mean(vertcat(padPush{:}), 1, 'omitnan')';
    
    if (i == 1) && ~strcmp(box_exp, '_box')
        avgForce = -avgForce;
    end

    %% Now that data is cleaned and averaged, plot it!
    if strcmp(box_exp, '_box')
        time_thresh = 9.2;
    else
        time_thresh = 7.2;
    end
    avgTime = linspace(0, times{cellIdxOfLongestTime}(end), maxLength)';
    
    % Trim the end because average includes the single long trial and throws off results
    valid_indices = avgTime <= time_thresh;
    plotT = avgTime(valid_indices);
    plotF = avgForce(valid_indices);
    plotP = avgPayl(valid_indices);
    plotR = -avgPush(valid_indices); % MUST NEGATE due to opposite-direction assignment previously
    
    % if i==0
    %     plotF = 0.5*randn(1,length(plotF));
    % end
    if PLOTSERIES
        h(1) = plot(plotT, plotF , 'Color', c{i+1}, 'LineStyle', '-',  'LineWidth', 2); hold on; % Plot sensor angle
        h(2) = plot(plotT, plotP,  'Color', c{i+1}, 'LineStyle', '--', 'LineWidth', 2); % Plot payload angle
        h(3) = plot(plotT, plotR,  'Color', c{i+1}, 'LineStyle', '-.',  'LineWidth', 2); % Plot push angle 
        % h(3) = plot(avgTime(avgTime<=time_thresh), (avgForce - avgPush)*180/pi,  'Color', c{i+1}, 'LineStyle', ':',  'LineWidth', 2.5); % Plot robot pushes
    end
    % Also save payload angular vel
    padPaylGlobal{i+1}  = payload_angles_global;
    avgTimeAngVel{i+1} = avgTime;
end

if PLOTSERIES
    % Tidy up the plot(s)
    % title(['Payload \theta_z vs Force Response \theta_z CoM Position ', num2str(i)]);
    ax = gca;
    xhand = xlabel('time (s)'); 
    yhand = ylabel('+Y Axis Deviation (deg)');
    leg = legend('','','','Force Vector', 'Payload Orientation', 'Push Vector');%, 'Angular Error'); %, 'Robot Push Angle'); % 'Force Angle', 'Payload Orientation Trial 1');
    xlim(ax,[-0.1, inf]);
    ylim(ax, [-55, 55]);
    
    set(xhand, 'FontSize', 20); set(yhand, 'FontSize', 20);
    set(seriesFig, 'Position', [300 300 1100 1000])
    set(leg, 'FontSize', 20);
    ax.FontSize = 20;
end


%% ===============================================================================
%% Now plot the angular velocity of the payload over time -> it should be brought to zero (global frame)
omegaFig = figure; grid on;
for i=1:numPos
    c = {'c', 'm', 'g'};
    R2 = [];
    RMSE = [];
    STD = [];
    STD_resid = [];
    for j = 1:numTrials
        data = padPaylGlobal{i}{j};

        % Focus on the latter part of the trajectory
        latterPart = round(0.5 * length(data)):length(data); % Adjust this to focus on different parts
        observed = data(latterPart);

        % Corresponding time
        timeLatter = avgTimeAngVel{i}(latterPart)';

        % Fit a straight line for R2
        p = polyfit(timeLatter, observed, 1);
        theta_pred = polyval(p, timeLatter);
        predicted = theta_pred;

        % Get R2
        SS_res = sum((observed - predicted).^2);
        SS_tot = sum((observed - mean(observed)).^2);
        R2(i,j) = 1 - (SS_res / SS_tot);

        % Get RMSE
        RMSE(i,j) = sqrt(mean(observed - predicted).^2);

        % Get STD
        STD(i,j) = std(observed);
        resid = observed - predicted;
        STD_resid(i,j) = std(resid);

        % Get MAE
        MAE(i,j) = mean(abs(observed - predicted));
        
        plot(timeLatter, predicted, 'Color', c{i}, 'LineWidth', 2, 'LineStyle', '--');
        hold on;
        plot(timeLatter, observed, 'Color', c{i}, 'LineWidth', 2);
    end

    disp('R2:')
    mean(R2(i,:))
    % disp('RMSE:')
    % mean(RMSE(i,:))
    disp('std:')
    mean(STD(i,:))
    disp('std resid:')
    mean(STD_resid(i,:))
    disp('MAE:')
    mean(MAE(i,:))
end

set(omegaFig, 'Position', [300 300 1100 1000])
ax = gca;
xhand = xlabel('time (s)'); 
yhand = ylabel('Global Payload Orientation (deg)');
leg = legend('','','','','Linearization', 'Payload Orientation');

set(xhand, 'FontSize', 20); set(yhand, 'FontSize', 20);
set(seriesFig, 'Position', [300 300 1100 1000])
set(leg, 'FontSize', 20);
ax.FontSize = 20;

xlim(ax,[2.8, 11]);
ylim(ax,[-120; 80]);

%% ===============================================================================
%% Now plot all of the CoM footprints on single plot
if PLOTFOOT
colors = {'c', 'm', 'g'};   % Define a cell array of colors for each footprint (cyan, magenta, yellow)
footFig = figure; grid on; hold on;   % Create a single figure for all footprints
horzAlgn = {'center', 'right', 'left'}; % horizontal alignment for the footprint annotation
horzPos = [0.5, 0.95, 0.05];            % Horizontal position for footprint annotation

tempo = [1, 2, 0];
% Loop thru each of the CoM positions
for it = 1:numPos
    i = tempo(it);
    combinedX = [];
    combinedY = [];
    % Loop through each trial to concatenate the trajectory data points
    for j = 1:numTrials
        data_parent_path = [DATE, '_com', num2str(i), '_trial', num2str(j), box_exp];
        load([data_parent_path, '.mat']);
        % IMPORTANT: must flip these two due to mistake in Optitrack assignment
        % payload_pose = data_robot_pose;
        % robot_pose = data_payload_pose;
        payload_pose = data_payload_pose;
        robot_pose = data_robot_pose;
        % Concatenate all x and y values for this CoM Position
        combinedX = [combinedX; payload_pose(:,1)];
        combinedY = [combinedY; payload_pose(:,2)];
    end
    
    % Now SORT combined data for best-fit-line. Sort by Y since upwards traj.
    [sortedY, sortIndices] = sort(combinedY);
    sortedX = combinedX(sortIndices); % Reorder combinedX based on sorted Y indices
    combinedData = [sortedX, sortedY];

    % Remove zeros (mocap initialization period)
    nonZeroRows = ~(combinedData(:,1) == 0 & combinedData(:,2) == 0);
    nonZeroData = combinedData(nonZeroRows, :);
    
    % Clean data and get best-fit-line
    [cleanData, fitLine] = removeOutliers(nonZeroData, 5);

    % Find the start and end points based on distance from the origin
    [startY, startIdx] = min(cleanData(:,2));      % Start point is always lowest Y-value
    startPoint = [cleanData(startIdx,1), startY];

    [~, endIdx] = max(sqrt(cleanData(:,1).^2 + cleanData(:,2).^2));   % End point is the farthest from the origin
    
    % Shift the points so that they all start at the origin
    shiftedData = cleanData - startPoint;
    shiftedFitLine = fitLine - startPoint;
    % TEMP DONT SHIFT DATA, I STARTED IT ALL AT THE ORIGIN (ACTUALLY A BIT +Y FROM ORIGIN)
    % shiftedData = cleanData;
    % shiftedFitLine = fitLine;
    storedFitLine{i+1} = shiftedFitLine;

    % TEMP get the derivative of this fitLine
    dx_dt = diff(shiftedFitLine(:,1));
    dy_dt = diff(shiftedFitLine(:,2));
    vel_mag = sqrt(dx_dt.^2 + dy_dt.^2);
    slope = atan2(dy_dt, dx_dt);
    stored_dF_dt{i+1} = slope; % [dx_dt, dy_dt];

    % ---------- Get the convex hull and the area ----------
    k = convhull(shiftedData(:,1), shiftedData(:,2));                  % Get the convex hull of all points
    area = polyarea(shiftedData(k,1), shiftedData(k,2));   % Get the area of the convex hull

    % Plotting the footprint with different colors
    plot(shiftedData(:,1), shiftedData(:,2), 'o', 'MarkerFaceColor', colors{i+1});  % Plot points with specific color
    fill(shiftedData(k,1), shiftedData(k,2), colors{i+1}, 'FaceAlpha', 0.3); % Fill the convex hull with the corresponding color
    plot(shiftedData(k,1), shiftedData(k,2), '-', 'LineWidth', 2, 'Color', colors{i+1});  % Outline the convex hull

    % Plot and annotate the start/end points
    plot(shiftedData(startIdx,1), shiftedData(startIdx,2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);  % Plot start point in green
    text(shiftedData(startIdx,1), shiftedData(startIdx,2), 'Start', 'FontSize', 15, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    
    plot(shiftedData(endIdx,1), shiftedData(endIdx,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);  % Plot end point in red
    text(shiftedData(endIdx,1), shiftedData(endIdx,2), 'End', 'FontSize', 15, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');
    
    % IF I WANT THE RATE OF CHANGE OF PAYLOAD THETA, JUST GET IT FROM DIFF(THETA)
    % Annotate the footprint of this CoM position in the plot
    text(horzPos(i+1), 0.95, sprintf('Pos.%i: %.f cm^2', i, area*1e4 ), ...
        'FontSize', 15, 'FontWeight', 'bold', 'HorizontalAlignment', horzAlgn{i+1}, ...
        'Units', 'normalized', 'BackgroundColor', 'white', 'EdgeColor', 'black', 'Margin', 3);
end
for i=1:numPos
    % Plot the best-fit line in black
    plot(storedFitLine{i}(:,1), storedFitLine{i}(:,2), 'k-', 'LineWidth', 2);  % 'k-' specifies a black solid line
end
% Clean up plot
ax = gca;
xlim([-0.4 0.4]); ylim([-0.025, 0.52]);
xhand = xlabel('X Position (m)');
yhand = ylabel('Y Position (m)');

set(xhand, 'FontSize', 20);
set(yhand, 'FontSize', 20);
set(footFig, 'Position', [300 300 1100 1000])
% title('Footprints of CoM Positions');
hold off;  % Release the hold after plotting all footprints
ax.FontSize = 20;
end




%% Function to remove outliers using IQR method
function [cleanData, fitLine] = removeOutliers(data, thresh)
    % Threshold is num std. dev. to enfore for outlier removal.

    % Perform Principal Component Analysis (PCA) for best-fit and outliers
    [coeff, score, ~] = pca(data);
    p = polyfit(score(:,1), score(:,2), 2); % Quadratic fit in principal component space (1st PC vs. 2nd PC)
    t = linspace(min(score(:,1)), max(score(:,1)), 100); % Range of t values for first principal component (PC1)
    fitPC2 = polyval(p, t);                 % Calculate the quadratic fit in the principal component space
    
    % Reconstruct the quadratic fit in the original space
    fitLine = [t' fitPC2'] * coeff' + mean(data);

    % Clean the data by removing outliers using Euclidian distance from best fit line
    distances = zeros(size(data, 1), 1); % Initialize an array to store distances
    for j = 1:size(data,1)
        % For each point find the closest point on the fitLine
        distances(j) = min(sqrt((fitLine(:,1) - data(j,1)).^2 + (fitLine(:,2) - data(j,2)).^2));
    end
    
    % Set an outlier threshold, e.g., based on mean and standard deviation
    out_threshold = mean(distances) + thresh * std(distances);
    % Identify and remove outliers (those whose distance exceeds the threshold)
    is_outlier = distances > out_threshold;

    cleanData = data(~is_outlier, :);
end







%% OLD unused stuff

% OLD individual plotting
% %% Now for Payload pose and robot pose tracking
% if PLOTSERIES
% for i = 0:numPos-1
%     f(i+1) = figure; grid; hold on;
%     min_length = inf;
% 
%     for j = 1:numTrials
%         data_parent_path = [DATE, '_com', num2str(i), '_trial', num2str(j), '.mat'];
%         load(data_parent_path);
% 
%         % Get the min length for trimming
%         min_length = min(min_length, length(data_time));
% 
%         robot_pose = data_robot_pose;
% 
%         % Get the velocity (push) vector of the robot
%         push_global = [0 0; diff(robot_pose(:,1)), diff(robot_pose(:,2))];
%         push_local  = zeros(size(push_global));
%         for k = 1:length(push_global)
%             R = Rz2(-robot_pose(k,3));
%             push_local(k,:) = reshape(R*push_global(k,:)' , [1 2]);
%         end
% 
%         pushes{j} = push_local;
% 
%         % Now also get the index of the 'flat spot' at the start of the sensor data...
%         flatIdX = 5;
%         flatVal = data_sensor(5,:);
%         for k = 5:length(data_time)
%             if data_sensor(k,:) == flatVal
%                 flatIdX = flatIdX + 1
%             else
%                 break
%             end
%         end
%         flatSpotIdx{j} = flatIdX;
% 
% 
%         % Now also get the average data across this CoM position
%         sensor{j} = getAngle2(-data_sensor(k,:), [0 1]); % [0; -1]);
% 
% 
%         clear data_payload_pose data_robot_pose
%     end
% 
%     % Pad Arrays with 0 to match longest array
%     maxLen = max(cellfun(@length, sensor));
%     paddedData = cellfun(@(x) [x; NaN(maxLen - length(x), 1)], sensor, 'UniformOutput', false);
%     dataMatrix = [paddedData{:}];
%     avgSensor = mean(dataMatrix, 2, 'omitnan');
% 
%     % Now plot the trials on the same figure window
%     for j = 1:numTrials
%         data_parent_path = [DATE, '_com', num2str(i), '_trial', num2str(j), '.mat'];
%         load(data_parent_path);
% 
%         payload_pose    = data_payload_pose;
%         robot_pose      = data_robot_pose;
% 
%         % Remove sensor flat spot using flat index
%         data_sensor(1:flatSpotIdx{j},:) = zeros([flatSpotIdx{j}, 2]);
% 
%         % --------------- ENSURE SAME FRAME OF REFERENCE ----------------- 
%         % Local force sensor angle
%         sensor_angle = zeros(length(data_sensor),1);
%         push_angle   = zeros(length(pushes{j}),1);
%         for k = 1:size(data_sensor,1)
%             sensor_angle(k) = getAngle2(-data_sensor(k,:), [0 1]); % [0; -1]);
%             push_angle(k)   = getAngle2(pushes{j}(k,:),  [0 1]); % [0; -1]);
%         end
% 
%         % Shift the sensor data a little to match valleys and peaks with robot pushes
%         % [corr, lags] = xcorr(push_angle, sensor_angle);
%         % [~,I] = max(corr);
%         % shift_amount = lags(I);
%         % if shift_amount > 0
%         %     shifted_data2 = [zeros(shift_amount,1); sensor_angle(1:end-shift_amount)];
%         % elseif shift_amount < 0
%         %     shifted_data2 = [sensor_angle(-shift_amount+1:end); zeros(-shift_amount, 1)];
%         % else
%         %     shifted_data2 = sensor_angle;
%         % end
% 
% 
%         % Make payload rotation wrt local frame
%         payload_angle = payload_pose(:,3) - robot_pose(:,3);
% 
% 
%     % Plot sensor angle
%         h(1) = plot(data_time, sensor_angle*180/pi, 'LineWidth', 1, 'Color', c{j});
%     % Plot payload angle
%         h(2) = plot(data_time, payload_angle*180/pi, 'Color', c{j}, 'LineStyle', '--', 'LineWidth', 1);
%     % Plot error
%         % h(3) = plot(data_time, (sensor_angle - push_angle)*180/pi, 'LineStyle', ':', 'LineWidth', 1); %, 'Color', c{j});
%     % Plot robot heading/velocity
%         % h(4) = plot(data_time, push_angle*180/pi, 'LineStyle', '-.', 'LineWidth', 1, 'Color', c{j});
% 
%         % For text labels at the end
%         label_ypos(j) = payload_angle(min_length); % for adding text labels
% 
%         clear data_payload_pose sensor_angle push_angle
%     end
% 
%     % Add labels for each trial at x = min_length
%     for j = 1:numTrials
%         text(data_time(min_length), label_ypos(j)*180/pi, ...
%             sprintf('Trial %i', j), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', ...
%             'FontSize', 9); %, 'Color', c{j});
%     end
% 
%     % Tidy up each CoM pos plot
%     title(['Payload \theta_z vs Force Response \theta_z CoM Position ', num2str(i)]);
%     xlabel('time (s)'); 
%     ylabel('Angle wrt -y axis (deg)');
%     legend('Force Angle', 'Payload Pose'); %, 'Robot Push Angle'); % 'Force Angle', 'Payload Orientation Trial 1');
%     xlim([data_time(3) data_time(min_length)]); % Set x-lim based on min length for each plot
%     ylim([-60 60]);   % Set y-lim to -180 to 180 for now...
% end
% end


%{

%% Now let's figure out the footprint of each CoM Position
plot_individual_footprints = 0;
if plot_individual_footprints

for i=0:numPos-1
    combinedX = [];
    combinedY = [];
    for j=1:numTrials
        data_parent_path = [DATE, '_com', num2str(i), '_trial', num2str(j), '.mat'];
        load(data_parent_path);

        % IMPORTANT: must flip these two due to mistake in Optitrack assignment
        payload_pose    = data_robot_pose;
        robot_pose      = data_payload_pose;
        
        % Concatenate all x and y values for this CoM Position
        combinedX = [combinedX; payload_pose(:,1)];
        combinedY = [combinedY; payload_pose(:,2)];
    end
    
    this_com_coords = [combinedX combinedY];
    
    % Find the start and end points based on distance from the origin
    distances_from_origin = sqrt(combinedX.^2 + combinedY.^2);
    [~, startIdx] = min(distances_from_origin);  % Start point is closest to the origin
    [~, endIdx] = max(distances_from_origin);    % End point is the farthest from the origin
    
    k = convhull(combinedX, combinedY);                  % Get the convex hull of all points
    area = polyarea(this_com_coords(k,1), this_com_coords(k,2));   % Get the area of the convex hull

    fprintf('\n\nThe area of CoM %i is %.4f', [i, area])

    % Plotting the area
    figure; grid on; % Create a new figure
    xlim([-0.5, 0.5]); ylim([-0.1, 1.0])
    hold on; % Hold on to plot multiple layers
    plot(this_com_coords(:,1), this_com_coords(:,2), 'bo', 'MarkerFaceColor', 'b');  % Plot all points as blue dots
    fill(this_com_coords(k,1), this_com_coords(k,2), 'c', 'FaceAlpha', 0.3); % Fill the convex hull with a cyan color
    plot(this_com_coords(k,1), this_com_coords(k,2), 'r-', 'LineWidth', 2);  % Outline the convex hull with a red line
    
    % Annotate the start and end points
    plot(this_com_coords(startIdx,1), this_com_coords(startIdx,2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);  % Plot start point in green
    text(this_com_coords(startIdx,1), this_com_coords(startIdx,2), 'Start', 'FontSize', 12, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    
    plot(this_com_coords(endIdx,1), this_com_coords(endIdx,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);  % Plot end point in red
    text(this_com_coords(endIdx,1), this_com_coords(endIdx,2), 'End', 'FontSize', 12, 'VerticalAlignment', 'top', 'HorizontalAlignment', 'left');

    % Annotate the footprint of this CoM position
    text(0.95, 0.95, sprintf('Total Footprint: %.3f m^2', area), ...
        'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'right', ...
        'Units', 'normalized', 'BackgroundColor', 'white', 'EdgeColor', 'black', 'Margin', 3);


    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title(['Footprint CoM', num2str(i)]);
    hold off; % Release the hold
end
end




%% Error plotting
maxLength = max(cellfun(@length, e_angle));
paddedData = NaN(maxLength, 8);

for i = 1:8
    currLength = length(e_angle{i});
    paddedData(1:currLength, i) = e_angle{i};
end

boxplot(paddedData, 'Labels', {'Trial 1', 'Trial 2', 'Trial 3', 'Trial 4', 'Trial 5', 'Trial 6', 'Trial 7', 'Trial 8'})
title('Angle Error Between Force Feedback and Payload Orientation for All Trials')
ylabel('Angle (rads)')



%% Now for payload rotation vs robot's command input (which is local frame)
for i = 1:8
    load([data_parent_path, num2str(i), '.mat'])

    f2 = figure("Position",[600 300 800 500]); hold on;
    axis equal;
    
    % Plot cos and sin of payload rotation (add dimension to payload rotation)
    title(['Payload & Robot Rotation vs Robot Command - Trial ', num2str(i)])
    plot(data_time, data_payload_pose(:,3), 'b')
    plot(data_time, data_robot_pose(:,3), 'g')
    plot(data_time, data_robot_command(:,1)/20, '--r')
    xlabel 'time (s)'; ylabel 'mm'
    legend('\theta\_p', '\theta\_r', 'robot\_\theta / 20')
    hold off
    
    clearvars -except data_parent_path
    
end

%% Payload rotation vs Force sensor measurement
plotSensor = 0;
if plotSensor
    for j = 1:numTrials
        load([data_parent_path, num2str(j), '.mat'])
        
        if plotTrials
            n = size(data_sensor,1);
            cmap = jet(n);
            alphas = linspace(0.15, 1, n);
            
            figure; hold on; grid;
            xlim([-20 20]); ylim([-35 15]);
            
            for i = 1:n-1
                segColor = cmap(i, :); % interp1(1:n, cmap, i+0.5, 'linear');
                line(data_sensor(i:i+1, 1), data_sensor(i:i+1, 2), 'Color', [segColor alphas(i)], 'LineWidth', 3);
                hold on;
                s = scatter(data_sensor(i,1), data_sensor(i,2), 15, segColor, 'filled');
                s.MarkerFaceAlpha = alphas(i);
            end
            
            title(['Trial ' num2str(j)])
            co = colorbar;
            colormap(jet);
            xlabel 'x (mm)'
            ylabel 'y (mm)'
            co.Label.String = 'time (s)';
        end
        
        clearvars -except data_parent_path plotTrials
    end
end

%}