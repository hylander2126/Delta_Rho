%% Process MoCap Five-Bar Experiment
clc; clear; close all;
% addpath('Utilities\', 'five_bar_force_experiments\exp_06-06-24_five_bar_force\');
% addpath('Utilities\', 'ICRA_2025/exp_07-31-24');
addpath('Utilities\', 'ICRA_2025/exp_08-27-24');

numPos      = 1;    % Number of CoM positions tested
numTrials   = 1;    % Number of trials for this experiment
saveVids    = 0;    % Save animation as mp4
divFactor   = 10;   % Every nth step for text and plot shapes
comPos      = 0;    % Which com position to visualize


% Load the experiment(s) or trial(s)
% data_parent_path = '06-06-24-five_bar_force_trial';
% data_parent_path = ['07-31-24_com', num2str(comPos), '_trial'];
% data_parent_path = ['08-27-24_com', num2str(comPos), '_trial'];


% TESTING if I can forgo the movie saving situation and just use the data stored in the .mat file
for i = 0:numPos-1
    for j = 1:numTrials
        data_parent_path = ['08-27-24_com', num2str(i), '_trial', num2str(j), '.mat'];
        load(data_parent_path);

        figure; grid on; axis equal;
        patch([-1 1 0], [0 0 -1.7], 'k');
        hold on;
        q1 = quiver(0,0,0,1, 'LineWidth', 2);

        % Plot the sensor data over time
        for k = 2:length(data_time)
            norm_sensor = data_sensor(k,:) / norm(data_sensor(k,:));

            set(q1, 'UData', norm_sensor(1), 'VData', norm_sensor(2));
            
            xlim([-2 2]); ylim([-2 2]);
            hold on;
            
            pause(data_time(k) - data_time(k-1));
        end
    end
end

return

%% Now save stored movies / animations
for i = 0:numPos-1
    for j= 1:numTrials
        data_parent_path = ['08-27-24_com', num2str(i), '_trial', num2str(j), '.mat'];
        load(data_parent_path);
        
        % Set up the video writer
        videoFilename = sprintf('%s.mp4', data_parent_path(1:20));
        writerObj = VideoWriter(videoFilename, 'MPEG-4'); % Create a VideoWriter object for MP4
        writerObj.FrameRate = 30;           % Set the frame rate
        
        open(writerObj);                    % Open the video file for writing

        for k = 1:length(M)                 % Assuming M is an array of frames
            % Check the size of the current frame
            frame = M(k).cdata;             % Extract the frame data
            
            % If the frame size is not 240x240, resize it
            if size(frame, 1) ~= 240 || size(frame, 2) ~= 240
                frame = imresize(frame, [240 240]);
            end

            writeVideo(writerObj, frame);    % Write each frame to the video file
        end
        close(writerObj);                   % Close the file
    end
end