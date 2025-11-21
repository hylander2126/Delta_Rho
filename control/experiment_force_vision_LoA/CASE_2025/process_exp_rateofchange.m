%% Process MoCap Five-Bar Experiment
clc; close all; clear;

% DATE = '08-27-24';
DATE = '09-10-24';

addpath('Utilities\', ['ICRA_2025/exp_', DATE]);

numPos      = 3;    % Number of CoM positions tested
numTrials   = 10;    % Number of trials for this experiment
divFactor   = 10;   % Every nth step for text and plot shapes

PLOTSERIES  = 1;    % Whether to plot time series of sensor vs payload orientation
PLOTFOOT    = 1;    % Whether to plot combined footprint plot

c           = mat2cell(lines(numTrials), ones(1, numTrials), 3); % Colormap

box_exp         = '';  % _box % % Box payload or not



%% Now AVERAGE each of the runs for each CoM Position
seriesFig = figure; grid;

for i = 0:numPos-1
    times={}; robot_poses={}; payload_poses={}; sensor_forces={}; sensor_angles={}; payload_angles={};
    % figure; grid;
    for j = 1:numTrials
        data_parent_path = [DATE, '_com', num2str(i), '_trial', num2str(j), box_exp];
        load([data_parent_path, '.mat']);

        ang_vel = diff(data_payload_pose(:,3))


    end
end