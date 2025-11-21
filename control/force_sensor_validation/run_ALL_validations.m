%% Run experiment and save figures for each CSV
clc; clear; close all;
addpath("validation_data\")

%% Head-on data
disp("Running head-on data")

data_raw = readmatrix("head-on-data_2023_09_11.csv");
true_direction = -2*pi;
validation_READCSV

return
%% +45 degree data
disp("Running +45 data")

data_raw = readmatrix("+45-deg-data_2023_09_11.csv");
true_direction = [-0.7071, 0.7071];
validation_READCSV

%% -45 degree data
disp("Running -45 data")

data_raw = readmatrix("-45-deg-data_2023_09_11.csv");
true_direction = [-0.7071, -0.7071];
validation_READCSV

%% Save all figures as PNG
saveFigure;
