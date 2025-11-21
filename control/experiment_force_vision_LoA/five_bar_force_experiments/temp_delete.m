clc; clear;


%% Load 1
load(['06-12-24-five_bar_force+mocap_data_trial', num2str(1), '.mat'])

data_payload_pose = data_payload_pose(1:95,:);
data_robot_pose = data_robot_pose(1:95, :);

save('06-12-24-five_bar_force+mocap_data_trial1_mod.mat', 'data_time', 'data_payload_pose', ...
    'data_robot_pose', 'data_sensor', 'M', 'frameCount');


%% Load 2
load(['06-12-24-five_bar_force+mocap_data_trial', num2str(2), '.mat'])

data_payload_pose = data_payload_pose(1:95,:);
data_robot_pose = data_robot_pose(1:95, :);

save('06-12-24-five_bar_force+mocap_data_trial2_mod.mat', 'data_time', 'data_payload_pose', ...
    'data_robot_pose', 'data_sensor', 'M', 'frameCount');


%% Load 3
load(['06-12-24-five_bar_force+mocap_data_trial', num2str(3), '.mat'])

data_payload_pose = temp;
data_robot_pose = temp2;

save('06-12-24-five_bar_force+mocap_data_trial3_mod.mat', 'data_time', 'data_payload_pose', ...
    'data_robot_pose', 'data_sensor', 'M', 'frameCount');


%% Load 4
load(['06-12-24-five_bar_force+mocap_data_trial', num2str(4), '.mat'])

data_payload_pose = temp;
data_robot_pose = temp2;

save('06-12-24-five_bar_force+mocap_data_trial4_mod.mat', 'data_time', 'data_payload_pose', ...
    'data_robot_pose', 'data_sensor', 'M', 'frameCount');



%%


