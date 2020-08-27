close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('map4.txt', 0.2, 0.5, 0.25);
% plot_path(map, path);
% start = {[0.0, -4.9, 5]};
% stop = {[8.0, 19.5, 6]};
% map3
% start = {[0.0, 5, 6]};
% stop = {[20, 0, 0]};
%mymap
start = {[20, 0, 6]};
stop = {[0, 17, 5.5]};
nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Generate trajectory
disp('Generating Trajectory ...');
trajectory_generator([], [], map, path{1});

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
