%% Copyright Pieter Abbeel
%%
%% Let's do some motion planning.
%%


%% (a) Our robot has just two joints, and there are no obstacles.
%% For this case you can easily solve in closed form for a path,
%% but as we are learning about RRT's (which we'll need in more complicated situations)
%% let's implement an RRT for finding a path from start to within radius my_eps
%% of the goal.   



clear; clc; close all;

% joint limits:
joint1_range = [0 pi];
joint2_range = [-pi pi];


x_goal = [pi/4; 0];

my_eps = 0.1; % if norm(x_goal - x,2) <= my_eps, then x to be considered success


x_init = [0;0];

DELTA = 0.05; % as in code posted for motion planning lecture, let's only allow motions up to length DELTA in each step
rrt.states = x_init;
rrt.parents= [0];

figure; hold on; axis([0 pi -pi pi]); plot(x_init(1), x_init(2), 'xr', 'markersize', 20); plot(x_goal(1), x_goal(2), 'xg', 'markersize', 20);
K=500; %max number of times we try to extend the RRT, we break out early once we found a path to the goal

% YOUR code here ; implement RRT


% link lengths:
l1=1; l2=0.5;

% YOUR code here:
%  - visualize your RRT, highlight start, goal, solution path
%  - visualize your 2 link robot's path by showing all intermediate poses
%  of the robot

