%% Copyright Pieter Abbeel
%%
%% Let's do some motion planning.
%%



%% (b) Now we have an obstacle, let's make sure this still works. 
%% You can simply deal with the obstacle by only checking for it after you 
%% generated the extension to the RRT and then simply discarding extensions that run
%% into the obstacle.  If you use a different method of dealing with the
%% obstacle, describe what you did.
%%

clear; clc; close all;

% joint limits:
joint1_range = [0 pi];
joint2_range = [-pi pi];

% obstacle is an axis-aligned rectangle in workspace:
obstacle_x_range = [1 1.5];
obstacle_y_range = [0.7 0.75];

workspace_goal = [1.2; 0.9];

my_eps = 0.1; % if norm(workspace_goal - workspace(x),2) <= my_eps, then x to be considered success

l1=1; l2=1;

x_init = [0;0];

DELTA = 0.05; % as in code posted for motion planning lecture, let's only allow motions up to length DELTA in each step

% YOUR code for RRT here

% you might want to use the following function:
% conservative_collision_check(x_new, l1, l2, obstacle_x_range,obstacle_y_range, DELTA)


% YOUR code here:
%  visualize in a meaningful way
%   - RRT itself
%   - sequence of poses of robot and the obstacle












%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Congratulations!!!!!
%%
%% You have by now gained a solid understanding of particle filters, Kalman filters, EM, 
%% optimal control, and motion planning.  And ... you are done with problem sets!!!
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%