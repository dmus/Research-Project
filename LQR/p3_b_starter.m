%% Copyright Pieter Abbeel
%%
%% Before starting this problem, you should familiarize yourself with CVX:
%%  cvxr.com/cvx/
%% The website explains how to install cvx, and has a great getting-started guide.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% (b) Now let's find a (open-loop) trajectory for a nonlinear system, the cartpole

%% let's do so by solving the following optimization problem:
%% 
%% min_{u,x}  norm(u,2) 
%% s.t.       x_{t+1} = f(x_t, u_t), t=1,2,...,T-1
%%            x_1 = x_{\rm init}
%%            x_T = x_{\rm target}
%%
%%
%% You should do so by solving the nonlinear optimization problem by
%% alternating between linearizing and solving the linearized problem.
%% You can choose to use either a shooting method or a collocation method. 
%% I decided to use collocation as it provides a natural way of
%% initializing.  

clear; clc; close all;

T = 100;
x_init = [-20; pi; 0; 0]; 
%x_init = [-10; pi; 0; 0];%-- this is what I used to generate reference trajectory for p2_b
x_target = [0;pi;0;0];
nX = 4; nU = 1;
dt = 0.1;

% manual choice of points around which I decided to linearize in first iteration
x_iter1 = zeros(nX, T);
x_iter1(1,:) = x_init(1):(x_target(1)-x_init(1))/(T-1):x_target(1);
x_iter1(2,:) = pi;
u_iter1=zeros(nU,T);

my_eps = 0.1; % for numerical linearization

% Sequential Convex Programming to find x, u:

%YOUR code here



figure(3); subplot(3,1,1); hold on; plot(u); ylabel('u');
subplot(3,1,2); hold on; plot(x(1,:)); ylabel('x'); subplot(3,1,3); hold on; plot(x(2,:));ylabel('\theta');

%x_ref=x; u_ref = u;
%save p2_b_x_ref_u_ref.mat x_ref u_ref


% let's evaluate the resulting open-loop sequence:
%   does it succeed at reaching the target? why / why not?

x(:,1) = x_init;
for t=1:T-1
	x(:,t+1) = sim_cartpole(x(:,t), u(:,t), dt);
end
figure(4); subplot(3,1,1); hold on; plot(u); ylabel('u');
subplot(3,1,2); hold on; plot(x(1,:)); ylabel('x'); subplot(3,1,3); hold on; plot(x(2,:));ylabel('\theta');


% let's see what happens with LQR around the open-loop sequence:
% (you should be able to re-use your code from p2 for this)
Q = eye(nX); R = eye(nU); Q_final = 10*Q;
[K, P] = lqr_time_varying_controller(A, B, Q, R, Q_final);% YOUR code from p2

x(:,1) = x_init;
for t=1:T-1
	u(:,t) = % YOUR CODE here 
	x(:,t+1) = sim_cartpole(x(:,t), u(:,t), dt);
end
figure(5); subplot(3,1,1); hold on; plot(u); ylabel('u');
subplot(3,1,2); hold on; plot(x(1,:)); ylabel('x'); subplot(3,1,3); hold on; plot(x(2,:));ylabel('\theta');



