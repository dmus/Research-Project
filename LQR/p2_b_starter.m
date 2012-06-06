%% Copyright Pieter Abbeel

%% In this exercise we study stabilization around a target trajectory in state space
%% using linear quadratic methods

clear; clc; close all;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% (b) Now let's stabilize a trajectory of a nonlinear system, the cartpole

clear; clc; close all;

f = @sim_cartpole;
dt = 0.1; % we work with discrete time

% our reference trajectory brings the cartpole from x = -1, to x=0, while
% keeping the pole +/- up-right throughout 
load p2_b_x_ref_u_ref.mat
T = size(x_ref,2);
nX = size(x_ref,1);
nU = size(u_ref,1);

% let's evaluate the reference open-loop sequence first:
x(:,1) = x_ref(:,1);
for t=1:T-1
	x(:,t+1) = f(x(:,t), u_ref(:,t), dt);
end
figure; subplot(3,1,1); hold on; plot(u_ref); ylabel('u');
subplot(3,1,2); hold on; plot(x(1,:)); ylabel('x'); subplot(3,1,3); hold on; plot(x(2,:));ylabel('\theta');


% let's see what happens with LQ based feedback control for trajectory
% stablization

my_eps = 0.01; % finite difference for numerical differentiation
for t=1:T-1
	[A{t}, B{t}, c{t}] = linearize_dynamics(f, x_ref(:,t), u_ref(:,t), dt, my_eps, x_ref(:,t+1));%YOURS now it includes x_ref(:,t+1) argument
end
% meaning: x(:,t+1) - x_ref(:,t+1)  approximately = A*( x(:,t)-x_ref(:,t) ) + B* ( u(:,t) - u_ref(:,t) ) + c
%  if we pick x_ref and u_ref to constitute a fixed point, then c == 0

Q = eye(nX); R = eye(nU); Q_final = 10*Q;

[K, P] = lqr_time_varying_controller(A, B, Q, R, Q_final); %YOURS, you already have this from (a)

x(:,1) = x_ref(:,1);
for t=1:T-1
	u(:,t) = u_ref(:,t) + K{T-t}*(x(:,t)-x_ref(:,t));
	x(:,t+1) = sim_cartpole(x(:,t), u(:,t), dt);
end
figure; subplot(3,1,1); hold on; plot(u); ylabel('u');
subplot(3,1,2); hold on; plot(x(1,:)); ylabel('x'); subplot(3,1,3); hold on; plot(x(2,:));ylabel('\theta');


% and with noise

%p2_b_w = randn(nX, T)*.01; save p2_b_w.mat
load p2_b_w.mat

x(:,1) = x_ref(:,1);
for t=1:T-1
	u(:,t) = u_ref(:,t) + K{T-t}*(x(:,t)-x_ref(:,t));
	x(:,t+1) = sim_cartpole(x(:,t), u(:,t), dt) + p2_b_w(:,t);
end
figure; subplot(3,1,1); hold on; plot(u); ylabel('u');
subplot(3,1,2); hold on; plot(x(1,:)); ylabel('x'); subplot(3,1,3); hold on; plot(x(2,:));ylabel('\theta');


