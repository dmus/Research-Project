%% Copyright Pieter Abbeel

%% In this exercise we study stabilization around a fixed point using
%% linear quadratic methods

clear; clc; close all;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% (a) Let's start with a linear system

% This is the system we'll work with:

A = [...
    0.0481   -0.5049    0.0299    2.6544    1.0608
    2.3846   -0.2312   -0.1260   -0.7945    0.5279
    1.4019   -0.6394   -0.1401    0.5484    0.1624
   -0.0254    0.4595   -0.0862    2.1750    1.1012
    0.5172    0.5060    1.6579   -0.9407   -1.4441];

B = [ ...
   -0.7789   -1.2076
    0.4299   -1.6041
    0.2006   -1.7395
    0.8302    0.2295
   -1.8465    1.2780];

nX = size(A,1);
nU = size(B,2);

% Let's verify the system is controllable. 
% To do so we need to check if rank( [B A*B A^2*B A^3*B A^4*B] ) == 5
% [generally we'd go up to A^(n-1)*B, and verify rank == n]

rank([B A*B A^2*B A^3*B A^4*B])

% The linear system x_{t+1} = A x_t + B u_t has one fixed point, x = 0, and
% it stays fixed for u = 0;  indeed 0 = A * 0 + B * 0

% Recall that we can use the Value Iteration Solution to LQR control problems
% to solve the following optimal control problem:
% min_{x,u} \sum_{t=1}^T x(:,t)'*Q*x(:,t) + u(:,t)'*R*u(:,t) + x(:,T+1)'*Qfinal*x(:,T+1)
% s.t.      x(:,t+1) = A*x(:,t) + B*u(:,t)
% 
% For T going to infinity, we'll have that the Value Function and the
% Feedback Controller at time t=0 reach a steady-state --- the optimal
% value function and feedback controller for infinitely many time-steps to-go
%
% So let's run the Value Iteration Solution to the LQR control problem
% until it has converged, and then use that infinite horizon optimal
% feedback controller to stabilize our system at 0:

Q = eye(nX);
R = eye(nU);

% Problem has been defined, let's solve it:

% LQR Value Iteration back-ups to find:
% K{k}, k=1,2,...,T : feedback control for k time steps to-go if in state x equals K{k}*x
% P{k}, k=1,2,...,T : cost-to-go for k time steps to-go if in state x equals x'*P{k}*x

[K_inf, P_inf] = lqr_infinite_horizon_solution(A, B, Q, R); % YOURS to implement

% Note you can verify your solution obtained iteratively by comparing it
% with the matlab built-in solution method dlqr; be careful though b/c
% matlab defines K as u = -K*x; and we have worked with u = K*x, so the sign is
% flipped


% Now let's simulate and see what happens for a few different starting states:

starting_states = [ ...
   -1.9613    1.9277   -0.2442 
   -1.3127   -0.2406   -0.0260 
    0.0698   -0.5860   -0.7522 
    0.0935   -0.1524   -0.9680 
    1.2494    0.5397   -0.5146 ];
n_starting_states = size(starting_states,2);

T = 100; % let's simulate for 100 steps

% no dynamics noise:

figure; 
for s=1:n_starting_states
	x(:,1) = starting_states(:,s);
	cost(s) = 0;
	for t=1:T
		u(:,t) = K_inf*x(:,t);
		x(:,t+1) = A*x(:,t) + B*u(:,t);
	end
		
	% plotting results
	subplot(n_starting_states, 1, s); plot(x');  
end

% with dynamics noise:

% p1_a_w = randn(nX, T)*0.1; save p1_a_w.mat p1_a_w
% I saved w so you have the same random noise, so I can spot-check your
% plot more easily

load p1_a_w.mat

figure; 
for s=1:n_starting_states
	x(:,1) = starting_states(:,s);
	cost(s) = 0;
	for t=1:T
		u(:,t) = K_inf*x(:,t);
		x(:,t+1) = A*x(:,t) + B*u(:,t) + p1_a_w(:,t);
	end
		
	% plotting results
	subplot(n_starting_states, 1, s); plot(x');  
end

% note how the controller successfully drives the state to zero in case of
% zero noise, and keeps it close to zero in the presence of noise





