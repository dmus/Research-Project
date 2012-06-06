%% Copyright Pieter Abbeel

%% In this exercise we study stabilization around a target trajectory in state space
%% using linear quadratic methods

clear; clc; close all;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% (a) Let's start with a linear time-varying system

% This is the system we'll work with:
T = 100;
 
% for t=1:T
% A{t} = randn(5,5);
% B{t} = randn(5,4);
% end
% save p2_a_A_B.mat A B

load p2_a_A_B.mat


nX = size(A{1},1);
nU = size(B{1},2);

load p2_a_x_ref_u_ref.mat
T = size(x_ref,2);
% x_ref, u_ref satisfy: x_ref(:,t+1) = A{t}*x_ref(:,t) + B{t}*u_ref(:,t);
% they will be our target trajectory



% Let's use the Value Iteration Solution to LQR control problems
% to solve the following optimal control problem:
% min_{x,u} \sum_{t=1}^T (x(:,t)-x_ref(:,t))'*Q*(x(:,t)-x_ref(:,t)) + (u(:,t)-u_ref(:,t))'*R*(u(:,t)-u_ref(:,t)) + (x(:,T+1)-x_ref(:,T+1))'*Qfinal*(x(:,T+1)-x_ref(:,T+1))
% s.t.      x(:,t+1) = A{t}*x(:,t) + B{t}*u(:,t) <=> x(:,t+1)-x_ref(:,t+1) = A{t}*(x(:,t)-x_ref(:,t))+B{t}*(u(:,t)-u_ref(:,t))

Q = eye(nX);
R = 0.1 * eye(nU);
Qfinal = 10*eye(nX); % making sure that at end of horizon we still push state to its target, 
                     %   rather than encountering the "end-effect" of the optimal solution 
					 %   being the one that uses very small controls at the end at the cost of not steering onto the target (0)

% Problem has been defined, let's solve it:

% LQR Value Iteration back-ups to find:
% K{k}, k=1,2,...,T : feedback control for k time steps to-go if in state x equals  u_ref + K{k}*(x-x_ref)
% P{k}, k=1,2,...,T : cost-to-go for k time steps to-go if in state x equals x'*P{k}*x

[K, P] = lqr_time_varying_controller(A, B, Q, R, Qfinal); %% YOURS to implement


% Now let's simulate and see what happens:


p2_a_w = randn(nX, T); save p2_a_w.mat
load p2_a_w.mat

% optimal T step horizon controller:
x(:,1) = zeros(nX,1);
for t=1:T
	u(:,t) = u_ref(:,t) + K{T+1-t}*(x(:,t)-x_ref(:,t));
	x(:,t+1) = A{t}*x(:,t) + B{t}*u(:,t) + p2_a_w(:,t);
end

figure; plot(x','--'); hold on; plot(x_ref');


