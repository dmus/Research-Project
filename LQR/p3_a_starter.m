%% Copyright Pieter Abbeel
%%
%% Before starting this problem, you should familiarize yourself with CVX:
%%  cvxr.com/cvx/
%% The website explains how to install cvx, and has a great getting-started guide.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% (a) Let's find a (open-loop) trajectory for a linear system:


% This can be done in one optimization, as actual system is linear
% Concretely solve the following problem:
%    min_{u,x}  (\sum_{t=1}^{T-1} norm(x_t - x_{{\rm target}, t},2) + norm(u_t, 2) + norm(u_{t+1}-u_t, 2) ) + norm(x_T - x_{{\rm target}, T}, 2)
%    s.t.       x_{t+1} = A{t} x_t + B{t} u_t,  t=1,2,...,T-1
%               x_1 = x_{\rm init}


clear; clc; close all;
load p2_a_A_B.mat

T = length(A);
nX = size(A{1},1);
nU = size(B{1},2);

%   If you use the following x_init and x_target, 
% then the solution to this is the reference trajectory you loaded in for p2_a.
% This should allow you to verify correctness.
%
%x_init = [    1.8713    0.1100   -0.4113    0.5112   -1.1991 ]';
%x_target = [repmat(zeros(nX,1), 1, floor(T/5)) repmat(10*ones(nX,1), 1,floor(T/5)) repmat(-10*ones(nX,1), 1, floor(T/5)) repmat(10*ones(nX,1), 1, floor(T/5)) repmat(zeros(nX,1), 1, floor(T/5))];


% Now to report on in your problem set write-up, solve the following
% (similar problem):
% 

x_init = [ 1 1 1 1 1 ]';
x_target = [repmat(zeros(nX,1), 1, floor(T/5)) repmat(-10*ones(nX,1), 1, floor(T/5)) repmat(10*ones(nX,1), 1, floor(T/5)) repmat(-10*ones(nX,1), 1, floor(T/5)) repmat(zeros(nX,1), 1, floor(T/5))];


% solve convex problem
cvx_begin

%YOUR code here

cvx_end

figure; plot(x_cvx'); 

% x_ref=x_cvx;
% u_ref = u_cvx;
% save p2_a_x_ref_u_ref.mat x_ref u_ref

% Note the solution is not all that close to the target, b/c the target is
% not dynamically feasible; the solution is just the best dynamically
% feasible trajectory under the given objective function and dynamics.


