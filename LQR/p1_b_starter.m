%% Copyright Pieter Abbeel

%% In this exercise we study stabilization around a fixed point using
%% linear quadratic methods

clear; clc; close all;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% (b) Now let's consider our first nonlinear system (cartpole), linearize it around one
%% point, and design an infinite horizon controller for this linearized system
clear; clc; close all;

f = @sim_cartpole;
dt = 0.1; % we work with discrete time

% our reference point is x=0, theta=pi, xdot = 0; thetadot=0; this has the vertically pole UP
% our reference control input is 0; this is a fixed point of the system

x_ref = [0; pi; 0; 0];
u_ref = 0;

my_eps = 0.1; % finite difference for numerical differentiation

[A, B, c] = linearize_dynamics(f, x_ref, u_ref, dt, my_eps); % YOURS to implement

% meaning: x(:,t+1) - x_ref  approximately = A*( x(:,t)-x_ref ) + B* ( u(:,t) - u_ref ) + c
%  if we pick x_ref and u_ref to constitute a fixed point, then c == 0


% now let's find the infinite horizon controller
% for the linearized version of the cartpole balancing problem:

Q = eye(4); R = eye(1); 

[K_inf, P_inf] = lqr_infinite_horizon_solution(A, B, Q, R); %YOURS, implemented for part (a)



% now let's simulate and see what happens for a few different starting states:

starting_states = [ ...
	0           0          0            10       50
 	pi-pi/10    pi-pi/4    pi-pi/2       pi       pi
	0           0          0             0        0
	0           0          0             0        0];

n_starting_states = size(starting_states,2);

T = 500;

figure; 
for s=1:n_starting_states
	x(:,1) = starting_states(:,s);
	for t=1:T
		u(:,t) =  ( K_inf * ( x(:,t) - x_ref ) ) + u_ref;
		x(:,t+1) = f(x(:,t), u(:,t), dt);
	end
		
	% plotting results
	subplot(n_starting_states, 1, s); plot(x'); hold on; plot(u,'--'); 
end

	
% briefly discuss performance for the different initial conditions


% let's do a simulation in the presence of noise:

% p1_b_w = randn(4, T)*0.1; save p1_b_w.mat p1_b_w

load p1_b_w

figure; 
for s=1:n_starting_states
	x(:,1) = starting_states(:,s);
	for t=1:T
		u(:,t) =  ( K_inf * ( x(:,t) - x_ref ) ) + u_ref;
		x(:,t+1) = f(x(:,t), u(:,t), dt);
	end
		
	% plotting results
	subplot(n_starting_states, 1, s); plot(x'); hold on; plot(u,'--'); 
end


