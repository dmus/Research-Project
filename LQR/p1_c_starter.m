%% Copyright Pieter Abbeel

%% In this exercise we study stabilization around a fixed point using
%% linear quadratic methods

clear; clc; close all;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% (c, optional/extra credit) Now let's consider controlling a helicopter in hover

clear; clc; close all;

f = @sim_heli; % look at sim_heli.m to understand the state space and dynamics better

dt = 0.1; % we work with discrete time


% First step: find the fixed point for hover at North, East, Down
% coordinates (0, 0, 0), facing North;  
% note that b/c of tail rotor sideways thrust it's
% not the level orientation!  It's the combination of gravity, main rotor
% thrust, and tail rotor thrust that need to add up to a force of (0,0,0);
% Hint: 
% First find the roll angle (i.e, axis angle of the form (alpha, 0, 0)) such that 
%  sideways thrust == gravity component in sideways direction in helicopter
%  frame.
% Then find the amount of vertical thrust (in helicopter frame) needed to
%  compensate for gravity component aligned with helicopter's vertical axis.


x_ref = [0;0;0; 0;0;0; 0; 0; 0; asin(3.0/(5*9.81)) ;0;0];
u_ref = [0;0;0; 9.81*5*cos(x_ref(10))/137.5];


% Second step: linearize dynamics, you can re-use your existing numerical
% linearization code for this:

my_eps = 0.1; % finite difference for numerical differentiation
[A, B, c] = linearize_dynamics(f, x_ref, u_ref, dt, my_eps); %YOURS, you did this for (b)

% meaning: x(:,t+1) - x_ref  approximately = A*( x(:,t)-x_ref ) + B* ( u(:,t) - u_ref ) + c
%  if we pick equilibrium x_ref and u_ref, then c == 0 --- check this is
%  true! otherwise you did not pick your fixed point correctly!


% Third step: now let's find the infinite horizon controller
% for the linearized dynamics of our helicopter around hover:

Q = eye(12); R = eye(4); 
[K_inf, P_inf] = lqr_infinite_horizon_solution(A, B, Q, R); %YOURS, you did this for (a) and (b)



% Fourth step: now let's simulate and see what happens for a few different starting states:

T = 200;

% heli_starting_states = randn(12,4)*0.1; save p1_c_heli_starting_states.mat heli_starting_states
load p1_c_heli_starting_states.mat
% p1_c_w = randn(12, T)*0.01; save p1_c_w.mat p1_c_w
load p1_c_w.mat

n_starting_states = size(heli_starting_states,2);


for s=1:n_starting_states
	x(:,1) = heli_starting_states(:,s);
	for t=1:T
		u(:,t) = %% YOURS to fill in
		x(:,t+1) = f(x(:,t), u(:,t), dt) + p1_c_w(:,t);
	end
		
	% plotting results
	figure;
	subplot(5, 1, 1); plot(x(1:3,:)'); ylabel('ndot, edot, ddot');
	subplot(5, 1, 2); plot(x(4:6,:)'); ylabel('n, e, d');
	subplot(5, 1, 3); plot(x(7:9,:)'); ylabel('p, q, r');
	subplot(5, 1, 4); plot(x(10:12,:)'); ylabel('axis angle rotation x, y, z');
	subplot(5, 1, 5); plot(u'); ylabel('control inputs: roll, pitch, yaw, collective');
end



% If the helicopter stabilizes to around the reference hover state (not exactly, b/c of noise moving it around)
% for all initial conditions, you successfully designed a helicopter hover controller, nicely done!





