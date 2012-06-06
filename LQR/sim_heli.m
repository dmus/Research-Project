function x1 = sim_heli(x0, u0, total_dt)

addpath utils_rotations;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% set up idx, model params, model features  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k=1;
idx.ned_dot = k:k+2; k=k+3;
idx.ned = k:k+2; k=k+3; %North, East, Down
idx.pqr = k:k+2; k=k+3; % angular rate around x, y, z of helicopter
idx.axis_angle = k:k+2; k=k+3;


%% [[tic toc + sweeps data model (20070927)]]

% mass and inertia
model.params.m = 5; % kg
model.params.Ixx = .3; % assuming diagonal inertia matrix
model.params.Iyy = .3;
model.params.Izz = .3;

% aerodynamic forces parameters
model.params.Tx = [0   -1.0410    3.9600]';
model.params.Ty = [0   -0.9180   -2.7630]';
model.params.Tz = [0   -0.7740    4.4520]';
model.params.Fx = [-0.2400]';
model.params.Fy = [-3.0   -0.6000]';
model.params.Fz = [0   -0.0025 -137.5000]';

DT = .05;
t = 0;
while( t < total_dt )
	dt = min(DT, total_dt-t);
	
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% compute forces and torques                %%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
	
	[Fned, Txyz] = compute_forces_and_torques(x0, u0, model, idx);
	
	
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% forward integrate state                   %%
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
	
	%% angular rate and velocity simulation:  [this ignores inertial coupling;
	%% apparently works just fine on our helicopters]
	
	x1(idx.ned_dot,1) = x0(idx.ned_dot) + dt * Fned / model.params.m;
	x1(idx.pqr,1) = x0(idx.pqr) + dt * Txyz ./ [model.params.Ixx; model.params.Iyy; model.params.Izz];
	
	
	%% position and orientation merely require integration (we use Euler integration):
	x1(idx.ned,1) = x0(idx.ned) + dt * x0(idx.ned_dot);
	x1(idx.axis_angle,1) = axis_angle_dynamics_update(x0(idx.axis_angle), x0(idx.pqr)*dt);

	x0 = x1;
	t = t+dt;
end








function [Fned, Txyz] = compute_forces_and_torques(x0, u0, model, idx)


% compute helicopter velocity in its own frame (it experiences drag forces
% in its own frame)

uvw = express_vector_in_quat_frame(x0(idx.ned_dot), quaternion_from_axis_rotation(x0(idx.axis_angle)));


%% aerodynamic forces 

% expressed in heli frame:
Fxyz_minus_g(1,1) = model.params.Fx' * uvw(1);
Fxyz_minus_g(2,1) = model.params.Fy' * [1; uvw(2)];
Fxyz_minus_g(3,1) = model.params.Fz' * [1; uvw(3); u0(4)];

% expressed in ned frame
F_ned_minus_g = rotate_vector(Fxyz_minus_g, quaternion_from_axis_rotation(x0(idx.axis_angle)));

% add gravity to complete the forces:
Fned = F_ned_minus_g + model.params.m * [0;0;9.81];

%% torques
Txyz(1,1) = model.params.Tx' * [1; x0(idx.pqr(1)); u0(1)];
Txyz(2,1) = model.params.Ty' * [1; x0(idx.pqr(2)); u0(2)];
Txyz(3,1) = model.params.Tz' * [1; x0(idx.pqr(3)); u0(3)];




