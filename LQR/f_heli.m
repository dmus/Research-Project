function x1 = f_heli(x0, u0, sim_time, model, idx, noise)


%DT = .01;
%t = 0;
%while( t < sim_time )
%    dt = min(DT, sim_time-t);
dt = sim_time;

    [Fned, Txyz] = compute_forces_and_torques(x0, u0, model, idx);


    %% add noise
    if(nargin >=6)
        Fned = Fned + noise(1:3);
        Txyz = Txyz + noise(4:6);
    end


    %% angular rate and velocity simulation:  [this ignores inertial coupling;
    %% apparently works just fine on our helicopters]

    x1(idx.ned_dot,1) = x0(idx.ned_dot) + dt * Fned / model.params.m;
    x1(idx.pqr,1) = x0(idx.pqr) + dt * Txyz ./ [model.params.Ixx; model.params.Iyy; model.params.Izz];


    %% position and orientation merely require integration (we use Euler integration):
    x1(idx.ned,1) = x0(idx.ned) + dt * x0(idx.ned_dot);
    x1(idx.axis_angle,1) = axis_angle_dynamics_update(x0(idx.axis_angle), x0(idx.pqr)*dt);

%    x0 = x1;
%    t = t+dt;
%end







function [Fned, Txyz] = compute_forces_and_torques(x0, u0, model, idx)


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




