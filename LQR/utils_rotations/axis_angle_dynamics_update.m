function axis_angle1 = axis_angle_dynamics_update(axis_angle0, pqr_times_dt)

q0 = quaternion_from_axis_rotation(axis_angle0);
q1 = quat_multiply(q0, quaternion_from_axis_rotation(pqr_times_dt));
axis_angle1 = axis_rotation_from_quaternion(q1);

