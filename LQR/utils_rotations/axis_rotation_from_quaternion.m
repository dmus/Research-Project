function a = axis_rotation_from_quaternion(q)

rotation_angle = asin(norm(q(1:3),2)) * 2;

my_eps = 1e-6;
if(rotation_angle < my_eps)
	a = zeros(3,1);
else
	a = q(1:3)/norm(q(1:3)) * rotation_angle;
end
