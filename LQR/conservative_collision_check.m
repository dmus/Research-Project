function collision_flag = conservative_collision_check(x, l1, l2, obstacle_x_range, obstacle_y_range, DELTA)

% x = theta1, theta2
% l1, l2 = link lengths
% obstacle is an axis-aligned rectangle in workspace
% DELTA: maximum motion of x in one move


% we'll check conservatively and make sure the obstacle inflated by
% DELTA*(l1+l2)*2 does not collide with any point on robot where we check discrete points DELTA*(l1+l2) apart, then it means the previous entire
% motion was obstace-free as each point could have moved at most DELTA*(l1+l2)  (I believe)


discretization = DELTA*(l1+l2)/4; % the /4 is a fudge factor that seems to make it work better, I guess it makes it less conservative, if even still conservative
inflation = discretization*2;

end_l1_x = l1*cos(x(1));
end_l1_y = l1*sin(x(1));
end_l2_x = l1*cos(x(1))+l2*cos(x(1)+x(2));
end_l2_y = l1*sin(x(1))+l2*sin(x(1)+x(2));

points_to_check = [];

for delta = 0:discretization:l1
	points_to_check = [points_to_check [0;0]+delta/l1*[end_l1_x;end_l1_y] ];
end
for delta = 0:discretization:l2
	points_to_check = [points_to_check [end_l1_x;end_l1_y]+delta/l2*[end_l2_x-end_l1_x;end_l2_y-end_l1_y] ];
end

	
collision_flag = 0;

for i=1:size(points_to_check,2)
	
	if(points_to_check(1,i) > obstacle_x_range(1) - inflation && points_to_check(1,i) < obstacle_x_range(2) + inflation ...
			&& points_to_check(2,i) > obstacle_y_range(1) - inflation && points_to_check(2,i) < obstacle_y_range(2) + inflation)
		collision_flag = 1;
		return;
	end
end





