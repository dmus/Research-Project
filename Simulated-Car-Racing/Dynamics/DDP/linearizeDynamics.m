function [A, B, c] = linearizeDynamics(f, x_ref, u_ref, dt, my_eps, x_ref_tplus1)

% meaning: x(:,t+1) - x_ref_tplus1  to-first-order-equal-to A*( x(:,t)-x_ref ) + B* ( u(:,t) - u_ref ) + c
%  if we pick an equilibrium x_ref and u_ref, then x_ref = f(x_ref, u_ref),
%  and c == 0, and x_ref_tplus1 = x_ref and per check below does not need
%  to be passed in; for question 1 this is the case, but for later
%  questions you'll also need x_ref_tplus1 (when trajectory stabilizing
%  with LQ control)

if(nargin == 6)
	x_ref_next = x_ref_tplus1;
else
	x_ref_next = x_ref;
end

x_eps = my_eps;
u_eps = my_eps;

for i = 1:length(x_ref)
	x_plus = x_ref_next;
	x_plus(i) = x_plus(i) + x_eps;
	x1_plus = f(x_plus, u_ref, dt);
	x_minus = x_ref_next;
	x_minus(i) = x_minus(i) - x_eps;
	x1_minus = f(x_minus, u_ref, dt);
	A(:,i) = (x1_plus - x1_minus) / (2*x_eps);
end

for i = 1:length(u_ref)
	u_plus = u_ref;
	u_plus(i) = u_plus(i) + u_eps;
	x1_plus = f(x_ref, u_plus, dt);
	u_minus = u_ref;
	u_minus(i) = u_minus(i) - u_eps;
	x1_minus = f(x_ref, u_minus, dt);
	B(:,i) = (x1_plus - x1_minus) / (2*u_eps);
end

c = f(x_ref,u_ref,dt) - x_ref_next;