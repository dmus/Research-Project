function [A, B, c] = linearize_dynamics(f, x_ref, u_ref, dt, my_eps, x_ref_tplus1)

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


