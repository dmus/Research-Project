function [K, P] = createTimeVaryingController(A, B, Q, R, Qfinal)
%CREATETIMEVARYINGCONTROLLER Summary of this function goes here
%   Detailed explanation goes here

% Now let's use the Value Iteration Solution to LQR control problems
% to solve the following optimal control problem:
% min_{x,u} \sum_{t=1}^T x(:,t)'*Q*x(:,t) + u(:,t)'*R*u(:,t) + x(:,T+1)'*Qfinal*x(:,T+1)
% s.t.      x(:,t+1) = A{t}*x(:,t) + B{t}*u(:,t)

% Problem has been defined, let's solve it:

% LQR Value Iteration back-ups to find:
% K{k}, k=1,2,...,T : feedback control for k time steps to-go if in state x equals K{k}*x
% P{k}, k=1,2,...,T : cost-to-go for k time steps to-go if in state x equals x'*P{k}*x

P_current = Qfinal;
t = length(A);
for k = 1:length(A)
    K_new = -pinv(R{t} + B{t}' * P_current * B{t}) * B{t}' * P_current * A{t};
    P_new = Q{t} + K_new' * R{t} * K_new + (A{t} + B{t} * K_new)' * P_current * (A{t} + B{t} * K_new);
    
    K_current = K_new;
    P_current = P_new;
    
    K{k} = K_current;
    P{k} = P_current;
    
    t = t-1;
end

