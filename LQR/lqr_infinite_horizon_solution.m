function [K, P] = lqr_infinite_horizon_solution(A, B, Q, R)

%% find the infinite horizon K and P through running LQR back-ups
%%   until norm(K_new - K_current, 2) <= 1e-4  

