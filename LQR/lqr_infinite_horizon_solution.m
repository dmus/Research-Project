function [K, P] = lqr_infinite_horizon_solution(A, B, Q, R)

%% find the infinite horizon K and P through running LQR back-ups
%%   until norm(K_new - K_current, 2) <= 1e-4  

P_current = 0;
for t = 1:10e10
    K_new = -(R + B' * P_current * B)^-1 * B' * P_current * A; 
    P_new = Q + K_new' * R * K_new + (A + B * K_new)' * P_current * (A + B * K_new);
    
    if  t > 1 && norm(K_new - K_current, 2) <= 1e-4    
        break;
    end
    
    K_current = K_new;
    P_current = P_new;
end

K = K_new;
P = P_new;