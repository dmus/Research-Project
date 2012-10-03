function [K,P] = computePolicy(S, U, times, Map)
%COMPUTEPOLICY Summary of this function goes here
%   Detailed explanation goes here
    
    % Horizon
    H = size(S,1);

    % Parameter for finite difference methods
    my_eps = 0.1;
    
    for t = 1:H-1
    
        % Add state features
        s_ref = S(t,:)';
        s_ref_tplus1 = S(t+1,:)';
        
        % Control input and time difference
        u_ref = U(t,:)';
        dt = times(t+1) - times(t);

        % Linearize dynamics around reference trajectory
        [A{t}, B{t}, c{t}] = linearizeDynamics(@f, s_ref(1:6), u_ref(1:2), dt, my_eps, s_ref_tplus1(1:6), Map);
        
        % Transform affine system into standard LQR format
        A{t} = [A{t} c{t}; zeros(1, size(A{t},2)) 1];
        B{t} = [B{t} zeros(size(B{t},1),1); zeros(1,size(B{t},2) + 1)];

        % Approximate costs in quadratic form around reference trajectory
        [Q{t}, R{t}] = quadraticizeCosts(@g, @h, s_ref(1:13), u_ref(1:2), my_eps);

        % Extend matrices to be able to store previous state in current
        % state (and penalize for change in control inputs)
        Aprime{t} = [A{t} zeros(size(A{t}));% B{t}; 
                 eye(length(s_ref) / 2) zeros(length(s_ref) / 2)];% zeros(length(s_ref) + 1, length(u_ref) + 1);
                 %zeros(length(u_ref) + 1, (length(s_ref) + 1)*2)];% eye(length(u_ref) + 1)];
        Bprime{t} = [B{t};
                 zeros(length(s_ref) / 2, length(u_ref))]; 
                 %eye(length(u_ref) + 1)];
        Qprime{t} = Q{t};%[Q{t} zeros(length(s_refprime) + 1, length(u_ref) + 1); 
                 %zeros(length(u_ref) + 1, length(s_refprime) + 1) R{t}];
        Rprime{t} = R{t};%0.1 * eye(length(u_ref) + 1);
    end

    Qprime{H} = quadraticizeCosts(@g, @h, S(H,1:13)', U(H,1:2)', my_eps);
    [K, P] = createTimeVaryingController(Aprime, Bprime, Qprime, Rprime, Qprime{H});

end

