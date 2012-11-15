function [A,B,Q,R] = approximateLQR(S, U, times, Map, alpha, model)
%COMPUTEPOLICY Approximate a standard linear-time varying LQR problem.
%   Dynamics and cost matrices are approximated with Taylor expansions.
    
    % Horizon
    H = size(S,1);

    n_states = size(S,2);
    n_inputs = size(U,2);
    
    % Parameter for finite difference methods
    my_eps = 0.01;
    
    for t = 1:H-1
        % Add state features
        s_ref = S(t,:)';
        s_ref_tplus1 = S(t+1,:)';
        
        % Control input and time difference
        u_ref = U(t,:)';
        dt = times(t+1) - times(t);

        % Linearize dynamics around reference trajectory
        [A{t}, B{t}, c{t}] = linearizeDynamics(@f, s_ref, u_ref, dt, my_eps, s_ref_tplus1, Map, model);
        
        % Transform affine system into standard LQR format
        A{t} = [A{t} c{t}; zeros(1, n_states) 1];
        Temp = zeros(n_states + 1, n_inputs + 1);
        Temp(1:size(B{t},1),1:size(B{t},2)) = B{t};
        B{t} = Temp;
        
        % Approximate costs in quadratic form around reference trajectory
        [Q{t}, R{t}] = quadraticizeCosts(@g, @h, s_ref(1:3), u_ref(1:2), my_eps);
        
        % Penalize deviations from reference trajectory
        Q{t} = (1 - alpha) * Q{t} + alpha * [eye(3) zeros(3,1); zeros(1,3 + 1)];
        R{t} = (1 - alpha) * R{t} + alpha * [eye(2) zeros(2,1); zeros(1,2 + 1)];
        
        Aprime{t} = [A{t} B{t};
                     zeros(n_inputs+1,n_states+1) eye(n_inputs+1)];
                 
        Bprime{t} = [B{t}(1:n_states+1, 1:n_inputs);
                     eye(3,2)];
        
        if t == 1
            Qprime{t} = [Q{t} zeros(n_states+1, n_inputs+1);
                         zeros(n_inputs+1, n_states+1) zeros(size(R{t}))];
        else
            Qprime{t} = [Q{t} zeros(n_states+1, n_inputs+1);
                         zeros(n_inputs+1, n_states+1) R{t-1}];
        end
        
        % Matrix to penalize change in control inputs
        Rprime{t} = zeros(2);
                 
        % Extend matrices to be able to store previous state in current
        % state (and penalize for change in control inputs)
%         Aprime{t} = [A{t} zeros(size(A{t}));% B{t}; 
%                  eye(length(s_ref) / 2) zeros(length(s_ref) / 2)];% zeros(length(s_ref) + 1, length(u_ref) + 1);
%                  %zeros(length(u_ref) + 1, (length(s_ref) + 1)*2)];% eye(length(u_ref) + 1)];
%         Bprime{t} = [B{t};
%                  zeros(length(s_ref) / 2, length(u_ref))]; 
%                  %eye(length(u_ref) + 1)];
%         Qprime{t} = Q{t};%[Q{t} zeros(length(s_refprime) + 1, length(u_ref) + 1); 
%                  %zeros(length(u_ref) + 1, length(s_refprime) + 1) R{t}];
%         Rprime{t} = R{t};%0.1 * eye(length(u_ref) + 1);
    end

    Q{H} = quadraticizeCosts(@g, [], S(H,1:3)', [], my_eps);
    Q{H} = (1 - alpha) * Q{H} + alpha * [eye(3) zeros(3,1); zeros(1, 3 + 1)];
    
    Qprime{H} = [Q{H} zeros(n_states+1, n_inputs+1);
                 zeros(n_inputs+1, n_states+1) R{H-1}];
    
%     Qprime{H} = quadraticizeCosts(@g, [], S(H,1:13)', [], my_eps);
%     Qprime{H} = (1 - alpha) * Qprime{H} + alpha * [eye(13) zeros(13, 1); zeros(1, 13 + 1)];
%     
    A = Aprime;
    B = Bprime;
    Q = Qprime;
    R = Rprime;
end

