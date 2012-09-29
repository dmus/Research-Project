%% Compute offline a policy

% Read output log and do some formatting
init;

% Horizon, corresponding to 1 seconds
H = 50;

% Compute 6-dimensional state containing (position, orientation, velocity, 
% angular rate)
S = computeStateRepresentation(Laps{2}.S(1:H+1,:));
% Gas, brake and steering controls
U = Laps{2}.A(1:H+1,[1 2 5]);

save('exp1.mat', 'S', 'U');
% load('exp1.mat', 'S', 'U');

times = computeDiscretizedTimes(Laps{2}.S(1:H+1,:));

% Map represents the track axis
Map = buildMap(S, Laps{2}.S);
trackLength = 6205.46;
trackWidth = 12;

% Iterate until convergence
for i = 1:10
    % Parameter for finite difference methods
    my_eps = 0.1;
    
    features = Laps{1}.S(end,[4 69 1])';
    features(1) = features(1) - trackLength;
    s_ref_previous = [S(1,4:6)'; features];
    
    for t = 1:H
        % Linear and angular velocities
        s_ref_dynamic = S(t,4:6)';
        s_ref_tplus1_dynamic = S(t+1,4:6)';
        
        % Add state features
        s_ref = [s_ref_dynamic; Laps{2}.S(t,[4 69 1])'];
        s_ref_tplus1 = [s_ref_tplus1_dynamic; Laps{2}.S(t+1,[4 69 1])'];
        
        % Control input and time difference
        u_ref = U(t,:)';
        dt = times(t+1) - times(t);

        % Linearize dynamics around reference trajectory
        [A{t}, B{t}, c{t}] = linearizeDynamics(@f, s_ref, u_ref, dt, my_eps, s_ref_tplus1, Map);
        
        % Transform affine system into standard LQR format
        A{t} = [A{t} c{t}; zeros(1, size(A{t},2)) 1];
        B{t} = [B{t} zeros(size(B{t},1),1); zeros(1,size(B{t},2) + 1)];

        % Approximate costs in quadratic form around reference trajectory
        s_refprime = [s_ref; 1; s_ref_previous];
        [Q{t}, R{t}] = quadraticizeCosts(@g, @h, s_refprime, u_ref, my_eps);

        % Extend matrices to be able to store previous state in current
        % state and penalize for change in control inputs
        Aprime{t} = [A{t} zeros(size(A{t})) B{t}; 
                     eye(length(s_ref) + 1) zeros(length(s_ref) + 1) zeros(length(s_ref) + 1, length(u_ref) + 1);
                     zeros(length(u_ref) + 1, (length(s_ref) + 1)*2) eye(length(u_ref) + 1)];
        Bprime{t} = [B{t};
                     zeros(length(s_ref) + 1, length(u_ref) + 1); 
                     eye(length(u_ref) + 1)];
        Qprime{t} = [Q{t} zeros(length(s_refprime) + 1, length(u_ref) + 1); 
                     zeros(length(u_ref) + 1, length(s_refprime) + 1) R{t}];
        Rprime{t} = 0.1 * eye(length(u_ref) + 1);
        
        s_ref_previous = s_ref;
    end

    [K, P] = createTimeVaryingController(Aprime, Bprime, Qprime, Rprime, Qprime{H});
    Policy{i}.K = K;
    Policy{i}.P = P;
%     for i = 1:H
%         s = S(H-i+1,:)';
%         z = [s-s; 1];
%         cost = z' * P{i} * z;
%         fprintf('Cost to go with %i steps to go: %f\n', i, cost);
%     end
end