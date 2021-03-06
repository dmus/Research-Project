%% Compute offline a policy

% Read output log and do some formatting
init;

% Horizon, corresponding to 1 seconds
H = 250;

refTrajectory.S = [Laps{1}.S; Laps{2}.S];
refTrajectory.A = [Laps{1}.A; Laps{2}.A];

% % Compute 6-dimensional state containing (position, orientation, velocity, 
% % angular rate)
% S = computeStateRepresentation(refTrajectory.S);
% % Gas, brake combined and steering control
% U = [refTrajectory.A(:,1) + -1 * refTrajectory.A(:,2), refTrajectory.A(:,5)];
% 
% save('exp1.mat', 'S', 'U');
load('exp1.mat', 'S', 'U');
load('Map.mat', 'Map');

times = computeDiscretizedTimes(refTrajectory.S);

% Map represents the track axis
% Map = buildMap(computeStateRepresentation(Laps{2}.S), Laps{2}.S);
trackLength = 6205.46;
trackWidth = 12;


% Parameter for finite difference methods
my_eps = 0.1;

alpha = 0.99;

reference.S = zeros(H,14);
reference.U = zeros(H-1,3);
reference.T = zeros(H,1);

% Initialize previous state
s_ref_previous = [S(1,4:6)'; refTrajectory.S(1,[4 69 1])'];
    
for t = 1:H-1
    
    % Add state features
    s_ref = [S(t,4:6)'; refTrajectory.S(t,[4 69 1])'];
    s_ref_tplus1 = [S(t+1,4:6)'; refTrajectory.S(t+1,[4 69 1])'];
        
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
    
    % Also penalize deviation from reference trajectory
    Q{t} = (1 - alpha) * Q{t} + alpha * [eye(length(s_refprime)) zeros(length(s_refprime), 1); zeros(1, length(s_refprime) + 1)];
    R{t} = (1 - alpha) * R{t} + alpha * [eye(length(u_ref)) zeros(length(u_ref), 1); zeros(1, length(u_ref) + 1)];
    
    % Extend matrices to be able to store previous state in current
    % state and penalize for change in control inputs
    Aprime{t} = [A{t} zeros(size(A{t}));% B{t}; 
                 eye(length(s_ref) + 1) zeros(length(s_ref) + 1)];% zeros(length(s_ref) + 1, length(u_ref) + 1);
                 %zeros(length(u_ref) + 1, (length(s_ref) + 1)*2)];% eye(length(u_ref) + 1)];
    Bprime{t} = [B{t};
                 zeros(length(s_ref) + 1, length(u_ref) + 1)]; 
                 %eye(length(u_ref) + 1)];
    Qprime{t} = Q{t};%[Q{t} zeros(length(s_refprime) + 1, length(u_ref) + 1); 
                 %zeros(length(u_ref) + 1, length(s_refprime) + 1) R{t}];
    Rprime{t} = R{t};%0.1 * eye(length(u_ref) + 1);
    
    reference.S(t,:) = [s_refprime; 1]';
    reference.U(t,:) = [u_ref; 1]';
    %reference.T
    
    s_ref_previous = s_ref;
end

s_ref = [S(H,4:6)'; refTrajectory.S(H,[4 69 1])'; 1; s_ref_previous];
u_ref = U(H,:)';

reference.S(H,:) = [s_ref; 1]';
reference.T = times(1:H);
Qprime{H} = quadraticizeCosts(@g, @h, s_ref, u_ref, my_eps);
Qprime{H} = (1 - alpha) * Qprime{H} + alpha * [eye(length(s_ref)) zeros(length(s_ref), 1); zeros(1, length(s_ref) + 1)];

[K, P] = createTimeVaryingController(Aprime, Bprime, Qprime, Rprime, Qprime{H});
save('policy.mat', 'K', 'P');
save('reference.mat', 'reference');
%     for i = 1:H
%         s = S(H-i+1,:)';
%         z = [s-s; 1];
%         cost = z' * P{i} * z;
%         fprintf('Cost to go with %i steps to go: %f\n', i, cost);
%     end
