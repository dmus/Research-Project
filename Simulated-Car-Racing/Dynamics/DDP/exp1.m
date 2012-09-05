init;

H = 40; % Corresponding to 5 seconds

% S{1} = computeStateRepresentation(Laps{2}.S(1:H+1,:));
% U{1} = computeActionRepresentation(Laps{2}.A(1:H+1,:));
% 
% save('exp1.mat', 'S', 'U');
load('exp1.mat', 'S', 'U');

times = computeDiscretizedTimes(Laps{2}.S(1:H+1,:));

% Approximate A, B matrices
i = 1;
my_eps = 0.05;
for t = 1:H
    x_ref = S{i}(t,:)';
    u_ref = U{i}(t,:)';
    dt = times(t+1) - times(t);
    x_ref_tplus1 = S{i}(t+1,:)';
    [A{t}, B{t}, c{t}] = linearizeDynamics(@f, x_ref, u_ref, dt, my_eps, x_ref_tplus1);
end

% Time-independent Q and R matrices
Qfinal = 10*eye(6);
%Qfinal(5, 5) = 1;

Q = eye(6);
Q(4:6, 4:6) = 0.1;

R = 0.1 * eye(3);

[K, P] = createTimeVaryingController(A, B, Q, R, Qfinal);

for i = 1:H
    s = S{1}(H-i+1,:)';
    cost = s' * P{i} * s;
    fprintf('Cost to go with %i steps to go: %f\n', i, cost);
end