init;

H = 20; % Corresponding to 5 seconds

S = computeStateRepresentation(Laps{2}.S(1:H+1,:));
U = computeActionRepresentation(Laps{2}.A(1:H+1,:));

save('exp1.mat', 'S', 'U');
% load('exp1.mat', 'S', 'U');

times = computeDiscretizedTimes(Laps{2}.S(1:H+1,:));

% Approximate A, B matrices
i = 1;
my_eps = 0.05;
for t = 1:H
    x_ref = S(t,:)';
    u_ref = U(t,:)';
    dt = times(t+1) - times(t);
    x_ref_tplus1 = S(t+1,:)';
    [A{t}, B{t}, c{t}] = linearizeDynamics(@f, x_ref, u_ref, dt, my_eps, x_ref_tplus1);
    A{t} = [A{t} c{t}; zeros(1, size(A{t},2)) 1];
    B{t} = [B{t}; zeros(1,size(B{t},2))];
    
    
    % x(:,t+1) - x_ref_tplus1  to-first-order-equal-to A*( x(:,t)-x_ref ) +
    % B* ( u(:,t) - u_ref ) + c
    %A{t} * () + B{t} * () + c;
end

% Time-independent Q and R matrices
Qfinal = 10*eye(7);
%Qfinal(5, 5) = 1;

Q = eye(7);
Q(4:6, 4:6) = 0.1;

R = 0.1 * eye(3);

[K, P] = createTimeVaryingController(A, B, Q, R, Qfinal);

for i = 1:H
    s = S(H-i+1,:)';
    z = [s-s; 1];
    cost = z' * P{i} * z;
    fprintf('Cost to go with %i steps to go: %f\n', i, cost);
end