%% Compute offline a policy

init;

H = 20; % Horizon, corresponding to 5 seconds

S = computeStateRepresentation(Laps{2}.S(1:H+1,:));
U = computeActionRepresentation(Laps{2}.A(1:H+1,:));

save('exp1.mat', 'S', 'U');
% load('exp1.mat', 'S', 'U');

times = computeDiscretizedTimes(Laps{2}.S(1:H+1,:));

for i = 1:10
    % Approximate A, B matrices
    my_eps = 0.05;
    for t = 1:H
        x_ref = S(t,:)';
        u_ref = U(t,:)';
        dt = times(t+1) - times(t);
        x_ref_tplus1 = S(t+1,:)';
        [A{t}, B{t}, c{t}] = linearizeDynamics(@f, x_ref, u_ref, dt, my_eps, x_ref_tplus1);
        A{t} = [A{t} c{t}; zeros(1, size(A{t},2)) 1];
        B{t} = [B{t}; zeros(1,size(B{t},2))];

        [Q{t}, R{t}] = quadratizeCosts(@g, @h, x_ref, u_ref, my_eps);

        % x(:,t+1) - x_ref_tplus1  to-first-order-equal-to A*( x(:,t)-x_ref ) +
        % B* ( u(:,t) - u_ref ) + c
        %A{t} * () + B{t} * () + c;
    end

    [K, P] = createTimeVaryingController(A, B, Q, R, Qfinal);
    Policy{i}.K = K;
    Policy{i}.P = P;
%     for i = 1:H
%         s = S(H-i+1,:)';
%         z = [s-s; 1];
%         cost = z' * P{i} * z;
%         fprintf('Cost to go with %i steps to go: %f\n', i, cost);
%     end
end