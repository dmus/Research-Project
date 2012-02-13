load('track01_mrracer.mat');

% Actions: velocity and steering control
A = Actions(:,1) + -1 * Actions(:,2);
A(:,2) = Actions(:,5);

% States: distFromStart, trackPos, angle, speedX, speedY
S = States(:, [4, 69, 1, 47, 48]);

X = ones(size(S,1) - 1, 1);
X = [X S(1:end-1,:) A(1:end-1,:)];
y = S(2:end,5);

theta = pinv(X' * X) * X' * y;