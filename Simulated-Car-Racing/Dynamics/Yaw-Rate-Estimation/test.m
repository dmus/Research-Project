addpath('../Trials');
filename = 'track01_MrRacer.mat';

T = load(filename);
    
% Remove states and actions before start signal
States = T.States(T.States(:,2) > 0,:);
Actions = T.Actions(T.States(:,2) > 0,:);

S(:,1) = States(:,47) * 1000 / 3600;
S(:,2) = States(:,48) * 1000 / 3600;

%50-68
Landmarks = zeros(0,2);
c = zeros(0,1);
for i = 1:19
    sensor = 49 + i;
    angle = ((i-10) / 9) * (0.5 * pi) * -1;
    
    if States(1,sensor) >= 200
        continue;
    end
    
    x = States(1, sensor) * cos(angle);
    y = States(1, sensor) * sin(angle);
    Landmarks = [Landmarks; [x y]];
    
    c = [c; 'b'];
end

new_pos = [S(2,2) S(2,1)];
c = [c;'r'];

x = [Landmarks(:,2); new_pos(1)];
y = [Landmarks(:,1); new_pos(2)];

scatter(x * -1, y, 5, c);