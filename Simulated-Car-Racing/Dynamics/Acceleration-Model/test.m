s = S(1000,:)';
start = s;

% u1, u2, u3

% simulate for 0.2 s

for i = 1:10
    dt = 0.02;
    
    acc = model2.Apos * s + model2.Bpos * u3;
    acc(3) = model2.Arot * s + model2.Brot * u3;
    
    R = [cos(s(3)) -sin(s(3)); sin(s(3)) cos(s(3))];
    
    s(1:2) = R * (s(1:2) + acc(1:2) * dt);
    s(3) = s(3) + acc(3) * dt;
end

