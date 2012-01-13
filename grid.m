gamma = 0.99;

grid = zeros(128,128);
rewards = rand(64,1);
rewards(rand(64,1) < 0.9) = 0;

D = rand(128,128);