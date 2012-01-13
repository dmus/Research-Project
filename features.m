function f = features(state)
%FEATURES Summary of this function goes here
%   Detailed explanation goes here
    f = zeros(n_macrocells, 1);

    col = floor(state / n); 
	row = mod(state,n);

    macro_col = ceil(col / (n/m));
    macro_row = ceil(row / (n/m));
    macrocell = (macro_col * n) - (n - macro_row);
    f(macrocell) = 1;
end
