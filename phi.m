function f = phi(state)
%PHI Summary of this function goes here
%   Detailed explanation goes here
    global n m n_macrocells;

    f = zeros(n_macrocells, 1);

    col = ceil(state / n); 
	row = mod(state,n);
    if row == 0
        row = 4;
    end
    
    macro_col = ceil(col / (n/m));
    macro_row = ceil(row / (n/m));
    macrocell = (macro_col * n) - (n - macro_row);
    f(macrocell) = 1;

end

