function f = phi(state)
%PHI Summary of this function goes here
%   Detailed explanation goes here
    global n m num_macrocells;

    f = zeros(num_macrocells, 1);

    col = ceil(state / n); 
    row = mod(state,n);
    if row == 0
        row = n;
    end
    
    macro_col = ceil(col / m);
    macro_row = ceil(row / m);
    macrocell = ((macro_col - 1) * (n/m)) + macro_row;
    f(macrocell) = 1;

end

