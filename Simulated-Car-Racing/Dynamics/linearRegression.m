function [theta] = linearRegression(X, y)
%LINEARREGRESSION Computes the closed-form solution to a linear least squares problem 
%   LINEARREGRESSION(X,y) computes the closed-form solution to linear 
%   regression using the normal equation.

    theta = pinv(X' * X) * X' * y;

end
