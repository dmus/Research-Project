function [theta] = linearRegression(X, y)
%LINEARREGRESSION Computes the closed-form solution to linear regression 
%   LINEARREGRESSION(X,y) computes the closed-form solution to linear 
%   regression using the normal equations.

theta = pinv(X' * X) * X' * y;

end
