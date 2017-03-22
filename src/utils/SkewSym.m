function [x_skew] = SkewSym(x)
% Computes the skew-symmetric matrix of a vector
%
% [x_skew] = SkewSym(x)
%
% :parameters:
%	* x -- [3x1] vector.
%
% :return:
%	* x_skew -- [3x3] skew-symmetric matrix of the x vector.

x_skew=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];

end