function [x_skew] = SkewSym(x) %#codegen
%Computes the skewsymmetric matrix of a vector

x_skew=[0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];

end