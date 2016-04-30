function [H0, H0m, Hm] = GIM_Serial(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,data) %#codegen
% Computes the Generalized Inertia Matrix of a Serial Manipulator.
%
% Input ->
%   M0_tilde -> Base-spacecraft mass matrix of composite body.
%   Mm_tilde -> Manipulator mass matrix of composite body.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   data -> Manipulator data.
%       data.n -> Manipulator number of joints and links.
%
% Output ->
%   H0 -> Base-spacecraft inertia matrix.
%   H0m -> Base-spacecraft - manipulator coupling inertia matrix.
%   Hm -> Manipulator inertia matrix.

%=== LICENSE ===%


%=== CODE ===%

%--- Number of links and Joints ---%
n=data.n;

%--- H Martix ---%
%Base-spacecraft Inertia matrix
H0 = P0'*M0_tilde*P0;
%Pre-allocate Hm
Hm=zeros(n,n);
%Manipulator Inertia matrix Hm
for j=1:n
    for i=j:n
        Hm(i,j)=pm(1:6,i)'*Mm_tilde(1:6,1:6,i)*Bij(1:6,1:6,i,j)*pm(1:6,j);
        Hm(j,i)=Hm(i,j);
    end
end
%Pre-allocate H0m
H0m=zeros(6,n);
%Coupling Inertia matrix
for i=1:n
    H0m(1:6,i)=(pm(1:6,i)'*Mm_tilde(1:6,1:6,i)*Bi0(1:6,1:6,i)*P0)';
end

end