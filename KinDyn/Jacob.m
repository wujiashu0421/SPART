function [J0, Jm]=Jacob(rxi,r0,r,P0,pm,i,n) %#codegen
% Computes the Jacobian of the xi point.
%
% Input ->
%   rxi -> Inertial position of the point of interest.
%   r0 -> Inertial position of the base-spacecraft.
%   r -> Links inertial positions.
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   i -> Link where the point xi is located.
%   n -> Manipulator number of joints and links.
%       
% Output ->
%   J0 -> Base-spacecraft Jacobian
%   Jm -> Manipulator Jacobian

%=== LICENSE ===%

%=== CODE ===%

%Base Jacobian
J0=[eye(3),zeros(3,3);SkewSym(r0-rxi),eye(3)]*P0;

%Pre-allocate
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    Jm=zeros(6,n);
end
%Manipulator Jacobian
for j=1:i
    Jm(1:6,j)=[eye(3),zeros(3,3);SkewSym(r(1:3,j)-rxi),eye(3)]*pm(1:6,j);
end

%Add zeros if required
if isempty(coder.target) %Only when not pre-allocated
    if i<n
        Jm(1:6,i+1:n)=zeros(6,n-i);
    end
end


end