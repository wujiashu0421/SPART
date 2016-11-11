function [J0, Jm]=Jacob(rxi,r0,rL,P0,pm,i,robot) %#codegen
% Computes the Jacobian of the xi point.
%
% Input ->
%   rxi -> Inertial position of the point of interest.
%   r0 -> Inertial position of the base-spacecraft.
%   r -> Links inertial positions.
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   i -> Link where the point xi is located.
%   robot -> Robot model.
%
% Output ->
%   J0 -> Base-spacecraft Jacobian
%   Jm -> Manipulator Jacobian

%=== LICENSE ===%

%=== CODE ===%

%--- Number of links  ---%

%Base Jacobian
J0=[eye(3),zeros(3,3);SkewSym(r0-rxi),eye(3)]*P0;

%Pre-allocate
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    Jm=zeros(6,robot.n_q);
end
%Manipulator Jacobian
for j=1:i
    %If joint is not fixed
    if robot.joints(j).type~=0
        if robot.Con.Branch(i,j)==1
            Jm(1:6,robot.joints(j).q_id)=[eye(3),zeros(3,3);SkewSym(rL(1:3,j)-rxi),eye(3)]*pm(1:6,j);
        else
            Jm(1:6,robot.joints(j).q_id)=zeros(6,1);
        end
    end
end

%Add zeros if required
if isempty(coder.target) %Only when not pre-allocated
    if i<robot.n_q
        Jm(1:6,i+1:robot.n_q)=zeros(6,robot.n_q-i);
    end
end

end