function [t0,tm,Bij,Bi0,P0,pm]=DiffKinematics_Serial(R0,r0,q0dot,qmdot,r,l,e,g,data) %#codegen
% Computes the differential kineamtics of a serial manipulator.
%
% Input ->
%   R0 -> Rotation matrix from the base-spacecraft to the inertial frame.
%   r0 -> Position of the base-spacecraft to the inertial frame.
%   q0dot -> Base-spacecraft velocities [angular velocity in body, linear
%   velocity in inertial].
%   qmdot -> Manipulator joint rates.
%   data -> Manipulator data.
%       data.n -> Manipulator number of joints and links.
%       data.base -> Base-spacecraft data
%           data.base.T_L0_J1 -> Homogeneous transformation of the first
%           joint w.r.t. the base-spacecraft.
%       data.man -> Manipulator data.
%           data.man(i).DH -> DH parameters of the ith joint.
%           data.man(i).type -> Type of joint. type==0 for revolute,
%           otherwise prismatic.
%           data.man(i).b -> Vector from the ith link to the following
%           joint i+1.
%       data.EE -> End-effector parameters
%           data.EE.theta -> Rotation about the final z-axis so that any
%           desired End-Effector cartesian coordinate system orientation
%           can be achieved.
%
% Output ->
%   t0 -> Base-spacecraft twist vector
%   tm -> Manipulator twist vector.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.

%=== LICENSE ===%

%=== CODE ===%

%--- Number of links and Joints ---%
n=data.n;

%--- Twist-propagtaion matrix ---%
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate Bij
    Bij=zeros(6,6,n,n);
end
%Compute Bij
for j=1:n
    for i=1:n
        Bij(1:6,1:6,i,j)=[eye(3), zeros(3,3); SkewSym(r(1:3,j)-r(1:3,i)), eye(3)];
    end
end
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate Bi0
    Bi0=zeros(6,6,n);
end
%Compute Bi0
for i=1:n
    Bi0(1:6,1:6,i)=[eye(3), zeros(3,3); SkewSym(r0-r(1:3,i)), eye(3)];
end

%--- Twist-Propagation vector ---%
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    pm=zeros(6,n);
end
%Base-spacecraft
P0=[R0,zeros(3,3); zeros(3,3), eye(3)];
%Fordward recursion to obtain the Twist-Propagation vector
for i=1:n
    if data.man(i).type==0
        %Revolute joint
        pm(1:6,i)=[e(1:3,i);cross(e(1:3,i),g(1:3,i))];
    else
        %Prismatic joint
        pm(1:6,i)=[zeros(3,1);e(1:3,i)];
    end
end

%--- Generalized twist vector ---%
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-Allocate
    tm=zeros(6,n);
end
%Base-spacecraft
t0=P0*q0dot;
%First link
tm(1:6,1)=Bi0(1:6,1:6,1)*t0+pm(1:6,1)*qmdot(1);
%Fordward recursion to obtain the twist vector
for i=2:n
    tm(1:6,i)=Bij(1:6,1:6,i,i-1)*tm(1:6,i-1)+pm(1:6,i)*qmdot(i);
end

end







