function [t0,tm]=Velocities_Serial(Bij,Bi0,P0,pm,q0dot,qmdot,data) %#codegen
% Computes the velocities of a serial manipulator.
%
% Input ->
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
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
%   t0 -> Base-spacecraft twist vector [wx,wy,wz,vx,vy,vz].
%   tm -> Manipulator twist vector [wx,wy,wz,vx,vy,vz].

%=== LICENSE ===%

%=== CODE ===%

%--- Number of links and Joints ---%
n=data.n;
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







