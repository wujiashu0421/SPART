function [tauqm,q0ddot] = Floating_ID_Serial(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,qmddot,data) %#codegen
% This function solves the inverse dynamics problem (it obtains the
% generalized forces from the accelerations) for a Serial Manipulator with 
% a floating base.
%
% Input ->
%   wF0 -> External forces on the base-spacecraft.
%   wFm -> External forces on the manipulator links CoM.
%   Mm_tilde -> Manipulator mass matrix of composite body.
%   H0 -> Base-spacecraft inertia matrix.
%   t0 -> Base-spacecraft twist vector
%   tm -> Manipulator twist vector.
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   I0 -> Base-spacecraft inertia in inertial frame.
%   Im -> Manipulator inertia in inertial frame.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   q0dot -> Base-spacecraft velocities [angular velocity in body, linear
%   velocity in inertial].
%   qmdot -> Manipulator joint rates.
%   qmddot -> Manipulator joint accelerations.
%   data -> Manipulator data.
%       data.n -> Manipulator number of joints and links.
%
% Output ->
%   taum -> Manipulator joint space forces.
%   q0ddot -> Base-spacecraft acceleration.

%=== LICENSE ===%

%=== CODE ===%

%Number of links and Joints
n=data.n;

%Recompute Accelerations with qddot=0
[t0dot,tmdot]=Accelerations_Serial(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,zeros(6,1),qmddot,data);

%Use the inverse dynamics
[tauq0_0ddot,tauqm] = ID_Serial(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,data);

%Compute Kappa
kappa=zeros(n,6);
for i=1:n
    kappa(i,1:6)=pm(1:6,i)'*Mm_tilde(1:6,1:6,i)*Bi0(1:6,1:6,i);
end

%Compute base-spacecraft acceleration
q0ddot=-H0\tauq0_0ddot;

%Update joint forces
for i=1:n
    tauqm(i)=kappa(i,1:6)*q0ddot+tauqm(i);
end

end
