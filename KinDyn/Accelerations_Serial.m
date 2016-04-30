function [t0dot,tmdot]=Accelerations_Serial(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,q0ddot,qmddot,data) %#codegen
% Computes the accekerations (twist-rate) of a Serial Manipulator.
%
% Input ->
%   t0 -> Base-spacecraft twist vector
%   tm -> Manipulator twist vector.
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   q0dot -> Base-spacecraft velocities [angular velocity in body, linear
%   velocity in inertial].
%   qmdot -> Manipulator joint rates.
%   q0ddot -> Base-spacecraft accelerations.
%   qmddot -> Manipulator joint accelerations.
%   data -> Manipulator data.
%       data.n -> Manipulator number of joints and links.
% Output ->
%   t0dot -> Base-spacecraft twist rate vector
%   tmdot -> Manipulator twist rate vector.

%=== LICENSE ===%


%=== CODE ===%


%--- Number of links and Joints ---%
n=data.n;

%--- Omega matrices ---%
%Base-Spacecraft 
Omega0=[SkewSym(t0(1:3)), zeros(3,3);
        zeros(3,3), SkewSym(t0(1:3))];
%Pre-allocate
Omegam=zeros(6,6,n);
%Compute Omega for manipulator
for i=1:n
    Omegam(1:6,1:6,i)=[SkewSym(tm(1:3,i)), zeros(3,3);
        zeros(3,3), SkewSym(tm(1:3,i))];
end

%--- Twist Rate ---%
%Base-spacecraft
t0dot = Omega0*P0*q0dot+P0*q0ddot;
%Pre-allocate
tmdot=zeros(6,n);
%First link
tmdot(1:6,1)=Bi0(1:6,1:6,1)*t0dot+[zeros(3,6);SkewSym(t0(4:6)-tm(4:6,1)),zeros(3,3)]*t0+ Omegam(1:6,1:6,1)*pm(1:6,1)*qmdot(1)+pm(1:6,1)*qmddot(1);
%Forward recursion for rest of the links.
for i=2:n
    tmdot(1:6,i)=Bij(1:6,1:6,i,i-1)*tmdot(1:6,i-1)+[zeros(3,6); SkewSym(tm(4:6,i-1)-tm(4:6,i)), zeros(3,3)]*tm(1:6,i-1)+Omegam(1:6,1:6,i)*pm(1:6,i)*qmdot(i)+pm(1:6,i)*qmddot(i);
end


end