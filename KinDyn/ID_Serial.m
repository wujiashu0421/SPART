function [tauq0,tauqm] = ID_Serial(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,data) %#codegen
% This function solves the inverse dynamics problem (it obtains the
% generalized forces from the accelerations) for a Serial Manipulator.
%
% Input ->
%   wF0 -> External forces on the base-spacecraft.
%   wFm -> External forces on the manipulator links CoM.
%   t0 -> Base-spacecraft twist vector
%   tm -> Manipulator twist vector.
%   t0dot -> Base-spacecraft twist rate vector
%   tmdot -> Manipulator twist rate vector.
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   I0 -> Base-spacecraft inertia in inertial frame.
%   Im -> Manipulator inertia in inertial frame.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   data -> Manipulator data.
%       data.n -> Manipulator number of joints and links.
%
% Output ->
%   tau0 -> Base-spacecraft joint space forces.
%   taum -> Manipulator joint space forces.

%=== LICENSE ===%


%=== CODE ===%


%--- Number of links and Joints ---%
n=data.n;

%--- Mdot ---%
%Base-spacecraft Mdot
Mdot0=[SkewSym(t0(1:3))*I0, zeros(3,3); zeros(3,3), zeros(3,3)];
%Pre-allocate
Mdot=zeros(6,6,n);
%Manipulator Mdot
for i=1:n
    Mdot(1:6,1:6,i)=[SkewSym(tm(1:3,i))*Im(1:3,1:3,i), zeros(3,3); zeros(3,3), zeros(3,3)];
end

%--- Forces ---%
%Base-spacecraft
wq0=[I0,zeros(3,3);zeros(3,3),data.base.mass*eye(3)]*t0dot+Mdot0*t0-wF0;
%Pre-allocate
wq=zeros(6,n);
%Manipulator
for i=1:n
    wq(1:6,i)=[Im(1:3,1:3,i),zeros(3,3);zeros(3,3),data.man(i).mass*eye(3)]*tmdot(1:6,i)+Mdot(1:6,1:6,i)*tm(1:6,i)-wFm(1:6,i);
end
%Pre-allocate
wq_tilde=zeros(6,n);
%Initialize wq_tilde
wq_tilde(1:6,n)=wq(1:6,n);
%Backwards recursion
for i=n-1:-1:1
    wq_tilde(1:6,i)=wq(1:6,i)+Bij(1:6,1:6,i+1,i)'*wq_tilde(1:6,i+1);
end
%Base-spacecraft
wq_tilde0=wq0+Bi0(1:6,1:6,1)'*wq_tilde(1:6,1);

%---- Joint forces ---%
%Base-spacecraft
tauq0=P0'*wq_tilde0;
%Pre-allocate
tauqm=zeros(n,1);
%Manipulator joint forces.
for i=1:n
    tauqm(i,1)=pm(1:6,i)'*wq_tilde(1:6,i);
end

end