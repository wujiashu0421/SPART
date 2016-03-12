function [C0, C0m, Cm0, Cm] = C_Serial(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,data)
% Computes the Generalized Convective Inertia Matrix of a Serial Manipulator.
%
% Input ->
%   t0 -> Base-spacecraft twist vector
%   tm -> Manipulator twist vector.
%   I0 -> Base-spacecraft inertia in inertial frame.
%   Im -> Manipulator inertia in inertial frame.
%   M0_tilde -> Base-spacecraft mass matrix of composite body.
%   Mm_tilde -> Manipulator mass matrix of composite body.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   B0j -> Twist-propagation matrix (for i=0 and j>0).
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   data -> Manipulator data.
%       data.n -> Manipulator number of joints and links.
% Output ->
%   C0 -> Base-spacecraft convective inertia matrix.
%   C0m -> Base-spacecraft - manipulator coupling convective inertia matrix.
%   Cm0 -> Manipulator - Base-spacecraft coupling convective inertia matrix.
%   Cm -> Manipulator convective inertia matrix.

%=== LICENSE ===%

%=== CODE ===%

%--- Number of links and Joints ---%
n=data.n;

%--- Omega ---%
%Base-spacecraft Omega
Omega0=[SkewSym(t0(1:3)), zeros(3,3);
        zeros(3,3), SkewSym(t0(1:3))];
%Pre-allocate Omega
Omega=zeros(6,6,n);
%Compute Omega
for i=1:n
    Omega(1:6,1:6,i)=[SkewSym(tm(1:3,i)), zeros(3,3);
        zeros(3,3), SkewSym(tm(1:3,i))];
end

%--- Mdot ---%
%Base-spacecraft Mdot
Mdot0=[Omega0(1:3,1:3)*I0, zeros(3,3); zeros(3,3), zeros(3,3)];
%Pre-allocate
Mdot=zeros(6,6,n);
%Compute Mdot
for i=1:n
    Mdot(1:6,1:6,i)=[Omega(1:3,1:3,i)*Im(1:3,1:3,i), zeros(3,3); zeros(3,3), zeros(3,3)];
end

%--- Mdot tilde ---%
%Pre-Allocate
Mdot_tilde=zeros(6,6,n);
%Initialize
Mdot_tilde(1:6,1:6,n)=Mdot(1:6,1:6,n);
%Backwards recursion
for i=n-1:-1:1
    Mdot_tilde(1:6,1:6,i)=Mdot(1:6,1:6,i)+Mdot_tilde(1:6,1:6,i+1);
end
%Base-spacecraft
Mdot0_tilde = Mdot0+Mdot_tilde(1:6,1:6,1);

%--- Hij tilde ---%
%Pre-allocate Hij_tilde
Hij_tilde=zeros(6,6,n,n);
%Hij_tilde
for j=n:-1:1
    for i=n:-1:1
        if i==n
            Hij_tilde_pre=zeros(6,6);
            Bij_pre=zeros(6,6);
        else
            Hij_tilde_pre=Hij_tilde(1:6,1:6,i+1,i);
            Bij_pre=Bij(1:6,1:6,i+1,i);
        end
        Bdot=[zeros(3,3), zeros(3,3); SkewSym(tm(4:6,j)-tm(4:6,i)), zeros(3,3)];
        Hij_tilde(1:6,1:6,i,j)=Mm_tilde(1:6,1:6,i)*Bdot+Bij_pre'*Hij_tilde_pre;
    end
end
%Pre-allocate Hi0_tilde and H0j_tilde
Hi0_tilde=zeros(6,6,n);
H0j_tilde=zeros(6,6,n);
%Hi0_tilde
for i=n:-1:1
    if i==n
        Hij_tilde_pre=zeros(6,6);
        Bij_pre=zeros(6,6);
    else
        Hij_tilde_pre=Hij_tilde(1:6,1:6,i+1,i);
        Bij_pre=Bij(1:6,1:6,i+1,i);
    end
    Bdot=[zeros(3,3), zeros(3,3); SkewSym(t0(4:6)-tm(4:6,i)), zeros(3,3)];
    Hi0_tilde(1:6,1:6,i)=Mm_tilde(1:6,1:6,i)*Bdot+Bij_pre'*Hij_tilde_pre;
end
%H0j_tilde
for j=n:-1:1
    Bdot=[zeros(3,3), zeros(3,3); SkewSym(tm(4:6,j)-t0(4:6)), zeros(3,3)];
    Hij_tilde_pre=Hi0_tilde(1:6,1:6,1);
    H0j_tilde(1:6,1:6,j)=M0_tilde*Bdot+Bi0(1:6,1:6,1)'*Hij_tilde_pre;
end

% %H0_tilde
% Hij_tilde_pre=Hi0_tilde(1:6,1:6,1);
% Bdot=zeros(6,6);
% H0_tilde(1:6,1:6)=M0_tilde*Bdot+Bi0(1:6,1:6,1)'*Hij_tilde_pre;

%--- C Matrix ---%
%Pre-allocate
Cm=zeros(n,n);
C0m=zeros(6,n);
Cm0=zeros(n,6);
%Cm Matrix
for j=n:-1:1
    for i=n:-1:1
        if i<=j
            if j==n
                Cm(i,j)=pm(1:6,i)'*(Bij(1:6,1:6,j,i)'*Mm_tilde(1:6,1:6,j)*Omega(1:6,1:6,j)+Mdot_tilde(1:6,1:6,j))*pm(1:6,j);
            else
                Cm(i,j)=pm(1:6,i)'*(Bij(1:6,1:6,j,i)'*Mm_tilde(1:6,1:6,j)*Omega(1:6,1:6,j)+Bij(1:6,1:6,j+1,i)'*Hij_tilde(1:6,1:6,j+1,j)+Mdot_tilde(1:6,1:6,j))*pm(1:6,j);
            end
        else
            Cm(i,j)=pm(1:6,i)'*(Mm_tilde(1:6,1:6,i)*Bij(1:6,1:6,i,j)*Omega(1:6,1:6,j)+Hij_tilde(1:6,1:6,i,j)+Mdot_tilde(1:6,1:6,i))*pm(1:6,j);
        end
    end
end
%C0 matrix
C0 = P0'*(M0_tilde*Omega0+Bi0(1:6,1:6,1)'*Hi0_tilde(1:6,1:6,1)+Mdot0_tilde)*P0;
%C0m
for j=1:n
    if j==n
        C0m(1:6,j)=P0'*(Bi0(1:6,1:6,j)'*Mm_tilde(1:6,1:6,j)*Omega(1:6,1:6,j)+Mdot_tilde(1:6,1:6,j))*pm(1:6,j);
    else
        C0m(1:6,j)=P0'*(Bi0(1:6,1:6,j)'*Mm_tilde(1:6,1:6,j)*Omega(1:6,1:6,j)+Bi0(1:6,1:6,j+1)'*Hij_tilde(1:6,1:6,j+1,j)+Mdot_tilde(1:6,1:6,j))*pm(1:6,j);
    end
end
%Cm0
for i=1:n
    Cm0(i,1:6)=pm(1:6,i)'*(Mm_tilde(1:6,1:6,i)*Bi0(1:6,1:6,i)*Omega0+Hi0_tilde(1:6,1:6,i)+Mdot_tilde(1:6,1:6,i))*P0;
end

end

