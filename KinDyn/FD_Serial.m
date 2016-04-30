function [q0ddot,qmddot] = FD_Serial(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,data) %#codegen
% This function solves the inverse dynamics problem (it obtains the
% generalized forces from the accelerations) for a Serial Manipulator with 
% a floating base.
%
% Input ->
%   tauq0 -> Base-spacecraft control forces.
%   tauqm -> Manipulator control forces.
%   wF0 -> External forces on the base-spacecraft.
%   wFm -> External forces on the manipulator links CoM.
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
%   data -> Manipulator data.
%       data.n -> Manipulator number of joints and links.
%
% Output ->
%   q0ddot -> Base-spacecraft acceleration.
%   qmddot -> Manipulator acceleration.

%=== LICENSE ===%


%=== CODE ===%

%--- Number of links and Joints ---%
n=data.n;

%---- Inverse Dynamics with 0 accelerations ---%
%Recompute Accelerations with q0ddot=qmddot=0
[t0dot,tmdot]=Accelerations_Serial(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,zeros(6,1),zeros(n,1),data);
%Use the inverse dynamics
[tau0_0ddot,tauqm_0ddot] = ID_Serial(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,data);

%--- Forward Dynamics ---%

%Initialize solution
phi0=tauq0-tau0_0ddot;
phi=tauqm-tauqm_0ddot;


%--- M hat, psi hat and psi  ---%
%Pre-allocate
M_hat=zeros(6,6,n);
psi_hat=zeros(6,n);
psi=zeros(6,n);
%Initialize
M_hat(1:6,1:6,n)=[Im(1:3,1:3,n),zeros(3,3);zeros(3,3),data.man(n).mass*eye(3)];
psi_hat(1:6,n)=M_hat(1:6,1:6,n)*pm(1:6,n);
psi(1:6,n)=psi_hat(1:6,n)/(pm(1:6,n)'*psi_hat(1:6,n));
%Backwards recursion
for i=n-1:-1:1
    M_hatii=M_hat(1:6,1:6,i+1)-psi_hat(1:6,i+1)*psi(1:6,i+1)';
    M=[Im(1:3,1:3,i),zeros(3,3);zeros(3,3),data.man(i).mass*eye(3)];
    M_hat(1:6,1:6,i)=M+Bij(1:6,1:6,i+1,i)'*M_hatii*Bij(1:6,1:6,i+1,i);
    psi_hat(1:6,i)=M_hat(1:6,1:6,i)*pm(1:6,i);
    psi(1:6,i)=psi_hat(1:6,i)/(pm(1:6,i)'*psi_hat(1:6,i));
end
%Base-spacecraft
M_hat0ii=M_hat(1:6,1:6,1)-psi_hat(1:6,1)*psi(1:6,1)';
M0=[I0,zeros(3,3);zeros(3,3),data.base.mass*eye(3)];
M_hat0=M0+Bi0(1:6,1:6,1)'*M_hat0ii*Bi0(1:6,1:6,1);
psi_hat0=M_hat0*P0;
%psi0=psi_hat0/(P0'*psi_hat0);

%--- eta ---
%Pre-allocate and initialize
eta=zeros(6,n);
phi_hat=zeros(n);
phi_tilde=zeros(n);
%Initialize
phi_hat(n)=phi(n)-pm(1:6,n)'*eta(1:6,n);
phi_tilde(n)=phi_hat(n)/(pm(1:6,n)'*psi_hat(1:6,n));
%Backwards recursion
for i=n-1:-1:1
    eta(1:6,i)=Bij(1:6,1:6,i+1,i)'*(psi(1:6,i+1)*phi_hat(i+1)+eta(1:6,i+1));
    phi_hat(i)=phi(i)-pm(1:6,i)'*eta(1:6,i);
    phi_tilde(i)=phi_hat(i)/(pm(1:6,i)'*psi_hat(1:6,i));
end
%Base-spacecraft
eta0=Bi0(1:6,1:6,1)'*(psi(1:6,1)*phi_hat(1)+eta(1:6,1));
phi_hat0=phi0-P0'*eta0;
phi_tilde0=(P0'*psi_hat0)\phi_hat0;


%--- Base-spacecraft acceleration ---%
q0ddot=phi_tilde0;

%--- Manipulator acceleration (and mu) ---%
%Pre-allocate
mu=zeros(6,n);
qmddot=zeros(n,1);
%Initialize
mu(1:6,1)=Bi0(1:6,1:6,1)*(P0*q0ddot);
qmddot(1)=phi_tilde(1)-psi(1:6,1)'*mu(1:6,1);
%Forward recursion
for i=2:n
    mu(1:6,i)=Bij(1:6,1:6,i,i-1)*(pm(1:6,i-1)*qmddot(i-1)+mu(1:6,i-1));
    qmddot(i)=phi_tilde(i)-psi(1:6,i)'*mu(1:6,i);
end

end