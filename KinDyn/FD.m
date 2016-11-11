function [q0ddot,qmddot] = FD(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,robot) %#codegen
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
%   robot -> Robot model.
%
% Output ->
%   q0ddot -> Base-spacecraft acceleration.
%   qmddot -> Manipulator acceleration.

%=== LICENSE ===%

%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.

%=== CODE ===%

%--- Number of links and Joints ---%
n=robot.n_links;

%---- Inverse Dynamics with 0 accelerations ---%
%Recompute Accelerations with q0ddot=qmddot=0
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,zeros(6,1),zeros(n,1),robot);
%Use the inverse dynamics
[tau0_0ddot,tauqm_0ddot] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robot);

%--- Forward Dynamics ---%

%Initialize solution
phi0=tauq0-tau0_0ddot;
phi=tauqm-tauqm_0ddot;

%--- M hat, psi hat and psi  ---%
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    M_hat=zeros(6,6,n);
    psi_hat=zeros(6,n);
    psi=zeros(6,n);
end
%Backwards recursion
for i=n:-1:1
    %Initialize
    M_hat(1:6,1:6,i)=[Im(1:3,1:3,i),zeros(3,3);zeros(3,3),robot.links(i).mass*eye(3)];
    %Add children contributions
    for j=find(robot.Con.Child(:,i))'
        M_hatii=M_hat(1:6,1:6,j)-psi_hat(1:6,j)*psi(1:6,j)';
        M_hat(1:6,1:6,i)=M_hat(1:6,1:6,i)+Bij(1:6,1:6,j,i)'*M_hatii*Bij(1:6,1:6,j,i);
    end
    psi_hat(1:6,i)=M_hat(1:6,1:6,i)*pm(1:6,i);
    psi(1:6,i)=psi_hat(1:6,i)/(pm(1:6,i)'*psi_hat(1:6,i));
end
%Base-spacecraft
M_hat0=[I0,zeros(3,3);zeros(3,3),robot.base_link.mass*eye(3)];
%Add children contributions
for j=find(robot.Con.Child_base)'
    M_hat0ii=M_hat(1:6,1:6,j)-psi_hat(1:6,1)*psi(1:6,j)';
    M_hat0=M_hat0+Bi0(1:6,1:6,j)'*M_hat0ii*Bi0(1:6,1:6,j);
end
psi_hat0=M_hat0*P0;

%--- eta ---
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate and initialize
    eta=zeros(6,n);
    phi_hat=zeros(n);
    phi_tilde=zeros(n);
end

%Backwards recursion
for i=n:-1:1
    %Initialize
    eta(1:6,i)=zeros(6,1);
    %Add children contributions
    for j=find(robot.Con.Child(:,i))'
        eta(1:6,i)=eta(1:6,i)+Bij(1:6,1:6,j,i)'*(psi(1:6,j)*phi_hat(j)+eta(1:6,j));
        
    end
    phi_hat(i)=phi(i)-pm(1:6,i)'*eta(1:6,i);
    phi_tilde(i)=phi_hat(i)/(pm(1:6,i)'*psi_hat(1:6,i));
end
%Base-spacecraft
eta0=zeros(6,1);
%Add children contributions
for j=find(robot.Con.Child_base)'
    eta0=eta0+Bi0(1:6,1:6,j)'*(psi(1:6,j)*phi_hat(j)+eta(1:6,j));
end
phi_hat0=phi0-P0'*eta0;
phi_tilde0=(P0'*psi_hat0)\phi_hat0;


%--- Base-spacecraft acceleration ---%
q0ddot=phi_tilde0;

%--- Manipulator acceleration (and mu) ---%
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    mu=zeros(6,n);
    qmddot=zeros(n,1);
end

%Forward recursion
for i=2:n
    
    if robot.joints(i-1).parent_link==0
        %First joint
        mu(1:6,i)=Bi0(1:6,1:6,i)*(P0*q0ddot);
    else
        %Rest of the links
        if robot.joints(i-1).type~=0
            mu_aux=(pm(1:6,i-1)*qmddot(robot.joints(i-1).q_id)+mu(1:6,i-1));
        else
            mu_aux=mu(1:6,i-1);
        end
        mu(1:6,i)=Bij(1:6,1:6,i,i-1)*mu_aux;
    end
    
    %Initialize
    if robot.joints(i).type~=0
        qmddot(robot.joints(i).q_id)=phi_tilde(i)-psi(1:6,i)'*mu(1:6,i);
    end
end

end