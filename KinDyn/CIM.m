function [C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot) %#codegen
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
%   robot -> Robot model
%
% Output ->
%   C0 -> Base-spacecraft convective inertia matrix.
%   C0m -> Base-spacecraft - manipulator coupling convective inertia matrix.
%   Cm0 -> Manipulator - Base-spacecraft coupling convective inertia matrix.
%   Cm -> Manipulator convective inertia matrix.

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
n=robot.n_links_joints;

%--- Omega ---%
%Base-spacecraft Omega
Omega0=[SkewSym(t0(1:3)), zeros(3,3);
    zeros(3,3), SkewSym(t0(1:3))];
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate Omega
    Omega=zeros(6,6,n);
end
%Compute Omega
for i=1:n
    Omega(1:6,1:6,i)=[SkewSym(tm(1:3,i)), zeros(3,3);
        zeros(3,3), SkewSym(tm(1:3,i))];
end

%--- Mdot ---%
%Base-spacecraft Mdot
Mdot0=[Omega0(1:3,1:3)*I0, zeros(3,3); zeros(3,3), zeros(3,3)];
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    Mdot=zeros(6,6,n);
end
%Compute Mdot
for i=1:n
    Mdot(1:6,1:6,i)=[Omega(1:3,1:3,i)*Im(1:3,1:3,i), zeros(3,3); zeros(3,3), zeros(3,3)];
end

%--- Mdot tilde ---%
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-Allocate
    Mdot_tilde=zeros(6,6,n);
end
%Backwards recursion
for i=n:-1:1
    %Initialize
    Mdot_tilde(1:6,1:6,i)=Mdot(1:6,1:6,i);
    %Add children contributions
    child=find(robot.con.child(:,i))';
    for j=1:length(child)
        Mdot_tilde(1:6,1:6,i)=Mdot_tilde(1:6,1:6,i)+Mdot_tilde(1:6,1:6,child(j));
    end
end
%Base-spacecraft
Mdot0_tilde = Mdot0;
%Add children contributions
child=find(robot.con.child_base)';
for j=1:length(child)
    Mdot0_tilde = Mdot0_tilde+Mdot_tilde(1:6,1:6,child(j));
end


%--- Bdot ---%

%Pre-allocate Bdotij
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    Bdotij=zeros(6,6,n,n);
end

%Compute Bdotij
for j=1:n
    for i=1:n
        if robot.con.branch(i,j)==1
            %Links are in the same branch
            Bdotij(1:6,1:6,i,j)=[zeros(3,3), zeros(3,3); SkewSym(tm(4:6,j)-tm(4:6,i)), zeros(3,3)];
        else
            %Links are not in the same branch
            Bdotij(1:6,1:6,i,j)=zeros(6,6);
        end
    end
end


%--- Hij tilde ---%
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate Hij_tilde
    Hij_tilde=zeros(6,6,n,n);
end
%Hij_tilde
for i=n:-1:1
    for j=n:-1:1
        Hij_tilde(1:6,1:6,i,j)=Mm_tilde(1:6,1:6,i)*Bdotij(1:6,1:6,i,j);
        %Add children contributions
        child=find(robot.con.child(:,i))';
        for k=1:length(child)
            Hij_tilde(1:6,1:6,i,j)=Hij_tilde(1:6,1:6,i,j)+Bij(1:6,1:6,child(k),i)'*Hij_tilde(1:6,1:6,child(k),i);
        end
    end
end
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate Hi0_tilde and H0j_tilde
    Hi0_tilde=zeros(6,6,n);
end
%Hi0_tilde
for i=n:-1:1
    Bdot=[zeros(3,3), zeros(3,3); SkewSym(t0(4:6)-tm(4:6,i)), zeros(3,3)];
    Hi0_tilde(1:6,1:6,i)=Mm_tilde(1:6,1:6,i)*Bdot;
    %Add children contributions
    child=find(robot.con.child(:,i))';
    for k=1:length(child)
        Hi0_tilde(1:6,1:6,i)=Hi0_tilde(1:6,1:6,i)+Bij(1:6,1:6,child(k),i)'*Hij_tilde(1:6,1:6,child(k),i);
    end
end

%--- C Matrix ---%
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    Cm=zeros(n,n);
    C0m=zeros(6,n);
    Cm0=zeros(n,6);
end
%Cm Matrix
for j=1:n
    for i=1:n
        %Joints must not be fixed and links on the same branch
        if (robot.joints(i).type~=0 && robot.joints(j).type~=0) && (robot.con.branch(i,j)==1 || robot.con.branch(j,i)==1)
            %Compute Cm matrix
            if i<=j
                %Add children contributions
                child_con=zeros(6,6);
                child=find(robot.con.child(:,j))';
                for k=1:length(child)
                    child_con=child_con+Bij(1:6,1:6,child(k),i)'*Hij_tilde(1:6,1:6,child(k),j);
                end
                Cm(robot.joints(i).q_id,robot.joints(j).q_id)=pm(1:6,i)'*(Bij(1:6,1:6,j,i)'*Mm_tilde(1:6,1:6,j)*Omega(1:6,1:6,j)+child_con+Mdot_tilde(1:6,1:6,j))*pm(1:6,j);
            else
                Cm(robot.joints(i).q_id,robot.joints(j).q_id)=pm(1:6,i)'*(Mm_tilde(1:6,1:6,i)*Bij(1:6,1:6,i,j)*Omega(1:6,1:6,j)+Hij_tilde(1:6,1:6,i,j)+Mdot_tilde(1:6,1:6,i))*pm(1:6,j);
            end
        end
    end
end
%C0 matrix
%Add children contributions
child_con=zeros(6,6);
child=find(robot.con.child_base)';
for k=1:length(child)
    child_con=child_con+Bi0(1:6,1:6,child(k))'*Hi0_tilde(1:6,1:6,child(k));
end
C0 = P0'*(M0_tilde*Omega0+child_con+Mdot0_tilde)*P0;
%C0m
for j=1:n
    if  robot.joints(j).type~=0
        if j==n
            C0m(1:6,robot.joints(j).q_id)=P0'*(Bi0(1:6,1:6,j)'*Mm_tilde(1:6,1:6,j)*Omega(1:6,1:6,j)+Mdot_tilde(1:6,1:6,j))*pm(1:6,j);
        else
            %Add children contributions
            child_con=zeros(6,6);
            child=find(robot.con.child(:,j))';
            for k=1:length(child)
                child_con=child_con+Bi0(1:6,1:6,child(k))'*Hij_tilde(1:6,1:6,child(k),j);
            end
            C0m(1:6,robot.joints(j).q_id)=P0'*(Bi0(1:6,1:6,j)'*Mm_tilde(1:6,1:6,j)*Omega(1:6,1:6,j)+child_con+Mdot_tilde(1:6,1:6,j))*pm(1:6,j);
        end
    end
end
%Cm0
for i=1:n
    if  robot.joints(i).type~=0  
        Cm0(robot.joints(i).q_id,1:6)=pm(1:6,i)'*(Mm_tilde(1:6,1:6,i)*Bi0(1:6,1:6,i)*Omega0+Hi0_tilde(1:6,1:6,i)+Mdot_tilde(1:6,1:6,i))*P0;
    end
end

end

