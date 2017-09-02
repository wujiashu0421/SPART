function [C]=CIM_NOC(N,Ndot,t0,tL,I0,Im,robot)
% Computes the Generalized Convective Inertia Matrix of a Serial Manipulator.
%
% Input ->
%   N -> Natural Orthogonal Complement matrix.
%   N -> Natural Orthogonal Complement matrix time derivative.
%   t0 -> Inertial twist of the base-spacecraft.
%   tL -> Inertial twist of the links.
%   I0 -> Base-spacecraft inertia in inertial frame.
%   Im -> Manipulator inertia in inertial frame.
%   robot -> Robot model.
%
% Output ->
%   C -> Generalized convective inertia matrix.

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

%=== Code ===%

%--- Generalized mass matrix ---%

%Pre-allocate M
M=zeros(6+6*robot.n_links_joints,6+6*robot.n_links_joints,'like',N);

%Base contribution
M(1:6,1:6)=[I0(1:3,1:3),zeros(3,3);zeros(3,3),robot.base_link.mass*eye(3)];
%Manipulator contribution
for i=1:robot.n_links_joints
    M(6+6*i-5:6+6*i,6+6*i-5:6+6*i)=[Im(1:3,1:3,i),zeros(3,3);zeros(3,3),robot.links(i).mass*eye(3)];
end

%--- Omega ---%
%Base-spacecraft Omega
Omega0=[SkewSym(t0(1:3)), zeros(3,3);
    zeros(3,3), SkewSym(t0(1:3))];

%Pre-allocate Omega
Omega=zeros(6,6,robot.n_links_joints,'like',N);

%Compute Omega
for j=1:i
    Omega(1:6,1:6,j)=[SkewSym(tL(1:3,j)), zeros(3,3);
        zeros(3,3), SkewSym(tL(1:3,j))];
end

%--- Mdot ---%
%Pre-allocate
Mdot=zeros(6+6*robot.n_links_joints,6+6*robot.n_links_joints,'like',N);

%Base-spacecraft Mdot
Mdot(1:6,1:6)=[Omega0(1:3,1:3)*I0, zeros(3,3); zeros(3,3), zeros(3,3)];
%Compute Mdot
for i=1:robot.n_links_joints
    Mdot(6+6*i-5:6+6*i,6+6*i-5:6+6*i)=[Omega(1:3,1:3,i)*Im(1:3,1:3,i), zeros(3,3); zeros(3,3), zeros(3,3)];
end

%--- Convective Inertia Matrix ---%
C=N'*(M*Ndot+Mdot*N);

end