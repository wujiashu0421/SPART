function [Ndot] = NOCdot(r0,t0,rL,tL,P0,pm,robot)
% Computes the Natural Orthogonal Complement (NOC) matrix time derivative
%
% Input ->
%   r0 -> Inertial position of the base-spacecraft.
%   t0 -> Inertial twist of the base-spacecraft.
%   rL -> Links inertial positions.
%   tL -> Inertial twist of the links.
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   robot -> Robot model.
%
% Output ->
%   Ndot -> Natural Orthogonal Complement (NOC) matrix time derivative

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

%Pre-allocate
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate NOCdot
    Ndot=zeros(6+6*robot.n_links_joints,6+robot.n_q);
end

%Compute the NOC time derivative matrix by using the Jacobians time derivative
%Base-spacecraft contribution
Omega0=[SkewSym(t0(1:3)), zeros(3,3);
    zeros(3,3), zeros(3,3)];
Ndot(1:6,1:6+robot.n_q)=[Omega0*P0,zeros(6,robot.n_q)];
%Manipulator contribution
for i=1:robot.n_links_joints
    [J0doti, Jmdoti]=Jacobdot(rL(1:3,i),tL(1:6,i),r0,t0,rL,tL,P0,pm,i,robot);
    Ndot(6+6*i-5:6+6*i,1:6+robot.n_q)=[J0doti,Jmdoti];
end


end