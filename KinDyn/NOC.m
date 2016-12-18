function [N] = NOC(r0,rL,P0,pm,robot)
% Computes the Natural Orthogonal Complement (NOC) matrix
%
% Input ->
%   r0 -> Inertial position of the base-spacecraft.
%   rL -> Links inertial positions.
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   robot -> Robot model.
%
% Output ->
%   N -> Natural Orthogonal Complement (NOC) matrix

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
    %Pre-allocate NOC
    N=zeros(6+6*robot.n_links_joints,6+robot.n_q);
end

%Base-spacecraft contribution
N(1:6,1:6+robot.n_links_joints)=[P0,zeros(6,robot.n_links_joints)]; 

%Manipulator contribution
for i=1:robot.n_links_joints
    %Jacobian
    [J0i, Jmi]=Jacob(rL(1:3,i),r0,rL,P0,pm,i,robot);
    %Append Jacobian to the NOC
    N(6+6*i-5:6+6*i,1:6+robot.n_q)=[J0i,Jmi];
end


end