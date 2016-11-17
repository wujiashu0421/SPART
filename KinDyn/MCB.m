function [M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot) %#codegen
% Computes the Mass Composite Body matrix of a manipulator.
%
% Input ->
%   I0 -> Base-spacecraft inertia in inertial frame.
%   Im -> Manipulator inertia in inertial frame.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   robot -> Robot model.
%
% Output ->
%   M0_tilde -> Base-spacecraft mass matrix of composite body.
%   Mm_tilde -> Manipulator mass matrix of composite body.

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
%Number of links and Joints
n=robot.n_links;
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    Mm_tilde=zeros(6,6,n);
end

%Backwards recursion
for i=n:-1:1
    %Initialize M tilde
    Mm_tilde(1:6,1:6,i)=[Im(1:3,1:3,i),zeros(3,3);zeros(3,3),robot.links(i).mass*eye(3)];%+Bij(1:6,1:6,i+1,i)'*Mm_tilde(1:6,1:6,i+1)*Bij(1:6,1:6,i+1,i);
    %Add children contributions
    child=find(robot.Con.Child(:,i))';
    for j=1:length(child)
        Mm_tilde(1:6,1:6,i)=Mm_tilde(1:6,1:6,i)+Bij(1:6,1:6,child(j),i)'*Mm_tilde(1:6,1:6,child(j))*Bij(1:6,1:6,child(j),i);
    end
end

%Base-spacecraft M tilde
M0_tilde=[I0,zeros(3,3);zeros(3,3),robot.base_link.mass*eye(3)];
%Add children contributions
child=find(robot.Con.Child_base)';
for j=1:length(child);
    M0_tilde=M0_tilde+Bi0(1:6,1:6,child(j))'*Mm_tilde(1:6,1:6,child(j))*Bi0(1:6,1:6,child(j));
end


end