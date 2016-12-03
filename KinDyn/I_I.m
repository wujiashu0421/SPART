function [I0,Im]=I_I(R0,RL,robot) %#codegen
% Converts the inertias in local frame to inertia in the inertial frame.
%
% Input ->
%   R0 -> Rotation matrix from the base-spacecraft to the inertial frame.
%   RL -> Links 3x3 rotation matrices.
%   robot -> Robot model.
%
% Output ->
%   I0 -> Base-spacecraft inertia in inertial frame.
%   Im -> Manipulator inertia in inertial frame.

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

%Base-spacecraft inertia
I0 = R0*robot.base_link.inertia;
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate inertias
    Im=zeros(3,3,robot.n_links_joints);
end
%Inertias of the links
for i=1:(robot.n_links_joints)
    Im(1:3,1:3,i)=RL(1:3,1:3,i)*robot.links(i).inertia*RL(1:3,1:3,i)';
end

end
