function [m] = TotalMass(robot)
% Computes the total mass of the system.
%
% [m] = TotalMass(robot)
%
% :parameters: 
%   * robot -- Robot model (see :doc:`/Tutorial_Robot`).
%
% :return:
%   * m -- Total mass of the system.
%
% See also: :func:`src.utils.NomInertia`

%{  
    LICENSE

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
%}

%=== CODE ===%

%Initialize total mass
m=robot.base_link.mass;

%Add contributions of links
for i=1:robot.n_links_joints
    m=m+robot.links(i).mass;
end

end