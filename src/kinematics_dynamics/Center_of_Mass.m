function r_com = Center_of_Mass(r0,rL,robot)
% Computes the center-of-mass (CoM) of the system.
%
% r_com = Center_of_Mass(r0,rL,robot)
%
% :parameters: 
%   * r0 -- Position of the base-spacecraft with respect to the inertial frame -- [3x1].
%   * rL -- Links center-of-mass positions -- [3xn].
%   * robot -- Robot model (see :doc:`/Robot_Model`).
%
% :retruns:
%   * r_com -- Location of the center-of-mass -- [3x1].
%
% Use :func:`src.kinematics_dynamics.Kinematics` to compute the ``rL`` parameter.
%
% This function can also be used to compute the velocity/acceleration of the center-of-mass.
% To do it use as paremeters the velocities ``v0,vL`` or acceleration ``a0,am`` and you will get the CoM  velocity ``v_com`` or acceleration ``a_com``.
%
% See also: :func:`src.kinematics_dynamics.Kinematics`

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

%Initialize total mass and total mass * distance varables
mass_total=robot.base_link.mass;
mass_r=r0*robot.base_link.mass;

%Add contribution of manipulator links
for i=1:robot.n_links_joints
    mass_total=mass_total+robot.links(i).mass;
    mass_r = mass_r+rL(1:3,i).*robot.links(i).mass; 
end

%Compute center of mass
r_com=mass_r./mass_total;

end