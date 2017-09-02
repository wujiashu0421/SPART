function [t0,tL]=Velocities(Bij,Bi0,P0,pm,q0dot,qmdot,robot)
% Computes the velocities of the links.
%
% [t0,tL]=Velocities(Bij,Bi0,P0,pm,q0dot,qmdot,robot)
%
% :parameters:
%   * Bij -- Twist-propagation matrix (for manipulator i>0 and j>0) -- [6x6xn].
%   * Bi0 -- Twist-propagation matrix (for i>0 and j=0) -- [6x6xn].
%   * P0 -- Base-spacecraft twist-propagation vector -- [6x6].
%   * pm -- Manipulator twist-propagation vector -- [6x1].
%   * q0dot -- Base-spacecraft velocities [wx,wy,wz,vx,vy,vz]. The angular velocities are in body axis, while the linear velocities in inertial frame -- [6x1].
%   * qmdot -- Joint velocities -- [n_qx1].
%   * robot -- Robot model (see :doc:`/Robot_Model`).
%
% :return:
%   * t0 -- Base-spacecraft twist vector [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6x1].
%   * tL -- Manipulator twist vector [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6xn].
%
% Use :func:`src.kinematics_dynamics.DiffKinematics` to compute the ``Bij,Bi0,P0,pm`` parameters.
%
% See also: :func:`src.kinematics_dynamics.Jacob`


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

%--- Number of links and Joints ---%
n=robot.n_links_joints;

%Pre-Allocate
tL=zeros(6,n,'like',Bij);

%Base-spacecraft
t0=P0*q0dot;

%Fordward recursion to obtain the twist vector
for i=1:n
    
    if robot.joints(i).parent_link==0
        %First link
        tL(1:6,i)=Bi0(1:6,1:6,i)*t0;
    else
        %Rest of the links
        tL(1:6,i)=Bij(1:6,1:6,i,i-1)*tL(1:6,i-1);
    end
    
    %Add joint contribution
    if robot.joints(i).type~=0
        tL(1:6,i)=tL(1:6,i)+pm(1:6,i)*qmdot(robot.joints(i).q_id);
    end
    
end

end







