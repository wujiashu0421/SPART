function [H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot)
% Computes the Generalized Inertia Matrix H of the multibody vehicle.
% This function uses a recursive algorithm.
%
% [H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot)
%
% :parameters: 
%   * M0_tilde -- Base-spacecraft mass composite body matrix -- [6x6].
%   * Mm_tilde -- Manipulator mass composite body matrix -- [6x6xn].
%   * Bij -- Twist-propagation matrix (for manipulator i>0 and j>0) -- [6x6xn].
%   * Bi0 -- Twist-propagation matrix (for i>0 and j=0) -- [6x6xn].
%   * P0 -- Base-spacecraft twist-propagation vector -- [6x6].
%   * pm -- Manipulator twist-propagation vector -- [6xn].
%   * robot -- Robot model (see :doc:`/Robot_Model`).
%
% :return: 
%   * H0 -- Base-spacecraft inertia matrix -- [6x6].
%   * H0m -- Base-spacecraft -- manipulator coupling inertia matrix -- [6xn_q].
%   * Hm -- Manipulator inertia matrix -- [n_qxn_q].
%   
% To obtain the full generalized inertia matrix H:
%
% .. code-block:: matlab
%   
%   %Compute H
%   [H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
%   H=[H0,H0m;H0m';Hm];
%
% See also: :func:`src.kinematics_dynamics.CIM`.


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
n_q=robot.n_q;
n=robot.n_links_joints;

%--- H Martix ---%
%Base-spacecraft Inertia matrix
H0 = P0'*M0_tilde*P0;
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate Hm
    Hm=zeros(n_q,n_q);
end
%Manipulator Inertia matrix Hm
for j=1:n
    for i=j:n
        if robot.joints(i).type~=0 && robot.joints(j).type~=0
            Hm(robot.joints(i).q_id,robot.joints(j).q_id)=pm(1:6,i)'*Mm_tilde(1:6,1:6,i)*Bij(1:6,1:6,i,j)*pm(1:6,j);
            Hm(robot.joints(j).q_id,robot.joints(i).q_id)=Hm(robot.joints(i).q_id,robot.joints(j).q_id);
        end
    end
end
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate H0m
    H0m=zeros(6,n_q);
end
%Coupling Inertia matrix
for i=1:n
    if robot.joints(i).type~=0
        H0m(1:6,robot.joints(i).q_id)=(pm(1:6,i)'*Mm_tilde(1:6,1:6,i)*Bi0(1:6,1:6,i)*P0)';
    end
end

end