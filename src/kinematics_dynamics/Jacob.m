function [J0, Jm]=Jacob(rx,r0,rL,P0,pm,i,robot)
% Computes the Jacobian of the rx point.
%
% [J0, Jm]=Jacob(rxi,r0,rL,P0,pm,i,robot)
% 
% :parameters: 
%   * rx -- Inertial position of the point of interest -- [3x1].
%   * r0 -- Inertial position of the base-spacecraft -- [3x1].
%   * rL -- Links inertial positions -- [3xn].
%   * P0 -- Base-spacecraft twist-propagation vector -- [6x6].
%   * pm -- Manipulator twist-propagation vector -- [6x1].
%   * i -- Link where the point `x` is located -- int 0 to n.
%   * robot -- Robot model (see :doc:`/Robot_Model`).
%
% :return: 
%   * J0 -- Base-spacecraft Jacobian --[6x6].
%   * Jm -- Manipulator Jacobian -- [6xn_q].
%
% Examples:
%
%   To compute the velocity of a point ``rx`` on the ith link:
%
% .. code-block:: matlab
%   
%   %Compute Jacobians
%   [J0, Jm]=Jacob(rx,r0,rL,P0,pm,i,robot);
%   %Twist of that point
%   tx=J0*q0dot+Jm*qmdot;
%
% See also: :func:`src.kinematics_dynamics.Kinematics`, :func:`src.kinematics_dynamics.DiffKinematics`

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

%--- Number of links  ---%

%Base Jacobian
J0=[eye(3),zeros(3,3);SkewSym(r0-rx),eye(3)]*P0;

%Pre-allocate
Jm=zeros(6,robot.n_q,'like',rx);

%Manipulator Jacobian
joints_num=0;
for j=1:i
    %If joint is not fixed
    if robot.joints(j).type~=0
        if robot.con.branch(i,j)==1
            Jm(1:6,robot.joints(j).q_id)=[eye(3),zeros(3,3);SkewSym(rL(1:3,j)-rx),eye(3)]*pm(1:6,j);
        else
            Jm(1:6,robot.joints(j).q_id)=zeros(6,1);
        end
        joints_num=joints_num+1;
    end
end

end