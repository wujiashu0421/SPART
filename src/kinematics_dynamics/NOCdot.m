function [Ndot] = NOCdot(r0,t0,rL,tL,P0,pm,robot)
% Computes the Natural Orthogonal Complement (NOC) matrix time-derivative.
%
% [Ndot] = NOCdot(r0,t0,rL,tL,P0,pm,robot)
%
% :parameters: 
%   * r0 -- Position of the base-spacecraft with respect to the inertial frame -- [3x1].
%   * t0 -- Base-spacecraft twist vector [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6x1].
%   * rL -- Links center-of-mass positions -- [3xn].
%   * tL -- Manipulator twist vector [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6xn].%   * P0 -- Base-spacecraft twist-propagation vector -- [6x6].
%   * pm -- Manipulator twist-propagation vector -- [6xn].
%   * robot -- Robot model (see :doc:`/Robot_Model`).
%
% :return: 
%   * Ndot -- Natural Orthogonal Complement (NOC) matrix time-derivative -- [6+6*n,6+n_q].
%
% Examples:
%
%   To compute the accelerations of all links:
%
% .. code-block:: matlab
%	
%   %Compute NOC
%   [N] = NOC(r0,rL,P0,pm,robot)
%   %Compute NOC time-derivative
%   [Ndot] = NOCdot(r0,t0,rL,tL,P0,pm,robot)
%   %Twist time-derivatives of all the links
%   tdot=N*[u0dot;umdot]+Ndot*[u0;um];
%   %Twist time-derivative of the base-spacecraft
%   t0dot=tdot(1:6,1);
%   %Twist time-derivative of the ith link
%   i=2;
%   tidot=tdot(6*i:6+6*i,1);
%
% See also: :func:`src.kinematics_dynamics.Jacobdot` and :func:`src.kinematics_dynamics.NOC`. 


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

%=== Code ===%

%Pre-allocate NOCdot
Ndot=zeros(6+6*robot.n_links_joints,6+robot.n_q,'like',r0);

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