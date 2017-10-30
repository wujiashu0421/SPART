function [tauqm,u0dot] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0,um,umdot,robot)
% This function solves the inverse dynamics problem (it obtains the
% generalized forces from the accelerations) for a manipulator with 
% a floating base.
%
% [tauqm,u0dot] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0,um,umdot,robot)
%
% :parameters: 
%   * wF0 -- External forces on the base-spacecraft (in the inertial frame) [Tx,Ty,Tz,fx,fy,fz] -- [6x1].
%   * wFm -- External forces on the manipulator links CoM (in the inertial frame) [Tx,Ty,Tz,fx,fy,fz] -- [6x1].
%   * M0_tilde -- Base-spacecraft mass composite body matrix -- [6x6].
%   * Mm_tilde -- Manipulator mass composite body matrix -- [6x6xn].
%   * t0 -- Base-spacecraft twist vector [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6x1].
%   * tL -- Manipulator twist vector [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6xn].
%   * pm -- Manipulator twist-propagation vector -- [6x1].
%   * I0 -- Base-spacecraft inertia matrix in the inertial frame -- [3x3].
%   * Im -- Links inertia matrices in the inertial frame -- [3x3xn].
%   * Bij -- Twist-propagation matrix (for manipulator i>0 and j>0) -- [6x6xn].
%   * Bi0 -- Twist-propagation matrix (for i>0 and j=0) -- [6x6xn].
%   * u0 -- Base-spacecraft velocities [wx,wy,wz,vx,vy,vz]. The angular velocities are in body axis, while the linear velocities in inertial frame -- [6x1].
%   * um -- Joint velocities -- [n_qx1].
%   * umdot -- Manipulator joint accelerations -- [n_qx1].
%   * robot -- Robot model (see :doc:`/Robot_Model`).
%
% :return: 
%   * tauq0 -- Base-spacecraft forces [Tx,Ty,Tz,fx,fy,fz] (torques in the body frame) -- [6x1].
%   * tauqm -- Joint forces/troques -- [n_qx1].
%
% See also: :func:`src.kinematics_dynamics.sID` and :func:`src.kinematics_dynamics.FD`. 

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

%Number of links and Joints
n=robot.n_links_joints;

%Recompute Accelerations with qddot=0
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,u0,um,zeros(6,1),umdot,robot);

%Use the inverse dynamics
[tauq0_0ddot,tauqm] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robot);

%Pre-allocate kappa
kappa=zeros(n,6,'like',wF0);

%Compute Kappa
for i=1:n
    kappa(i,1:6)=pm(1:6,i)'*Mm_tilde(1:6,1:6,i)*Bi0(1:6,1:6,i);
end

%Compute base-spacecraft acceleration
u0dot=-H0\tauq0_0ddot;

%Update joint forces
for i=1:n
    if robot.joints(i).type~=0
        tauqm(robot.joints(i).q_id)=kappa(i,1:6)*P0*u0dot+tauqm(robot.joints(i).q_id);
    end
end

end
