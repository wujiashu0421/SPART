function [tau0,taum] = ID(wF0,wFm,t0,tL,t0dot,tLdot,P0,pm,I0,Im,Bij,Bi0,robot)
% This function solves the inverse dynamics (ID) problem (it obtains the
% generalized forces from the accelerations) for a manipulator.
%
% [tau0,taum] = ID(wF0,wFm,t0,tL,t0dot,tLdot,P0,pm,I0,Im,Bij,Bi0,robot)
% 
% :parameters: 
%   * wF0 -- External forces on the base-spacecraft (in the inertial frame) [Tx,Ty,Tz,fx,fy,fz] -- [6x1].
%   * wFm -- External forces on the manipulator links CoM (in the inertial frame) [Tx,Ty,Tz,fx,fy,fz] -- [6x1].
%   * t0 -- Base-spacecraft twist vector [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6x1].
%   * tL -- Manipulator twist vector [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6xn].
%   * t0dot -- Base-spacecraft twist-rate vector [alphax,alphay,alphaz,ax,ay,az] (all in inertial frame) -- [6x1].
%   * tLdot -- Manipulator twist-rate vector [alphax,alphay,alphaz,ax,ay,az] (all in inertial frame) -- [6xn].
%   * P0 -- Base-spacecraft twist-propagation vector -- [6x6].
%   * pm -- Manipulator twist-propagation vector -- [6x1].
%   * I0 -- Base-spacecraft inertia matrix in the inertial frame -- [3x3].
%   * Im -- Links inertia matrices in the inertial frame -- [3x3xn].
%   * Bij -- Twist-propagation matrix (for manipulator i>0 and j>0) -- [6x6xn].
%   * Bi0 -- Twist-propagation matrix (for i>0 and j=0) -- [6x6xn].
%   * robot -- Robot model (see :doc:`/Robot_Model`).
%
% :return: 
%   * tau0 -- Base-spacecraft forces [Tx,Ty,Tz,fx,fy,fz] (torques in the body frame) -- [6x1].
%   * taum -- Joint forces/troques -- [n_qx1].
%
% See also: :func:`src.kinematics_dynamics.Floating_ID` and :func:`src.kinematics_dynamics.FD`. 


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

%--- Mdot ---%
%Base-spacecraft Mdot
Mdot0=[SkewSym(t0(1:3))*I0, zeros(3,3); zeros(3,3), zeros(3,3)];

%Pre-allocate
Mdot=zeros(6,6,n,'like',wF0);

%Manipulator Mdot
for i=1:n
    Mdot(1:6,1:6,i)=[SkewSym(tL(1:3,i))*Im(1:3,1:3,i), zeros(3,3); zeros(3,3), zeros(3,3)];
end

%--- Forces ---%
%Base-spacecraft
wq0=[I0,zeros(3,3);zeros(3,3),robot.base_link.mass*eye(3)]*t0dot+Mdot0*t0-wF0;

%Pre-allocate
wq=zeros(6,n,'like',wF0);

%Manipulator
for i=1:n
    wq(1:6,i)=[Im(1:3,1:3,i),zeros(3,3);zeros(3,3),robot.links(i).mass*eye(3)]*tLdot(1:6,i)+Mdot(1:6,1:6,i)*tL(1:6,i)-wFm(1:6,i);
end

%Pre-allocate
wq_tilde=zeros(6,n,'like',wF0);

%Backwards recursion
for i=n:-1:1
    %Initialize wq_tilde
    wq_tilde(1:6,i)=wq(1:6,i);
    %Add children contributions
    for j=find(robot.con.child(:,i))'
        wq_tilde(1:6,i)=wq_tilde(1:6,i)+Bij(1:6,1:6,j,i)'*wq_tilde(1:6,j);
    end
end
%Base-spacecraft
wq_tilde0=wq0;
%Add children contributions
for j=find(robot.con.child_base)'
    wq_tilde0=wq_tilde0+Bi0(1:6,1:6,j)'*wq_tilde(1:6,j);
end

%---- Joint forces ---%
%Base-spacecraft
tau0=P0'*wq_tilde0;

%Pre-allocate
taum=zeros(robot.n_q,1,'like',wF0);

%Manipulator joint forces.
for i=1:n
    if robot.joints(i).type~=0
        taum(robot.joints(i).q_id,1)=pm(1:6,i)'*wq_tilde(1:6,i);
    end
end

end