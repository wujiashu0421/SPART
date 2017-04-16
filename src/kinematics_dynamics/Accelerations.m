function [t0dot,tLdot]=Accelerations(t0,tL,P0,pm,Bi0,Bij,q0dot,qmdot,q0ddot,qmddot,robot)
% Computes the accelerations (twist-rate) of the multibody system.
%
% [t0dot,tLdot]=Accelerations(t0,tL,P0,pm,Bi0,Bij,q0dot,qmdot,q0ddot,qmddot,robot)
%
% :parameters: 
%   * t0 -- Base-spacecraft twist vector [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6x1].
%   * tL -- Manipulator twist vector [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6xn].
%   * Bij -- Twist-propagation matrix (for manipulator i>0 and j>0) -- [6x6xn].
%   * Bi0 -- Twist-propagation matrix (for i>0 and j=0) -- [6x6xn].
%   * P0 -- Base-spacecraft twist-propagation vector -- [6x6].
%   * pm -- Manipulator twist-propagation vector -- [6x1].
%   * q0dot -- Base-spacecraft velocities [wx,wy,wz,vx,vy,vz]. The angular velocities are in body axis, while the linear velocities in inertial frame -- [6x1].
%   * qmdot -- Joint velocities -- [n_qx1].
%   * q0ddot -- Base-spacecraft accelerations [alphax,alphay,alphaz,ax,ay,az]. The angular accelerations are in body axis, while the linear accelerations are in inertial frame -- [6x1].
%   * qmddot -- Manipulator joint accelerations -- [n_qx1].
%   * robot -- Robot model (see :doc:`/Robot_Model`).
%
% :return: 
%   * t0dot -- Base-spacecraft twist-rate vector [alphax,alphay,alphaz,ax,ay,az] (all in inertial frame) -- [6x1].
%   * tLdot -- Manipulator twist-rate vector [alphax,alphay,alphaz,ax,ay,az] (all in inertial frame) -- [6xn].
%
% See also: :func:`src.kinematics_dynamics.Jacobdot`. 

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

%--- Omega matrices ---%
%Base-Spacecraft
Omega0=[SkewSym(t0(1:3)), zeros(3,3);
    zeros(3,3), zeros(3,3)];
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    Omegam=zeros(6,6,n);
end
%Compute Omega for manipulator
for i=1:n
    Omegam(1:6,1:6,i)=[SkewSym(tL(1:3,i)), zeros(3,3);
        zeros(3,3), SkewSym(tL(1:3,i))];
end

%--- Twist Rate ---%
%Base-spacecraft
t0dot = Omega0*P0*q0dot+P0*q0ddot;
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    tLdot=zeros(6,n);
end
%Forward recursion
for i=1:n
    
    if robot.joints(i).parent_link==0
        %First Link
        tLdot(1:6,i)=Bi0(1:6,1:6,i)*t0dot+[zeros(3,6);SkewSym(t0(4:6)-tL(4:6,i)),zeros(3,3)]*t0;
    else
        %Rest of the links
        tLdot(1:6,i)=Bij(1:6,1:6,i,robot.joints(i).parent_link)*tLdot(1:6,robot.joints(i).parent_link)+[zeros(3,6); SkewSym(tL(4:6,robot.joints(i).parent_link)-tL(4:6,i)), zeros(3,3)]*tL(1:6,robot.joints(i).parent_link);
    end
    
    %Add joint contribution
    if robot.joints(i).type~=0
        tLdot(1:6,i)=tLdot(1:6,i)+Omegam(1:6,1:6,i)*pm(1:6,i)*qmdot(robot.joints(i).q_id)+pm(1:6,i)*qmddot(robot.joints(i).q_id);
    end
    
    
end


end