function [J0dot, Jmdot]=Jacobdot(rx,tx,r0,t0,rL,tL,P0,pm,i,robot)
% Computes the Jacobian time-derivative of the rx point.
%
% [J0dot, Jmdot]=Jacobdot(rx,tx,r0,t0,rL,tL,P0,pm,i,robot)
%
% :parameters: 
%   * rx -- Inertial position of the point of interest -- [3x1].
%   * tx -- Twist of the point of interest [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6x1].
%   * r0 -- Inertial position of the base-spacecraft -- [3x1].
%   * t0 -- Base-spacecraft twist vector [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6x1].
%   * rL -- Links inertial positions -- [3xn].
%   * tL -- Manipulator twist vector [wx,wy,wz,vx,vy,vz] (all in inertial frame) -- [6xn].
%   * P0 -- Base-spacecraft twist-propagation vector -- [6x6].
%   * pm -- Manipulator twist-propagation vector -- [6xn].
%   * i -- Link where the point `x` is located -- int 0 to n.
%   * robot -- Robot model (see :doc:`/Robot_Model`).
%
% :return: 
%   * J0dot -- Base-spacecraft Jacobian time-derivative --[6x6].
%   * Jmdot -- Manipulator Jacobian time-derivative --[6xn_q].
%
% Examples:
%
%   To compute the acceleration of a point ``rx`` on the ith link:
%
% .. code-block:: matlab
%   
%   %Compute Jacobians
%   [J0, Jm]=Jacob(rx,r0,rL,P0,pm,i,robot);
%   Compute Jacobians time-derivatives
%   [J0dot, Jmdot]=Jacobdot(rx,tx,r0,t0,rL,tL,P0,pm,i,robot)
%   %Twist of that point
%   txdot=J0*q0ddot+J0dot*q0dot+Jm*qmddot+Jmdot*qmdot;
%
% See also: :func:`src.kinematics_dynamics.Accelerations` and :func:`src.kinematics_dynamics.Jacob`. 

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

%--- Omega ---%
%Base-spacecraft Omega
Omega0=[SkewSym(t0(1:3)), zeros(3,3);
    zeros(3,3), zeros(3,3)];
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate Omega
    Omega=zeros(6,6,robot.n_links_joints);
end
%Compute Omega
for j=1:i
    Omega(1:6,1:6,j)=[SkewSym(tL(1:3,j)), zeros(3,3);
        zeros(3,3), SkewSym(tL(1:3,j))];
end


%--- Jacobians time derivative ---%

%Base Jacobian
J0dot=[eye(3),zeros(3,3);SkewSym(r0-rx),eye(3)]*Omega0*P0+[zeros(3,3),zeros(3,3);SkewSym(t0(4:6)-tx(4:6)),zeros(3,3)]*P0;

%Pre-allocate
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    Jmdot=zeros(6,robot.n_q);
end
%Manipulator Jacobian
joints_num=0;
for j=1:i
    %If joint is not fixed
    if robot.joints(j).type~=0
        if robot.con.branch(i,j)==1
            Jmdot(1:6,robot.joints(j).q_id)=[eye(3),zeros(3,3);SkewSym(rL(1:3,j)-rx),eye(3)]*Omega(1:6,1:6,j)*pm(1:6,j)+[zeros(3,3),zeros(3,3);SkewSym(tL(4:6,j)-tx(4:6)),zeros(3,3)]*pm(1:6,j);
        else
            Jmdot(1:6,robot.joints(j).q_id)=zeros(6,1);
        end
        joints_num=joints_num+1;
    end
end

%Add zeros if required
if isempty(coder.target) %Only when not pre-allocated
    if joints_num<robot.n_q
        Jmdot(1:6,joints_num+1:robot.n_q)=zeros(6,robot.n_q-joints_num);
    end
end

end