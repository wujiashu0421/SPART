function [I] = NomInertia(robot)
% Computes the inertia matrix of the system with the nominal configuration, 
% i.e., qm=0.
%
% [I] = NomInertia(robot)
%
% :parameters: 
%   * robot -- Robot model (see :doc:`/Tutorial_Robot`).
%
% :return:
%   * m --  Inertia of the system with a nominal configuration qm=0
%
% See also: :func:`src.utils.TotalMass`

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

%--- Parameters ---%
R0=eye(3);
r0=zeros(3,1);
qm=zeros(robot.n_q,1);

%--- Kinematics ---%
%Kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%Diferential Kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);

%--- Dynamics ---%
%Inertias in inertial frames
[I0,Im]=I_I(R0,RL,robot);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);

%--- Extract inertia matrix ---%
I = H0(1:3,1:3);

end