function [tauqm,q0ddot] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,qmddot,robot) %#codegen
% This function solves the inverse dynamics problem (it obtains the
% generalized forces from the accelerations) for a manipulator with 
% a floating base.
%
% Input ->
%   wF0 -> External forces on the base-spacecraft.
%   wFm -> External forces on the manipulator links CoM.
%   Mm_tilde -> Manipulator mass matrix of composite body.
%   H0 -> Base-spacecraft inertia matrix.
%   t0 -> Base-spacecraft twist vector
%   tm -> Manipulator twist vector.
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   I0 -> Base-spacecraft inertia in inertial frame.
%   Im -> Manipulator inertia in inertial frame.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   q0dot -> Base-spacecraft velocities [angular velocity in body, linear
%   velocity in inertial].
%   qmdot -> Manipulator joint rates.
%   qmddot -> Manipulator joint accelerations.
%   robot -> Robot model
%
% Output ->
%   taum -> Manipulator joint space forces.
%   q0ddot -> Base-spacecraft acceleration.

%=== LICENSE ===%

%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.

%=== CODE ===%

%Number of links and Joints
n=robot.n_links;

%Recompute Accelerations with qddot=0
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,zeros(6,1),qmddot,robot);

%Use the inverse dynamics
[tauq0_0ddot,tauqm] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robot);

%Compute Kappa
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate kappa
    kappa=zeros(n,6);
end
for i=1:n
    kappa(i,1:6)=pm(1:6,i)'*Mm_tilde(1:6,1:6,i)*Bi0(1:6,1:6,i);
end

%Compute base-spacecraft acceleration
q0ddot=-H0\tauq0_0ddot;

%Update joint forces
for i=1:n
    if robot.joints(i).type~=0
        tauqm(robot.joints(i).q_id)=kappa(i,1:6)*q0ddot+tauqm(robot.joints(i).q_id);
    end
end

end
