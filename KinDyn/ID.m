function [tauq0,tauqm] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robot) %#codegen
% This function solves the inverse dynamics problem (it obtains the
% generalized forces from the accelerations) for a manipulator.
%
% Input ->
%   wF0 -> External forces on the base-spacecraft.
%   wFm -> External forces on the manipulator links CoM.
%   t0 -> Base-spacecraft twist vector
%   tm -> Manipulator twist vector.
%   t0dot -> Base-spacecraft twist rate vector
%   tmdot -> Manipulator twist rate vector.
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   I0 -> Base-spacecraft inertia in inertial frame.
%   Im -> Manipulator inertia in inertial frame.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   robot -> Robot model.
%
% Output ->
%   tau0 -> Base-spacecraft joint space forces.
%   taum -> Manipulator joint space forces.

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

%--- Number of links and Joints ---%
n=robot.n_links;

%--- Mdot ---%
%Base-spacecraft Mdot
Mdot0=[SkewSym(t0(1:3))*I0, zeros(3,3); zeros(3,3), zeros(3,3)];
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    Mdot=zeros(6,6,n);
end
%Manipulator Mdot
for i=1:n
    Mdot(1:6,1:6,i)=[SkewSym(tm(1:3,i))*Im(1:3,1:3,i), zeros(3,3); zeros(3,3), zeros(3,3)];
end

%--- Forces ---%
%Base-spacecraft
wq0=[I0,zeros(3,3);zeros(3,3),robot.base_link.mass*eye(3)]*t0dot+Mdot0*t0-wF0;
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    wq=zeros(6,n);
end
%Manipulator
for i=1:n
    wq(1:6,i)=[Im(1:3,1:3,i),zeros(3,3);zeros(3,3),robot.links(i).mass*eye(3)]*tmdot(1:6,i)+Mdot(1:6,1:6,i)*tm(1:6,i)-wFm(1:6,i);
end
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    wq_tilde=zeros(6,n);
end
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
tauq0=P0'*wq_tilde0;
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    tauqm=zeros(n,1);
end
%Manipulator joint forces.
for i=1:n
    tauqm(i,1)=pm(1:6,i)'*wq_tilde(1:6,i);
end

end