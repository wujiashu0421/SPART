function [t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,q0ddot,qmddot,robot) %#codegen
% Computes the accekerations (twist-rate) of a manipulator.
%
% Input ->
%   t0 -> Base-spacecraft twist vector
%   tm -> Manipulator twist vector.
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   q0dot -> Base-spacecraft velocities [angular velocity in body, linear
%   velocity in inertial].
%   qmdot -> Manipulator joint rates.
%   q0ddot -> Base-spacecraft accelerations.
%   qmddot -> Manipulator joint accelerations.
%   robot -> Robot model.
%
% Output ->
%   t0dot -> Base-spacecraft twist rate vector
%   tmdot -> Manipulator twist rate vector.

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

%--- Omega matrices ---%
%Base-Spacecraft
Omega0=[SkewSym(t0(1:3)), zeros(3,3);
    zeros(3,3), SkewSym(t0(1:3))];
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    Omegam=zeros(6,6,n);
end
%Compute Omega for manipulator
for i=1:n
    Omegam(1:6,1:6,i)=[SkewSym(tm(1:3,i)), zeros(3,3);
        zeros(3,3), SkewSym(tm(1:3,i))];
end

%--- Twist Rate ---%
%Base-spacecraft
t0dot = Omega0*P0*q0dot+P0*q0ddot;
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    tmdot=zeros(6,n);
end
%Forward recursion
for i=1:n
    
    if robot.joints(i).parent_link==0
        %First Link
        tmdot(1:6,i)=Bi0(1:6,1:6,i)*t0dot+[zeros(3,6);SkewSym(t0(4:6)-tm(4:6,i)),zeros(3,3)]*t0;
    else
        %Rest of the links
        tmdot(1:6,i)=Bij(1:6,1:6,i,robot.joints(i).parent_link)*tmdot(1:6,robot.joints(i).parent_link)+[zeros(3,6); SkewSym(tm(4:6,robot.joints(i).parent_link)-tm(4:6,i)), zeros(3,3)]*tm(1:6,robot.joints(i).parent_link);
    end
    
    %Add joint contribution
    if robot.joints(i).type~=0
        tmdot(1:6,i)=Omegam(1:6,1:6,i)*pm(1:6,i)*qmdot(i)+pm(1:6,i)*qmddot(i);
    end
    
    
end


end